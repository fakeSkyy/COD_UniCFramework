/**
 * @file dev_ws2812.c
 * @author Gao Xing
 * @date 2026/8/12
 * @version 1.0
 */

#include "dev_ws2812.h"

#include <stddef.h> /* NULL */

/* ==========================================================================
 * The timing this encoding assumes
 * ==========================================================================
 *
 * One SPI byte per WS2812 bit, so one SPI frame is one bit slot and the high bits
 * within the byte are the pulse. The part cares about the high time:
 *
 *   T0H  0.40 us +/- 0.15  ->  acceptable 0.25 .. 0.55 us
 *   T1H  0.80 us +/- 0.15  ->  acceptable 0.65 .. 0.95 us
 *
 * At DEV_WS2812_SCK_HZ one SPI bit is ~167 ns, so:
 *
 *   0x60 = 0b01100000  ->  2 bits high  ->  T0H = 333 ns   (centre of window)
 *   0x78 = 0b01111000  ->  4 bits high  ->  T1H = 667 ns   (inside window)
 *   slot = 8 bits                       ->        1333 ns  (nominal 1250)
 *
 * Both codes begin and end with a low SPI bit. That is deliberate: it means back to
 * back frames cannot merge two pulses into one, which a code starting high can do
 * when the previous slot ended high.
 *
 * @par Why the clock rate is not checked here
 * It cannot be. The SPI instance arrives already configured, and its kernel clock
 * and prescaler belong to whoever created it — neither is visible from this layer.
 * A wrong rate is silent at every level: the transfer succeeds, Show returns true,
 * and the light is simply wrong. DEV_WS2812_SCK_HZ states the requirement so the
 * creating code has something to be checked against; if an LED shows a wrong colour
 * or ignores data, verify the actual clock first and these codes second.
 * ==========================================================================
 */

/** @brief SPI byte for a WS2812 zero: 2 of 8 bits high. */
#define WS2812_CODE_0 0x60u

/** @brief SPI byte for a WS2812 one: 4 of 8 bits high. */
#define WS2812_CODE_1 0x78u

/** @brief Colour bits per LED: eight each of green, red, blue. */
#define WS2812_COLOUR_BITS 24u

/* One byte per colour bit, so these must be equal. Checked rather than trusted: a
 * mismatch would overrun the buffer by a fixed amount per LED, which corrupts
 * whatever the linker placed next rather than failing visibly. */
_Static_assert(WS2812_COLOUR_BITS == DEV_WS2812_BYTES_PER_LED,
               "DEV_WS2812_BYTES_PER_LED must be one byte per colour bit");

/**
 * @brief Blocking-send budget.
 *
 * One LED plus latch is ~165 us at the required clock, and a long strip scales
 * linearly, so
 * 50 ms is far more than any plausible chain needs. It exists to bound a hung
 * peripheral, not to pace anything.
 */
#define WS2812_TIMEOUT_MS 50u

/**
 * @brief Encode 24 colour bits into 24 SPI bytes, MSB first.
 *
 * @param out    Destination, at least DEV_WS2812_BYTES_PER_LED bytes.
 * @param green  Green channel.
 * @param red    Red channel.
 * @param blue   Blue channel.
 */
static void encode_pixel(uint8_t* out, uint8_t green, uint8_t red, uint8_t blue)
{
    /* GRB, not RGB. The WS2812 shifts green first; sending RGB gives a strip that
     * works but has red and green swapped, which reads as a wiring fault. */
    const uint8_t channel[3] = {green, red, blue};

    unsigned pos = 0u;

    for (unsigned ch = 0u; ch < 3u; ch++)
    {
        for (int bit = 7; bit >= 0; bit--)
        {
            out[pos] = ((channel[ch] >> (unsigned) bit) & 1u) ? WS2812_CODE_1 : WS2812_CODE_0;
            pos++;
        }
    }
}

bool DEV_WS2812_Init(DEV_WS2812_s* dev, SPI_Instance_s* spi, uint8_t* buf, uint16_t bytes,
                     uint16_t count)
{
    if (dev == NULL || spi == NULL || buf == NULL || count == 0u)
    {
        return false;
    }

    /* Computed in 32 bits: count * 24 + 100 overflows uint16_t above ~2730 LEDs,
     * and a wrapped product would compare as "big enough" and then be written
     * past. */
    const uint32_t needed = (uint32_t) count * DEV_WS2812_BYTES_PER_LED + DEV_WS2812_LATCH_BYTES;

    if (bytes < needed)
    {
        return false;
    }

    dev->spi         = spi;
    dev->buf         = buf;
    dev->bytes       = bytes;
    dev->count       = count;
    dev->len         = (uint16_t) needed;
    dev->initialized = true;

    /* Zero timeout: write-only, so silence is not evidence of anything. */
    DEV_Watchdog_Init(&dev->wd, "led", 0u);

    /* The latch region is written once, here, and never touched again — the pixel
     * writes below stay inside count * 24 bytes. */
    for (uint32_t i = (uint32_t) count * DEV_WS2812_BYTES_PER_LED; i < needed; i++)
    {
        buf[i] = 0u;
    }

    DEV_WS2812_SetAll(dev, 0u, 0u, 0u);

    return true;
}

void DEV_WS2812_SetPixel(DEV_WS2812_s* dev, uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (dev == NULL || !dev->initialized || index >= dev->count)
    {
        return;
    }

    encode_pixel(&dev->buf[(uint32_t) index * DEV_WS2812_BYTES_PER_LED], g, r, b);
}

void DEV_WS2812_SetAll(DEV_WS2812_s* dev, uint8_t r, uint8_t g, uint8_t b)
{
    if (dev == NULL || !dev->initialized)
    {
        return;
    }

    /* First pixel encoded, then copied: encoding is 24 iterations per LED, so for a
     * strip the copy is the cheaper half. Written as a loop rather than memcpy to
     * keep string.h out of a driver that needs nothing else from it. */
    encode_pixel(dev->buf, g, r, b);

    for (uint32_t i = 1u; i < dev->count; i++)
    {
        uint8_t* dst = &dev->buf[i * DEV_WS2812_BYTES_PER_LED];

        for (unsigned j = 0u; j < DEV_WS2812_BYTES_PER_LED; j++)
        {
            dst[j] = dev->buf[j];
        }
    }
}

bool DEV_WS2812_Show(DEV_WS2812_s* dev)
{
    if (dev == NULL || !dev->initialized)
    {
        return false;
    }

    /* Data and latch in one transfer, so nothing can be scheduled between the last
     * colour bit and the reset gap that commits it. */
    return PLAT_SPI_Send(dev->spi, dev->buf, dev->len, WS2812_TIMEOUT_MS);
}
