/**
 * @file dev_ws2812.h
 * @author Gao Xing
 * @date 2026/8/12
 * @version 1.0
 */

#ifndef DEV_WS2812_H
#define DEV_WS2812_H

#include <stdbool.h>
#include <stdint.h>

#include "dev_watchdog.h"
#include "plat_spi.h"

/* ==========================================================================
 * Driving a WS2812 from a SPI transmitter
 * ==========================================================================
 *
 * A WS2812 has no clock line. It reads a single data wire and distinguishes a 0
 * from a 1 by how long the line stays high within a ~1.25 us bit slot: about
 * 0.4 us for a 0, about 0.8 us for a 1. The low time afterwards only has to keep
 * the total slot from looking like a reset.
 *
 * @par Why SPI rather than a GPIO and a delay loop
 * Bit-banging means holding a ~30 us critical section per LED with interrupts off —
 * long enough to make a kilohertz control loop miss its deadline, and the timing
 * would then depend on not being interrupted rather than on hardware. Feeding the
 * data line from a shift register makes the waveform a property of the peripheral,
 * so a preemption between bytes is harmless.
 *
 * @par One SPI byte per WS2812 bit
 * Each colour bit becomes one 8-bit SPI frame whose high bits form the pulse:
 *
 *     WS2812 0  ->  0x60 = 0b01100000   high for 2 of 8 SPI bits
 *     WS2812 1  ->  0x78 = 0b01111000   high for 4 of 8 SPI bits
 *
 * One SPI bit is therefore an eighth of a bit slot, which fixes the required clock
 * rate: see DEV_WS2812_SCK_HZ. 24 colour bits cost 24 bytes per LED.
 *
 * @par Why not pack three SPI bits per colour bit
 * It is tempting — 24 bits x 3 is exactly 9 bytes, a third of the RAM — and it is
 * what this driver did first. It did not work. Packing forces the SPI clock to be
 * one third of a bit slot instead of an eighth, which puts the two pulse widths at
 * the edges of the datasheet windows rather than near their centres; a real LED
 * misread them, so a frame of all-zero codes failed to turn it off and colours came
 * out impure. Every symbol also straddles byte boundaries, making the encoder a bit
 * cursor rather than a lookup.
 *
 * The byte-per-bit form costs 24 bytes instead of 9 and buys pulse widths near the
 * centre of the window, a one-line encoder, and a first and last SPI bit that are
 * always low — so consecutive frames cannot merge two pulses into one. That trade
 * is settled; do not re-pack it to save 15 bytes per LED.
 * ==========================================================================
 */

/**
 * @brief SPI clock this encoding requires, in hertz.
 *
 * Eight SPI bits per 1.25 us bit slot puts one SPI bit at ~167 ns, so the codes
 * above give 333 ns and 667 ns of high time — both near the middle of the part's
 * tolerance (T0H 250..550 ns, T1H 650..950 ns).
 *
 * The rate is a property of this encoding, not of any board, so it is stated here
 * as the contract a caller's SPI instance has to meet. Whoever creates that
 * instance is responsible for its kernel clock and prescaler; nothing in this
 * driver can see either, and a wrong rate is silent — the transfer still succeeds
 * and the light is simply wrong. Anything within roughly +/-15% still lands inside
 * both windows.
 */
#define DEV_WS2812_SCK_HZ 6000000u

/** @brief SPI bytes one LED's colour occupies once encoded: 24 bits, one each. */
#define DEV_WS2812_BYTES_PER_LED 24u

/**
 * @brief Trailing all-zero bytes that latch a frame.
 *
 * @par Why this is not optional
 * A WS2812 latches what it has received once the line has been idle for tens of
 * microseconds, and nothing guarantees the data line parks low after a transfer —
 * on many SPI peripherals the pin returns to whatever the alternate-function output
 * leaves, which is not a documented level. An earlier version of this driver
 * assumed silence followed each frame and so never latched at all: the LED kept
 * displaying its previous contents no matter what was sent.
 *
 * 100 zero bytes is 800 low SPI bits, about 133 us at the required clock —
 * comfortably past the ~50 us reset threshold — and it rides in the same transfer,
 * so nothing can be scheduled between the last colour bit and the gap that commits
 * it.
 */
#define DEV_WS2812_LATCH_BYTES 100u

/**
 * @brief One WS2812 chain on one SPI instance.
 *
 * @par Why the buffer is the caller's
 * Its size is the one thing that varies per application — a single indicator or a
 * strip of sixty — and a driver that allocated it would either cap the strip or
 * waste RAM. It also has to satisfy placement rules only the caller knows: a
 * DMA-driven SPI instance may not be able to reach every RAM region, and the
 * platform layer rejects a buffer its backend cannot read rather than transferring
 * nothing.
 *
 * Treat the fields as opaque.
 */
typedef struct
{
    SPI_Instance_s* spi; /**< Transmit-only SPI instance driving the data line. */

    uint8_t* buf;   /**< Encoded waveform plus the trailing latch bytes.       */
    uint16_t bytes; /**< Its size.                                            */
    uint16_t count; /**< LEDs the buffer can hold.                            */
    uint16_t len;   /**< Bytes actually sent: count * 24 + latch.             */

    bool initialized;

    /**
     * @brief Liveness node, kicked by every successful Show.
     *
     * Registered with a zero timeout, because a strip is written and never read:
     * it cannot go silent, so there is nothing to time out on. What the node
     * still buys is presence in the one report that lists every device, and a
     * fail_count that a caller can watch if it starts checking Show's return.
     */
    DEV_Watchdog_s wd;
} DEV_WS2812_s;

/**
 * @brief Buffer size a chain of @p leds needs, latch included.
 *
 * Use this to declare the buffer, so the latch allowance cannot be forgotten:
 *
 *     static uint8_t buf[DEV_WS2812_BUF_BYTES(1u)];
 *
 * @param leds  Number of LEDs in the chain.
 */
#define DEV_WS2812_BUF_BYTES(leds) ((leds) * DEV_WS2812_BYTES_PER_LED + DEV_WS2812_LATCH_BYTES)

/**
 * @brief Bind a chain to an SPI instance and blank it.
 *
 * Does not transmit: the strip keeps whatever it was last sent, including across a
 * warm reset. Call Show once after this if a defined state at startup matters.
 *
 * @param dev    Storage to initialize. Must outlive every use.
 * @param spi    Transmit-only SPI instance whose data output drives the wire. Its
 *               clock must be DEV_WS2812_SCK_HZ.
 * @param buf    Waveform buffer, at least DEV_WS2812_BUF_BYTES(count) bytes. Must
 *               outlive @p dev, and must be readable by @p spi's backend — see the
 *               note on the struct if that instance is DMA-driven.
 * @param bytes  Size of @p buf.
 * @param count  Number of LEDs in the chain.
 * @return true on success; false on a NULL argument, a zero count, or a buffer too
 *         small — in which case @p dev is left uninitialized and every later call
 *         is a no-op.
 */
bool DEV_WS2812_Init(DEV_WS2812_s* dev, SPI_Instance_s* spi, uint8_t* buf, uint16_t bytes,
                     uint16_t count);

/**
 * @brief Set one LED's colour in the buffer, without transmitting.
 *
 * Encodes into the buffer only. Setting several LEDs and then calling Show once is
 * both faster and correct — a Show per LED makes a strip visibly ripple.
 *
 * @param dev    Initialized chain.
 * @param index  LED position, 0 nearest the controller. Out of range is ignored.
 * @param r      Red, 0-255.
 * @param g      Green, 0-255.
 * @param b      Blue, 0-255.
 */
void DEV_WS2812_SetPixel(DEV_WS2812_s* dev, uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set every LED to one colour, without transmitting.
 *
 * @param dev  Initialized chain.
 * @param r    Red, 0-255.
 * @param g    Green, 0-255.
 * @param b    Blue, 0-255.
 */
void DEV_WS2812_SetAll(DEV_WS2812_s* dev, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Send the buffer to the strip and latch it.
 *
 * @par Blocking, and why that is right here
 * One LED plus its latch is 124 bytes — about 165 us at the required clock, most of
 * it the latch. An asynchronous send would need the buffer left untouched until the
 * callback, so a caller updating colours could not simply overwrite it, and for a
 * status indicator that complexity buys nothing.
 *
 * The latch is part of the transfer, so on return the frame is committed and the
 * caller may immediately change colours again.
 *
 * @param dev  Initialized chain.
 * @return true when the whole buffer went out. false on an uninitialized chain, or
 *         when the SPI transfer failed — including because another device holds the
 *         bus.
 */
bool DEV_WS2812_Show(DEV_WS2812_s* dev);

#endif /* DEV_WS2812_H */
