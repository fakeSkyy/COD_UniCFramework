/**
 * @file test_util_crc.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <string.h>

#include "test_support.h"
#include "util_crc.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Independent reference implementations                                    */
/* ========================================================================= */

/*  The module is table-driven, so comparing it against its own table proves
 *  nothing. These two are bit-at-a-time implementations written from the
 *  parameters the header documents (reflected poly, given init, no final xor)
 *  and share no data with util_crc.c — a corrupted table entry shows up as a
 *  disagreement rather than as a self-consistent wrong answer.
 */

/**
 * @brief Bitwise reflected CRC-8 over @p UTIL_CRC8_POLY_REFLECTED.
 */
static uint8_t ref_crc8(const uint8_t* data, size_t len, uint8_t init)
{
    uint8_t crc = init;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 1u) ? (uint8_t) ((crc >> 1) ^ UTIL_CRC8_POLY_REFLECTED)
                             : (uint8_t) (crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief Bitwise reflected CRC-16 over @p UTIL_CRC16_POLY_REFLECTED.
 */
static uint16_t ref_crc16(const uint8_t* data, size_t len, uint16_t init)
{
    uint16_t crc = init;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 1u) ? (uint16_t) ((crc >> 1) ^ UTIL_CRC16_POLY_REFLECTED)
                             : (uint16_t) (crc >> 1);
        }
    }
    return crc;
}

/* ========================================================================= */
/*  Known-good values                                                        */
/* ========================================================================= */

/*  "123456789" is the CRC catalogue's standard check string. The header names
 *  the CRC-16 variant as CRC-16/MCRF4XX, whose published check value is 0x6F91;
 *  asserting it anchors this module to something outside the repository, which
 *  is the one thing a round-trip test cannot do. The CRC-8 counterpart has no
 *  catalogue entry at init 0xFF, so its expected value comes from the bitwise
 *  reference above rather than from a constant typed in here.
 */
#define CRC16_MCRF4XX_CHECK 0x6F91u

static const uint8_t check_string[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

static void test_util_crc_crc16_matches_published_check_value(void)
{
    TEST_ASSERT_EQUAL_HEX16(CRC16_MCRF4XX_CHECK,
                            UTIL_CRC16_Calc(check_string, sizeof(check_string), UTIL_CRC16_INIT));
}

static void test_util_crc_tables_agree_with_documented_polynomials(void)
{
    /* Sweep every single-byte input: that walks all 256 table entries of both
     * tables, so a single wrong entry cannot hide. */
    for (unsigned v = 0; v < 256u; v++)
    {
        uint8_t byte = (uint8_t) v;

        TEST_ASSERT_EQUAL_HEX8(ref_crc8(&byte, 1u, UTIL_CRC8_INIT),
                               UTIL_CRC8_Calc(&byte, 1u, UTIL_CRC8_INIT));
        TEST_ASSERT_EQUAL_HEX16(ref_crc16(&byte, 1u, UTIL_CRC16_INIT),
                                UTIL_CRC16_Calc(&byte, 1u, UTIL_CRC16_INIT));
    }
}

static void test_util_crc_calc_agrees_with_reference_over_long_input(void)
{
    uint8_t data[257];

    for (unsigned i = 0; i < sizeof(data); i++)
    {
        data[i] = (uint8_t) (i * 31u + 7u);
    }

    /* Every prefix length, so an off-by-one in the loop bound is caught at the
     * length where it first matters rather than only for one arbitrary size. */
    for (size_t len = 0; len <= sizeof(data); len++)
    {
        TEST_ASSERT_EQUAL_HEX8(ref_crc8(data, len, UTIL_CRC8_INIT),
                               UTIL_CRC8_Calc(data, len, UTIL_CRC8_INIT));
        TEST_ASSERT_EQUAL_HEX16(ref_crc16(data, len, UTIL_CRC16_INIT),
                                UTIL_CRC16_Calc(data, len, UTIL_CRC16_INIT));
    }
}

/* ========================================================================= */
/*  Calc: identity, seeding, NULL                                            */
/* ========================================================================= */

static void test_util_crc_calc_zero_length_returns_init_unchanged(void)
{
    const uint8_t data[4] = {0xDE, 0xAD, 0xBE, 0xEF};

    /* The empty range must be the algorithm's identity, otherwise chaining a
     * running CRC over a scattered buffer with an empty fragment would shift it. */
    TEST_ASSERT_EQUAL_HEX8(UTIL_CRC8_INIT, UTIL_CRC8_Calc(data, 0u, UTIL_CRC8_INIT));
    TEST_ASSERT_EQUAL_HEX8(0x5Au, UTIL_CRC8_Calc(data, 0u, 0x5Au));
    TEST_ASSERT_EQUAL_HEX16(UTIL_CRC16_INIT, UTIL_CRC16_Calc(data, 0u, UTIL_CRC16_INIT));
    TEST_ASSERT_EQUAL_HEX16(0x1234u, UTIL_CRC16_Calc(data, 0u, 0x1234u));
}

static void test_util_crc_calc_null_data_returns_init(void)
{
    /* The header allows NULL only at len 0; the implementation additionally
     * returns init for a NULL pointer with a non-zero length, deliberately, so
     * a caller never receives a plausible-looking checksum over unread memory. */
    TEST_ASSERT_EQUAL_HEX8(UTIL_CRC8_INIT, UTIL_CRC8_Calc(NULL, 0u, UTIL_CRC8_INIT));
    TEST_ASSERT_EQUAL_HEX8(0x42u, UTIL_CRC8_Calc(NULL, 16u, 0x42u));
    TEST_ASSERT_EQUAL_HEX16(UTIL_CRC16_INIT, UTIL_CRC16_Calc(NULL, 0u, UTIL_CRC16_INIT));
    TEST_ASSERT_EQUAL_HEX16(0x4321u, UTIL_CRC16_Calc(NULL, 16u, 0x4321u));
}

static void test_util_crc_calc_is_resumable_across_fragments(void)
{
    const uint8_t data[10] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x00, 0xFF};

    /* The header advertises passing a previous result as init to continue over a
     * scattered buffer; split at every point to confirm the split is invisible. */
    for (size_t split = 0; split <= sizeof(data); split++)
    {
        uint8_t  c8  = UTIL_CRC8_Calc(data, split, UTIL_CRC8_INIT);
        uint16_t c16 = UTIL_CRC16_Calc(data, split, UTIL_CRC16_INIT);

        TEST_ASSERT_EQUAL_HEX8(UTIL_CRC8_Calc(data, sizeof(data), UTIL_CRC8_INIT),
                               UTIL_CRC8_Calc(&data[split], sizeof(data) - split, c8));
        TEST_ASSERT_EQUAL_HEX16(UTIL_CRC16_Calc(data, sizeof(data), UTIL_CRC16_INIT),
                                UTIL_CRC16_Calc(&data[split], sizeof(data) - split, c16));
    }
}

static void test_util_crc_nonzero_init_makes_leading_zeros_matter(void)
{
    /* The stated reason for the non-zero seed: length changes made of zero bytes
     * must not be invisible. */
    const uint8_t one_zero[1]  = {0x00};
    const uint8_t two_zeros[2] = {0x00, 0x00};

    TEST_ASSERT_NOT_EQUAL(UTIL_CRC8_Calc(one_zero, 1u, UTIL_CRC8_INIT),
                          UTIL_CRC8_Calc(two_zeros, 2u, UTIL_CRC8_INIT));
    TEST_ASSERT_NOT_EQUAL(UTIL_CRC16_Calc(one_zero, 1u, UTIL_CRC16_INIT),
                          UTIL_CRC16_Calc(two_zeros, 2u, UTIL_CRC16_INIT));
}

/* ========================================================================= */
/*  Append / Verify guard clauses                                            */
/* ========================================================================= */

static void test_util_crc8_append_rejects_null_and_short_frames(void)
{
    uint8_t frame[4] = {0};

    TEST_ASSERT_FALSE(UTIL_CRC8_Append(NULL, 4u));
    TEST_ASSERT_FALSE(UTIL_CRC8_Append(NULL, 0u));
    TEST_ASSERT_FALSE(UTIL_CRC8_Append(frame, 0u));
    TEST_ASSERT_FALSE(UTIL_CRC8_Append(frame, 1u)); /* no payload left to protect */
    TEST_ASSERT_TRUE(UTIL_CRC8_Append(frame, 2u));  /* the documented minimum    */
}

static void test_util_crc8_verify_rejects_null_and_short_frames(void)
{
    uint8_t frame[4] = {0};

    TEST_ASSERT_TRUE(UTIL_CRC8_Append(frame, 4u));

    TEST_ASSERT_FALSE(UTIL_CRC8_Verify(NULL, 4u));
    TEST_ASSERT_FALSE(UTIL_CRC8_Verify(frame, 0u));
    TEST_ASSERT_FALSE(UTIL_CRC8_Verify(frame, 1u));
}

static void test_util_crc16_append_rejects_null_and_short_frames(void)
{
    uint8_t frame[5] = {0};

    TEST_ASSERT_FALSE(UTIL_CRC16_Append(NULL, 5u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Append(frame, 0u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Append(frame, 1u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Append(frame, 2u)); /* two checksum bytes, no payload */
    TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, 3u));
}

static void test_util_crc16_verify_rejects_null_and_short_frames(void)
{
    uint8_t frame[5] = {0};

    TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, 5u));

    TEST_ASSERT_FALSE(UTIL_CRC16_Verify(NULL, 5u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Verify(frame, 0u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Verify(frame, 1u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Verify(frame, 2u));
}

static void test_util_crc_append_rejection_leaves_frame_untouched(void)
{
    uint8_t frame[4] = {0xAA, 0xBB, 0xCC, 0xDD};

    TEST_ASSERT_FALSE(UTIL_CRC8_Append(frame, 1u));
    TEST_ASSERT_FALSE(UTIL_CRC16_Append(frame, 2u));

    /* A refused Append must not have written a partial checksum somewhere. */
    TEST_ASSERT_EQUAL_HEX8(0xAA, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, frame[1]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, frame[2]);
    TEST_ASSERT_EQUAL_HEX8(0xDD, frame[3]);
}

/* ========================================================================= */
/*  Append / Verify round-trip                                               */
/* ========================================================================= */

static void test_util_crc8_append_verify_round_trip(void)
{
    uint8_t frame[16];

    for (size_t frame_len = 2u; frame_len <= sizeof(frame); frame_len++)
    {
        for (size_t i = 0; i < frame_len - 1u; i++)
        {
            frame[i] = (uint8_t) (i * 17u + 3u);
        }
        frame[frame_len - 1u] = 0xA5; /* pre-poisoned, so a no-op Append fails */

        TEST_ASSERT_TRUE(UTIL_CRC8_Append(frame, frame_len));
        TEST_ASSERT_TRUE(UTIL_CRC8_Verify(frame, frame_len));

        /* The checksum must cover exactly frame_len-1 payload bytes; computing
         * it over the whole frame would fold the old checksum back in. */
        TEST_ASSERT_EQUAL_HEX8(ref_crc8(frame, frame_len - 1u, UTIL_CRC8_INIT),
                               frame[frame_len - 1u]);
    }
}

static void test_util_crc16_append_verify_round_trip(void)
{
    uint8_t frame[16];

    for (size_t frame_len = 3u; frame_len <= sizeof(frame); frame_len++)
    {
        for (size_t i = 0; i < frame_len - 2u; i++)
        {
            frame[i] = (uint8_t) (i * 29u + 11u);
        }
        frame[frame_len - 2u] = 0x5A;
        frame[frame_len - 1u] = 0xA5;

        TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, frame_len));
        TEST_ASSERT_TRUE(UTIL_CRC16_Verify(frame, frame_len));

        uint16_t expected = ref_crc16(frame, frame_len - 2u, UTIL_CRC16_INIT);
        TEST_ASSERT_EQUAL_HEX16(
            expected, (uint16_t) (frame[frame_len - 2u] | ((uint16_t) frame[frame_len - 1u] << 8)));
    }
}

static void test_util_crc16_append_is_little_endian_on_the_wire(void)
{
    /* Byte order is a wire-format promise, not the host's choice; asserting the
     * two bytes individually is the only way to catch a host-endian cast. */
    uint8_t frame[11];

    memcpy(frame, check_string, sizeof(check_string));
    TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, sizeof(frame)));

    TEST_ASSERT_EQUAL_HEX8((uint8_t) (CRC16_MCRF4XX_CHECK & 0xFFu), frame[9]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t) (CRC16_MCRF4XX_CHECK >> 8), frame[10]);
}

static void test_util_crc_append_preserves_payload(void)
{
    uint8_t frame[8];
    uint8_t payload[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

    memcpy(frame, payload, sizeof(payload));
    frame[6] = 0;
    frame[7] = 0;
    TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, frame, sizeof(payload));

    memcpy(frame, payload, sizeof(payload));
    frame[6] = 0;
    TEST_ASSERT_TRUE(UTIL_CRC8_Append(frame, 7u));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, frame, sizeof(payload));
}

static void test_util_crc8_verify_rejects_a_wrong_checksum_byte(void)
{
    uint8_t frame[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x00};

    TEST_ASSERT_TRUE(UTIL_CRC8_Append(frame, sizeof(frame)));
    frame[5] = (uint8_t) (frame[5] + 1u);
    TEST_ASSERT_FALSE(UTIL_CRC8_Verify(frame, sizeof(frame)));
}

static void test_util_crc16_verify_rejects_a_swapped_checksum(void)
{
    uint8_t frame[7] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00};

    TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, sizeof(frame)));

    /* Byte-swapping the checksum is the signature of a MODBUS-order sender; it
     * must be rejected unless the two bytes happen to be equal. */
    if (frame[5] != frame[6])
    {
        uint8_t tmp = frame[5];
        frame[5]    = frame[6];
        frame[6]    = tmp;
        TEST_ASSERT_FALSE(UTIL_CRC16_Verify(frame, sizeof(frame)));
    }
}

/* ========================================================================= */
/*  Corruption detection                                                     */
/* ========================================================================= */

static void test_util_crc8_detects_every_single_bit_flip(void)
{
    uint8_t frame[12];
    uint8_t good[12];

    for (unsigned i = 0; i < sizeof(good); i++)
    {
        good[i] = (uint8_t) (i * 37u + 5u);
    }
    memcpy(frame, good, sizeof(frame));
    TEST_ASSERT_TRUE(UTIL_CRC8_Append(frame, sizeof(frame)));
    memcpy(good, frame, sizeof(good));

    /* A generator polynomial with more than one term cannot divide x^k, so every
     * single-bit error is detectable at any frame length — including a flip in
     * the checksum byte itself. Anything that slips through is a real defect. */
    for (unsigned byte = 0; byte < sizeof(frame); byte++)
    {
        for (unsigned bit = 0; bit < 8u; bit++)
        {
            memcpy(frame, good, sizeof(frame));
            frame[byte] ^= (uint8_t) (1u << bit);
            TEST_ASSERT_FALSE_MESSAGE(UTIL_CRC8_Verify(frame, sizeof(frame)),
                                      "single-bit flip accepted by CRC-8");
        }
    }
}

static void test_util_crc16_detects_every_single_bit_flip(void)
{
    uint8_t frame[12];
    uint8_t good[12];

    for (unsigned i = 0; i < sizeof(good); i++)
    {
        good[i] = (uint8_t) (i * 53u + 9u);
    }
    memcpy(frame, good, sizeof(frame));
    TEST_ASSERT_TRUE(UTIL_CRC16_Append(frame, sizeof(frame)));
    memcpy(good, frame, sizeof(good));

    for (unsigned byte = 0; byte < sizeof(frame); byte++)
    {
        for (unsigned bit = 0; bit < 8u; bit++)
        {
            memcpy(frame, good, sizeof(frame));
            frame[byte] ^= (uint8_t) (1u << bit);
            TEST_ASSERT_FALSE_MESSAGE(UTIL_CRC16_Verify(frame, sizeof(frame)),
                                      "single-bit flip accepted by CRC-16");
        }
    }
}

static void test_util_crc_detects_adjacent_byte_transposition(void)
{
    /* A checksum that merely summed bytes would pass this; both must not. */
    uint8_t f8[6]  = {0x10, 0x20, 0x30, 0x40, 0x50, 0x00};
    uint8_t f16[7] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x00, 0x00};

    TEST_ASSERT_TRUE(UTIL_CRC8_Append(f8, sizeof(f8)));
    TEST_ASSERT_TRUE(UTIL_CRC16_Append(f16, sizeof(f16)));

    uint8_t tmp = f8[1];
    f8[1]       = f8[2];
    f8[2]       = tmp;
    TEST_ASSERT_FALSE(UTIL_CRC8_Verify(f8, sizeof(f8)));

    tmp    = f16[1];
    f16[1] = f16[2];
    f16[2] = tmp;
    TEST_ASSERT_FALSE(UTIL_CRC16_Verify(f16, sizeof(f16)));
}

static void test_util_crc_verify_rejects_a_truncated_frame(void)
{
    /* Re-verifying a frame one byte short reads the last payload byte as the
     * checksum; the non-zero seed is what makes this detectable. */
    uint8_t f8[8]  = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01, 0x02, 0x00};
    uint8_t f16[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01, 0x00, 0x00};

    TEST_ASSERT_TRUE(UTIL_CRC8_Append(f8, sizeof(f8)));
    TEST_ASSERT_FALSE(UTIL_CRC8_Verify(f8, sizeof(f8) - 1u));

    TEST_ASSERT_TRUE(UTIL_CRC16_Append(f16, sizeof(f16)));
    TEST_ASSERT_FALSE(UTIL_CRC16_Verify(f16, sizeof(f16) - 1u));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_crc_crc16_matches_published_check_value);
    RUN_TEST(test_util_crc_tables_agree_with_documented_polynomials);
    RUN_TEST(test_util_crc_calc_agrees_with_reference_over_long_input);

    RUN_TEST(test_util_crc_calc_zero_length_returns_init_unchanged);
    RUN_TEST(test_util_crc_calc_null_data_returns_init);
    RUN_TEST(test_util_crc_calc_is_resumable_across_fragments);
    RUN_TEST(test_util_crc_nonzero_init_makes_leading_zeros_matter);

    RUN_TEST(test_util_crc8_append_rejects_null_and_short_frames);
    RUN_TEST(test_util_crc8_verify_rejects_null_and_short_frames);
    RUN_TEST(test_util_crc16_append_rejects_null_and_short_frames);
    RUN_TEST(test_util_crc16_verify_rejects_null_and_short_frames);
    RUN_TEST(test_util_crc_append_rejection_leaves_frame_untouched);

    RUN_TEST(test_util_crc8_append_verify_round_trip);
    RUN_TEST(test_util_crc16_append_verify_round_trip);
    RUN_TEST(test_util_crc16_append_is_little_endian_on_the_wire);
    RUN_TEST(test_util_crc_append_preserves_payload);
    RUN_TEST(test_util_crc8_verify_rejects_a_wrong_checksum_byte);
    RUN_TEST(test_util_crc16_verify_rejects_a_swapped_checksum);

    RUN_TEST(test_util_crc8_detects_every_single_bit_flip);
    RUN_TEST(test_util_crc16_detects_every_single_bit_flip);
    RUN_TEST(test_util_crc_detects_adjacent_byte_transposition);
    RUN_TEST(test_util_crc_verify_rejects_a_truncated_frame);

    return UNITY_END();
}
