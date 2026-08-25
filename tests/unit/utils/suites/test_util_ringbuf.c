/**
 * @file test_util_ringbuf.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <string.h>

#include "test_support.h"
#include "util_ringbuf.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Fixtures                                                                 */
/* ========================================================================= */

/*  Capacity semantics, established from the code rather than assumed:
 *  Count() is (head - tail) & mask and Free() is capacity - Count() - 1, so one
 *  slot is permanently reserved and the usable depth is capacity - 1. With
 *  CAP == 8 that is 7 bytes, and IsFull() is true at 7 — this is what the
 *  header's "one slot is reserved to distinguish full from empty" means in
 *  numbers, and it is the figure every test below is written against.
 */
#define CAP 8u
#define USE (CAP - 1u)

static UTIL_RingBuf_s rb;
static uint8_t        storage[CAP];

/**
 * @brief Re-init the shared fixture with poisoned storage.
 *
 * The poison matters: a Get that reports success without having written its
 * output would otherwise be masked by a zeroed buffer.
 */
static void fixture_init(void)
{
    memset(storage, 0xA5, sizeof(storage));
    UTIL_RingBuf_Init(&rb, storage, CAP);
}

/**
 * @brief Assert the four observers agree with an expected occupancy.
 */
static void expect_count(uint16_t count)
{
    TEST_ASSERT_EQUAL_UINT16(count, UTIL_RingBuf_Count(&rb));
    TEST_ASSERT_EQUAL_UINT16((uint16_t) (USE - count), UTIL_RingBuf_Free(&rb));
    TEST_ASSERT_EQUAL(count == 0u, UTIL_RingBuf_IsEmpty(&rb));
    TEST_ASSERT_EQUAL(count == USE, UTIL_RingBuf_IsFull(&rb));
}

/* ========================================================================= */
/*  Init and empty state                                                     */
/* ========================================================================= */

static void test_util_ringbuf_init_sets_empty_state_and_mask(void)
{
    fixture_init();

    TEST_ASSERT_EQUAL_PTR(storage, rb.buffer);
    TEST_ASSERT_EQUAL_UINT16(CAP, rb.capacity);
    TEST_ASSERT_EQUAL_UINT16(CAP - 1u, rb.mask);
    TEST_ASSERT_EQUAL_UINT16(0u, rb.head);
    TEST_ASSERT_EQUAL_UINT16(0u, rb.tail);
    expect_count(0u);
}

static void test_util_ringbuf_get_on_empty_is_refused_and_writes_nothing(void)
{
    uint8_t out = 0x5Cu;

    fixture_init();

    TEST_ASSERT_FALSE(UTIL_RingBuf_Get(&rb, &out));
    TEST_ASSERT_EQUAL_HEX8(0x5Cu, out); /* destination must be left alone */
    expect_count(0u);
}

static void test_util_ringbuf_getn_on_empty_returns_zero(void)
{
    uint8_t out[CAP];

    fixture_init();
    memset(out, 0x3Cu, sizeof(out));

    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_RingBuf_GetN(&rb, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8(0x3Cu, out[0]);
    expect_count(0u);
}

/* ========================================================================= */
/*  Single-byte Put / Get                                                    */
/* ========================================================================= */

static void test_util_ringbuf_put_get_preserves_fifo_order(void)
{
    static const uint8_t in[USE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

    fixture_init();

    for (uint16_t i = 0; i < USE; i++)
    {
        TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, in[i]));
        expect_count((uint16_t) (i + 1u));
    }

    for (uint16_t i = 0; i < USE; i++)
    {
        uint8_t out = 0;
        TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&rb, &out));
        TEST_ASSERT_EQUAL_HEX8(in[i], out);
        expect_count((uint16_t) (USE - i - 1u));
    }

    TEST_ASSERT_TRUE(UTIL_RingBuf_IsEmpty(&rb));
}

static void test_util_ringbuf_put_refused_when_full_and_drops_the_byte(void)
{
    fixture_init();

    /* capacity - 1 bytes fit, not capacity: the reserved slot is what makes
     * head == tail mean empty rather than ambiguous. */
    for (uint16_t i = 0; i < USE; i++)
    {
        TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, (uint8_t) (0x10u + i)));
    }
    expect_count(USE);

    TEST_ASSERT_FALSE(UTIL_RingBuf_Put(&rb, 0xFFu));
    TEST_ASSERT_FALSE(UTIL_RingBuf_Put(&rb, 0xFEu));
    expect_count(USE); /* a refused Put must not have advanced head */

    /* The dropped bytes must not appear in the stream, and the oldest byte must
     * not have been overwritten by them. */
    for (uint16_t i = 0; i < USE; i++)
    {
        uint8_t out = 0;
        TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&rb, &out));
        TEST_ASSERT_EQUAL_HEX8((uint8_t) (0x10u + i), out);
    }
    uint8_t drained = 0;
    TEST_ASSERT_FALSE(UTIL_RingBuf_Get(&rb, &drained));
}

static void test_util_ringbuf_last_storage_slot_is_never_written(void)
{
    fixture_init();

    /* With head starting at 0, filling to USE writes indices 0..USE-1, so the
     * reserved slot is observable as untouched poison. Documents the reservation
     * as a storage fact, not just an arithmetic one. */
    for (uint16_t i = 0; i < USE; i++)
    {
        TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, 0x11u));
    }
    TEST_ASSERT_EQUAL_HEX8(0xA5u, storage[CAP - 1u]);
}

/* ========================================================================= */
/*  Bulk PutN / GetN                                                         */
/* ========================================================================= */

static void test_util_ringbuf_putn_getn_full_transfer(void)
{
    static const uint8_t in[USE] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6};
    uint8_t              out[USE];

    fixture_init();

    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_PutN(&rb, in, USE));
    expect_count(USE);

    memset(out, 0, sizeof(out));
    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_GetN(&rb, out, USE));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, USE);
    expect_count(0u);
}

static void test_util_ringbuf_putn_zero_length_is_a_no_op(void)
{
    static const uint8_t in[4] = {1, 2, 3, 4};

    fixture_init();

    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_RingBuf_PutN(&rb, in, 0u));
    expect_count(0u);
    TEST_ASSERT_EQUAL_HEX8(0xA5u, storage[0]); /* nothing stored */
}

static void test_util_ringbuf_getn_zero_length_is_a_no_op(void)
{
    uint8_t out[2] = {0x77u, 0x77u};

    fixture_init();
    TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, 0x42u));

    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_RingBuf_GetN(&rb, out, 0u));
    TEST_ASSERT_EQUAL_HEX8(0x77u, out[0]);
    expect_count(1u); /* the byte is still queued */
}

static void test_util_ringbuf_putn_clamps_to_free_space(void)
{
    static const uint8_t in[16] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
                                   0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F};
    uint8_t              out[16];

    fixture_init();

    /* Offering more than fits must store the leading Free() bytes and report how
     * many, so the caller knows the tail of its payload was dropped. */
    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_PutN(&rb, in, sizeof(in)));
    expect_count(USE);

    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_GetN(&rb, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, USE); /* prefix kept, not the suffix */
}

static void test_util_ringbuf_putn_on_full_stores_nothing(void)
{
    static const uint8_t in[USE + 4u] = {0xE0, 0xE1, 0xE2, 0xE3};

    fixture_init();
    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_PutN(&rb, in, USE + 4u));

    /* Clamping to a Free() of zero must be a no-op, not a wrapped overwrite of
     * the oldest data. */
    uint16_t head_before = rb.head;
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_RingBuf_PutN(&rb, in, sizeof(in)));
    TEST_ASSERT_EQUAL_UINT16(head_before, rb.head);
    expect_count(USE);
}

static void test_util_ringbuf_putn_partial_fill_then_clamped_second_call(void)
{
    static const uint8_t first[3]  = {0x01, 0x02, 0x03};
    static const uint8_t second[6] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
    static const uint8_t expect[7] = {0x01, 0x02, 0x03, 0x11, 0x12, 0x13, 0x14};
    uint8_t              out[USE];

    fixture_init();

    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_PutN(&rb, first, sizeof(first)));
    /* Free is now 4, so only four of the six offered bytes may be taken. */
    TEST_ASSERT_EQUAL_UINT16(4u, UTIL_RingBuf_PutN(&rb, second, sizeof(second)));
    expect_count(USE);

    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_GetN(&rb, out, USE));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out, USE);
}

static void test_util_ringbuf_getn_clamps_to_available(void)
{
    static const uint8_t in[3] = {0x2A, 0x2B, 0x2C};
    uint8_t              out[USE];

    fixture_init();
    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_PutN(&rb, in, sizeof(in)));

    memset(out, 0x99u, sizeof(out));
    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_GetN(&rb, out, USE));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, 3u);

    /* Bytes past the returned count must be untouched — a caller sizing its
     * parse on the return value has to be able to trust the rest is its own. */
    TEST_ASSERT_EQUAL_HEX8(0x99u, out[3]);
    expect_count(0u);
}

static void test_util_ringbuf_getn_drains_incrementally(void)
{
    static const uint8_t in[USE] = {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66};
    uint8_t              out[USE];

    fixture_init();
    TEST_ASSERT_EQUAL_UINT16(USE, UTIL_RingBuf_PutN(&rb, in, USE));

    TEST_ASSERT_EQUAL_UINT16(2u, UTIL_RingBuf_GetN(&rb, &out[0], 2u));
    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_GetN(&rb, &out[2], 3u));
    TEST_ASSERT_EQUAL_UINT16(2u, UTIL_RingBuf_GetN(&rb, &out[5], 9u)); /* clamped */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, USE);
    expect_count(0u);
}

static void test_util_ringbuf_bulk_and_single_apis_interoperate(void)
{
    static const uint8_t in[4]     = {0x71, 0x72, 0x73, 0x74};
    static const uint8_t expect[6] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75};
    uint8_t              out[6];
    uint8_t              one = 0;

    fixture_init();

    TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, 0x70u));
    TEST_ASSERT_EQUAL_UINT16(4u, UTIL_RingBuf_PutN(&rb, in, sizeof(in)));
    TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, 0x75u));
    expect_count(6u);

    TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&rb, &one));
    TEST_ASSERT_EQUAL_HEX8(0x70u, one);
    TEST_ASSERT_EQUAL_UINT16(5u, UTIL_RingBuf_GetN(&rb, &out[1], 5u));
    out[0] = one;
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out, sizeof(expect));
}

/* ========================================================================= */
/*  Wrap-around                                                              */
/* ========================================================================= */

static void test_util_ringbuf_single_byte_wrap_preserves_data(void)
{
    fixture_init();

    /* Cycle far more bytes than the storage holds so head and tail each cross
     * the end of the buffer several times. */
    for (unsigned n = 0; n < 200u; n++)
    {
        uint8_t out = 0;
        TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, (uint8_t) n));
        TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&rb, &out));
        TEST_ASSERT_EQUAL_HEX8((uint8_t) n, out);
        expect_count(0u);
    }
}

static void test_util_ringbuf_putn_wraps_across_the_buffer_end(void)
{
    static const uint8_t first[5]  = {0x01, 0x02, 0x03, 0x04, 0x05};
    static const uint8_t second[5] = {0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t              out[5];

    fixture_init();

    /* Advance head to 5, then drain 4 so tail is 4 and Free is 6: the next PutN
     * of 5 bytes must split across index 7 -> 0, which is the case a memcpy-based
     * implementation gets wrong and a per-byte masked loop gets right. */
    TEST_ASSERT_EQUAL_UINT16(5u, UTIL_RingBuf_PutN(&rb, first, sizeof(first)));
    TEST_ASSERT_EQUAL_UINT16(4u, UTIL_RingBuf_GetN(&rb, out, 4u));
    TEST_ASSERT_EQUAL_UINT16(1u, UTIL_RingBuf_Count(&rb));

    TEST_ASSERT_EQUAL_UINT16(5u, UTIL_RingBuf_PutN(&rb, second, sizeof(second)));
    expect_count(6u);

    uint8_t one = 0;
    TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&rb, &one));
    TEST_ASSERT_EQUAL_HEX8(0x05u, one); /* the byte left behind before the wrap */

    memset(out, 0, sizeof(out));
    TEST_ASSERT_EQUAL_UINT16(5u, UTIL_RingBuf_GetN(&rb, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(second, out, sizeof(second));
}

static void test_util_ringbuf_getn_wraps_across_the_buffer_end(void)
{
    static const uint8_t pattern[7] = {0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7};
    uint8_t              out[7];

    fixture_init();

    /* Push tail to 6 first, so the following 7-byte read starts near the end and
     * must wrap to collect the rest. */
    TEST_ASSERT_EQUAL_UINT16(6u, UTIL_RingBuf_PutN(&rb, pattern, 6u));
    TEST_ASSERT_EQUAL_UINT16(6u, UTIL_RingBuf_GetN(&rb, out, 6u));
    expect_count(0u);
    TEST_ASSERT_EQUAL_UINT16(6u, rb.tail);

    TEST_ASSERT_EQUAL_UINT16(7u, UTIL_RingBuf_PutN(&rb, pattern, 7u));
    memset(out, 0, sizeof(out));
    TEST_ASSERT_EQUAL_UINT16(7u, UTIL_RingBuf_GetN(&rb, out, 7u));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(pattern, out, sizeof(pattern));
}

static void test_util_ringbuf_survives_uint16_index_overflow(void)
{
    uint8_t out[3];

    fixture_init();

    /* head and tail are uint16_t and are never reduced modulo capacity, so they
     * roll over at 65536. That is only harmless because capacity divides 65536;
     * cycling past the rollover proves no byte is lost or duplicated there. */
    for (unsigned n = 0; n < 70000u; n += 3u)
    {
        const uint8_t in[3] = {(uint8_t) n, (uint8_t) (n + 1u), (uint8_t) (n + 2u)};

        TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_PutN(&rb, in, 3u));
        TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_GetN(&rb, out, 3u));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, 3u);
    }
    expect_count(0u);
}

/* ========================================================================= */
/*  Flush                                                                    */
/* ========================================================================= */

static void test_util_ringbuf_flush_discards_everything(void)
{
    static const uint8_t in[5] = {0x31, 0x32, 0x33, 0x34, 0x35};
    uint8_t              out   = 0;

    fixture_init();
    TEST_ASSERT_EQUAL_UINT16(5u, UTIL_RingBuf_PutN(&rb, in, sizeof(in)));

    UTIL_RingBuf_Flush(&rb);

    expect_count(0u);
    TEST_ASSERT_FALSE(UTIL_RingBuf_Get(&rb, &out));
    TEST_ASSERT_EQUAL_UINT16(0u, rb.head);
    TEST_ASSERT_EQUAL_UINT16(0u, rb.tail);
}

static void test_util_ringbuf_flush_when_wrapped_leaves_it_usable(void)
{
    static const uint8_t in[6] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46};
    uint8_t              out[6];

    fixture_init();

    /* Flush resets both indices to 0 rather than equalising them, so the case
     * worth checking is a flush taken while the indices are mid-buffer. */
    TEST_ASSERT_EQUAL_UINT16(6u, UTIL_RingBuf_PutN(&rb, in, sizeof(in)));
    TEST_ASSERT_EQUAL_UINT16(5u, UTIL_RingBuf_GetN(&rb, out, 5u));
    UTIL_RingBuf_Flush(&rb);
    expect_count(0u);

    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_PutN(&rb, in, 3u));
    memset(out, 0, sizeof(out));
    TEST_ASSERT_EQUAL_UINT16(3u, UTIL_RingBuf_GetN(&rb, out, 3u));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, 3u);
}

static void test_util_ringbuf_flush_on_empty_is_harmless(void)
{
    fixture_init();
    UTIL_RingBuf_Flush(&rb);
    UTIL_RingBuf_Flush(&rb);
    expect_count(0u);
    TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&rb, 0x01u));
    expect_count(1u);
}

/* ========================================================================= */
/*  Larger capacity                                                          */
/* ========================================================================= */

static void test_util_ringbuf_larger_capacity_holds_capacity_minus_one(void)
{
    static uint8_t big_storage[256];
    UTIL_RingBuf_s big;
    static uint8_t in[256];
    static uint8_t out[256];

    for (unsigned i = 0; i < sizeof(in); i++)
    {
        in[i] = (uint8_t) (i ^ 0x3Cu);
    }

    UTIL_RingBuf_Init(&big, big_storage, (uint16_t) sizeof(big_storage));

    /* The reservation is one slot regardless of capacity, so 255 of 256. */
    TEST_ASSERT_EQUAL_UINT16(255u, UTIL_RingBuf_Free(&big));
    TEST_ASSERT_EQUAL_UINT16(255u, UTIL_RingBuf_PutN(&big, in, sizeof(in)));
    TEST_ASSERT_TRUE(UTIL_RingBuf_IsFull(&big));
    TEST_ASSERT_FALSE(UTIL_RingBuf_Put(&big, 0xFFu));

    TEST_ASSERT_EQUAL_UINT16(255u, UTIL_RingBuf_GetN(&big, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(in, out, 255u);
    TEST_ASSERT_TRUE(UTIL_RingBuf_IsEmpty(&big));
}

static void test_util_ringbuf_minimum_useful_capacity_of_two(void)
{
    static uint8_t tiny_storage[2];
    UTIL_RingBuf_s tiny;
    uint8_t        out = 0;

    /* Capacity 2 is the smallest that can hold anything at all, and the tightest
     * case for the reserved slot: depth 1. */
    UTIL_RingBuf_Init(&tiny, tiny_storage, 2u);

    TEST_ASSERT_TRUE(UTIL_RingBuf_IsEmpty(&tiny));
    TEST_ASSERT_EQUAL_UINT16(1u, UTIL_RingBuf_Free(&tiny));
    TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&tiny, 0xB7u));
    TEST_ASSERT_TRUE(UTIL_RingBuf_IsFull(&tiny));
    TEST_ASSERT_FALSE(UTIL_RingBuf_Put(&tiny, 0xB8u));

    TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&tiny, &out));
    TEST_ASSERT_EQUAL_HEX8(0xB7u, out);
    TEST_ASSERT_TRUE(UTIL_RingBuf_IsEmpty(&tiny));

    /* Wrapping with mask 1 alternates between the two slots. */
    for (unsigned n = 0; n < 10u; n++)
    {
        TEST_ASSERT_TRUE(UTIL_RingBuf_Put(&tiny, (uint8_t) n));
        TEST_ASSERT_TRUE(UTIL_RingBuf_Get(&tiny, &out));
        TEST_ASSERT_EQUAL_HEX8((uint8_t) n, out);
    }
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_util_ringbuf_init_sets_empty_state_and_mask);
    RUN_TEST(test_util_ringbuf_get_on_empty_is_refused_and_writes_nothing);
    RUN_TEST(test_util_ringbuf_getn_on_empty_returns_zero);

    RUN_TEST(test_util_ringbuf_put_get_preserves_fifo_order);
    RUN_TEST(test_util_ringbuf_put_refused_when_full_and_drops_the_byte);
    RUN_TEST(test_util_ringbuf_last_storage_slot_is_never_written);

    RUN_TEST(test_util_ringbuf_putn_getn_full_transfer);
    RUN_TEST(test_util_ringbuf_putn_zero_length_is_a_no_op);
    RUN_TEST(test_util_ringbuf_getn_zero_length_is_a_no_op);
    RUN_TEST(test_util_ringbuf_putn_clamps_to_free_space);
    RUN_TEST(test_util_ringbuf_putn_on_full_stores_nothing);
    RUN_TEST(test_util_ringbuf_putn_partial_fill_then_clamped_second_call);
    RUN_TEST(test_util_ringbuf_getn_clamps_to_available);
    RUN_TEST(test_util_ringbuf_getn_drains_incrementally);
    RUN_TEST(test_util_ringbuf_bulk_and_single_apis_interoperate);

    RUN_TEST(test_util_ringbuf_single_byte_wrap_preserves_data);
    RUN_TEST(test_util_ringbuf_putn_wraps_across_the_buffer_end);
    RUN_TEST(test_util_ringbuf_getn_wraps_across_the_buffer_end);
    RUN_TEST(test_util_ringbuf_survives_uint16_index_overflow);

    RUN_TEST(test_util_ringbuf_flush_discards_everything);
    RUN_TEST(test_util_ringbuf_flush_when_wrapped_leaves_it_usable);
    RUN_TEST(test_util_ringbuf_flush_on_empty_is_harmless);

    RUN_TEST(test_util_ringbuf_larger_capacity_holds_capacity_minus_one);
    RUN_TEST(test_util_ringbuf_minimum_useful_capacity_of_two);

    return UNITY_END();
}
