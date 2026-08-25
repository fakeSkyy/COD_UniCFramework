/**
 * @file test_util_maf.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "test_support.h"

#include "util_maf.h"

void setUp(void) {}
void tearDown(void) {}

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Step a filter @p n times with a constant input.
 * @return Last reported average.
 */
static float maf_drive(UTIL_MAF_s* ma, float input, int n)
{
    float avg = 0.0f;

    for (int i = 0; i < n; i++)
    {
        avg = UTIL_MAF_Step(ma, input);
    }

    return avg;
}

/**
 * @brief Fill caller storage with a value Step must never read back.
 *
 * Init deliberately does not clear the window, so a poison pattern is the only
 * way to prove that promise rather than assume it.
 */
static void poison(float* buf, uint16_t length)
{
    for (uint16_t i = 0u; i < length; i++)
    {
        buf[i] = 1.0e9f;
    }
}

/* ========================================================================= */
/*  Initialization                                                           */
/* ========================================================================= */

static void test_maf_init_rejects_bad_arguments(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    TEST_ASSERT_FALSE(UTIL_MAF_Init(NULL, buf, 4u));

    TEST_ASSERT_FALSE(UTIL_MAF_Init(&ma, NULL, 4u));
    TEST_ASSERT_FALSE(ma.initialized);
    TEST_ASSERT_NULL(ma.buffer);

    TEST_ASSERT_FALSE(UTIL_MAF_Init(&ma, buf, 0u));
    TEST_ASSERT_FALSE(ma.initialized);

    /* A zero length would otherwise divide by zero and index a slot the caller
     * never provided, so the instance must be left inert rather than partly
     * configured. */
    TEST_ASSERT_NULL(ma.buffer);
    TEST_ASSERT_EQUAL_UINT16(0u, ma.length);
}

static void test_maf_failed_init_leaves_step_inert(void)
{
    UTIL_MAF_s ma;

    TEST_ASSERT_FALSE(UTIL_MAF_Init(&ma, NULL, 4u));

    /* Nothing may touch unowned memory afterwards; Step's only defence is the
     * initialized flag, and Reset/Prime/Resync must also stay quiet. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Step(&ma, 5.0f));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Get(&ma));

    UTIL_MAF_Prime(&ma, 3.0f);
    UTIL_MAF_Resync(&ma);
    UTIL_MAF_Reset(&ma);

    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Step(&ma, 5.0f));
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));
}

static void test_maf_init_accepts_valid_window(void)
{
    UTIL_MAF_s ma;
    float      buf[8];

    poison(buf, 8u);

    TEST_ASSERT_TRUE(UTIL_MAF_Init(&ma, buf, 8u));
    TEST_ASSERT_TRUE(ma.initialized);
    TEST_ASSERT_EQUAL_UINT16(8u, ma.length);
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));
    TEST_ASSERT_FALSE(UTIL_MAF_IsFull(&ma));

    /* inv_count is left at zero while the window is empty, which is what makes
     * Get return 0 without a count == 0 branch on the hot path. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Get(&ma));
}

static void test_maf_null_instance_tolerated_by_void_calls(void)
{
    UTIL_MAF_Reset(NULL);
    UTIL_MAF_Prime(NULL, 1.0f);
    UTIL_MAF_Resync(NULL);
}

/* ========================================================================= */
/*  Averaging                                                                */
/* ========================================================================= */

static void test_maf_averages_over_arrived_samples_while_filling(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    poison(buf, 4u);
    UTIL_MAF_Init(&ma, buf, 4u);

    /* Dividing by the full length from the start would make the output ramp out
     * of zero and read as a transient the input never contained. */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_MAF_Step(&ma, 1.0f));
    TEST_ASSERT_EQUAL_UINT16(1u, UTIL_MAF_Count(&ma));

    TEST_ASSERT_EQUAL_FLOAT(1.5f, UTIL_MAF_Step(&ma, 2.0f));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_MAF_Step(&ma, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(2.5f, UTIL_MAF_Step(&ma, 4.0f));

    TEST_ASSERT_TRUE(UTIL_MAF_IsFull(&ma));
    TEST_ASSERT_EQUAL_UINT16(4u, UTIL_MAF_Count(&ma));
}

static void test_maf_slides_the_window_once_full(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    poison(buf, 4u);
    UTIL_MAF_Init(&ma, buf, 4u);

    maf_drive(&ma, 0.0f, 0);
    UTIL_MAF_Step(&ma, 1.0f);
    UTIL_MAF_Step(&ma, 2.0f);
    UTIL_MAF_Step(&ma, 3.0f);
    UTIL_MAF_Step(&ma, 4.0f);

    /* The departing sample must actually leave the sum: (2+3+4+5)/4. */
    TEST_ASSERT_EQUAL_FLOAT(3.5f, UTIL_MAF_Step(&ma, 5.0f));
    TEST_ASSERT_EQUAL_FLOAT(4.5f, UTIL_MAF_Step(&ma, 6.0f));
    TEST_ASSERT_EQUAL_UINT16(4u, UTIL_MAF_Count(&ma));
}

static void test_maf_window_of_one_is_a_passthrough(void)
{
    UTIL_MAF_s ma;
    float      buf[1];

    /* length 1 is the documented minimum, and it drives the wrap and resync
     * branches on every single call. */
    TEST_ASSERT_TRUE(UTIL_MAF_Init(&ma, buf, 1u));

    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_MAF_Step(&ma, 3.0f));
    TEST_ASSERT_EQUAL_FLOAT(9.0f, UTIL_MAF_Step(&ma, 9.0f));
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, UTIL_MAF_Step(&ma, -2.0f));
    TEST_ASSERT_TRUE(UTIL_MAF_IsFull(&ma));
}

static void test_maf_dc_gain_is_unity(void)
{
    UTIL_MAF_s ma;
    float      buf[8];

    UTIL_MAF_Init(&ma, buf, 8u);

    TEST_ASSERT_EQUAL_FLOAT(100.0f, maf_drive(&ma, 100.0f, 100));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, UTIL_MAF_Get(&ma));
}

static void test_maf_nulls_a_disturbance_matching_the_window(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    UTIL_MAF_Init(&ma, buf, 4u);

    /* The reason to choose this over an IIR low-pass: a moving average places
     * zeros on every frequency whose period divides the window, so a matched
     * disturbance is removed completely rather than merely attenuated. */
    float avg = 0.0f;

    for (int i = 0; i < 400; i++)
    {
        avg = UTIL_MAF_Step(&ma, 5.0f + ((i % 2) ? -2.0f : 2.0f));
    }

    TEST_ASSERT_EQUAL_FLOAT(5.0f, avg);
}

static void test_maf_running_sum_does_not_drift(void)
{
    UTIL_MAF_s ma;
    float      buf[32];

    UTIL_MAF_Init(&ma, buf, 32u);

    /* The header quantifies the drift a bare running sum accumulates over an
     * hour at 1 kHz; the periodic rebuild is what bounds it. 200k samples of a
     * 1000-unit offset is that scenario compressed. */
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPS_LOOSE, 1000.0f, maf_drive(&ma, 1000.0f, 200000));
}

static void test_maf_resync_repairs_a_corrupted_sum(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    UTIL_MAF_Init(&ma, buf, 4u);
    UTIL_MAF_Step(&ma, 1.0f);
    UTIL_MAF_Step(&ma, 2.0f);
    UTIL_MAF_Step(&ma, 3.0f);
    UTIL_MAF_Step(&ma, 4.0f);

    /* Stand in for accumulated rounding error. The automatic rebuild fires
     * every `length` steps, so the wrong average must clear within four. */
    ma.sum += 40.0f;

    TEST_ASSERT_EQUAL_FLOAT(4.0f, maf_drive(&ma, 4.0f, 4));
    TEST_ASSERT_EQUAL_FLOAT(4.0f, UTIL_MAF_Get(&ma));
}

static void test_maf_resync_ignores_slots_beyond_count(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    poison(buf, 4u);
    UTIL_MAF_Init(&ma, buf, 4u);
    UTIL_MAF_Step(&ma, 2.0f);
    UTIL_MAF_Step(&ma, 4.0f);

    /* A partly-filled window holds whatever the caller's storage held before
     * Init in the remaining slots; summing them would pull 1e9 into the mean. */
    UTIL_MAF_Resync(&ma);

    TEST_ASSERT_EQUAL_FLOAT(3.0f, UTIL_MAF_Get(&ma));
    TEST_ASSERT_EQUAL_UINT16(2u, UTIL_MAF_Count(&ma));
}

/* ========================================================================= */
/*  Reset and prime                                                          */
/* ========================================================================= */

static void test_maf_reset_empties_the_window(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    UTIL_MAF_Init(&ma, buf, 4u);
    maf_drive(&ma, 100.0f, 20);
    UTIL_MAF_Reset(&ma);

    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));
    TEST_ASSERT_FALSE(UTIL_MAF_IsFull(&ma));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Get(&ma));

    /* Back to averaging over one sample, so the old regime cannot blend into
     * the new one. */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_MAF_Step(&ma, 1.0f));
}

static void test_maf_prime_fills_at_full_length(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    poison(buf, 4u);
    UTIL_MAF_Init(&ma, buf, 4u);
    UTIL_MAF_Prime(&ma, 7.0f);

    /* Unlike Reset, the averaging length must not change underneath the loop:
     * the output is 7.0 immediately AND already at full window. */
    TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_MAF_Get(&ma));
    TEST_ASSERT_EQUAL_UINT16(4u, UTIL_MAF_Count(&ma));
    TEST_ASSERT_TRUE(UTIL_MAF_IsFull(&ma));

    TEST_ASSERT_EQUAL_FLOAT(8.0f, UTIL_MAF_Step(&ma, 11.0f));
}

static void test_maf_prime_nonfinite_clears_instead(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    UTIL_MAF_Init(&ma, buf, 4u);
    UTIL_MAF_Prime(&ma, 7.0f);
    UTIL_MAF_Prime(&ma, NAN);

    /* Writing the bad value into every slot would poison the whole window, so
     * clearing is the documented fallback. */
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));
    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Get(&ma));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_MAF_Step(&ma, 2.0f));

    UTIL_MAF_Prime(&ma, 7.0f);
    UTIL_MAF_Prime(&ma, INFINITY);
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));

    UTIL_MAF_Prime(&ma, 7.0f);
    UTIL_MAF_Prime(&ma, -INFINITY);
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));
}

/* ========================================================================= */
/*  Hostile input                                                            */
/* ========================================================================= */

static void test_maf_nonfinite_sample_held_and_never_latched(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    UTIL_MAF_Init(&ma, buf, 4u);
    UTIL_MAF_Step(&ma, 1.0f);
    UTIL_MAF_Step(&ma, 2.0f);
    UTIL_MAF_Step(&ma, 3.0f);
    UTIL_MAF_Step(&ma, 4.0f);

    float held = UTIL_MAF_Step(&ma, 5.0f);

    TEST_ASSERT_EQUAL_FLOAT(3.5f, held);

    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_MAF_Step(&ma, NAN));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_MAF_Step(&ma, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_MAF_Step(&ma, -INFINITY));

    /* Admitting one bad sample would poison the running sum for a whole window,
     * not one sample: the next average proves the window advanced by exactly
     * one slot and nothing else changed. (3+4+5+6)/4 = 4.5. */
    TEST_ASSERT_EQUAL_FLOAT(4.5f, UTIL_MAF_Step(&ma, 6.0f));
    TEST_ASSERT_EQUAL_FLOAT(20.0f, maf_drive(&ma, 20.0f, 8));
    TEST_ASSERT_FINITE(UTIL_MAF_Get(&ma));
}

static void test_maf_nonfinite_sample_rejected_while_filling(void)
{
    UTIL_MAF_s ma;
    float      buf[4];

    poison(buf, 4u);
    UTIL_MAF_Init(&ma, buf, 4u);

    /* Before any good sample there is nothing to hold, so 0 is returned — and
     * critically, count must not advance, or the bad sample would occupy a slot
     * and divide the running sum by one too many. */
    TEST_ASSERT_EXACTLY_ZERO(UTIL_MAF_Step(&ma, NAN));
    TEST_ASSERT_EQUAL_UINT16(0u, UTIL_MAF_Count(&ma));

    TEST_ASSERT_EQUAL_FLOAT(6.0f, UTIL_MAF_Step(&ma, 6.0f));
    TEST_ASSERT_EQUAL_UINT16(1u, UTIL_MAF_Count(&ma));

    TEST_ASSERT_EQUAL_FLOAT(6.0f, UTIL_MAF_Step(&ma, INFINITY));
    TEST_ASSERT_EQUAL_UINT16(1u, UTIL_MAF_Count(&ma));

    TEST_ASSERT_EQUAL_FLOAT(5.0f, UTIL_MAF_Step(&ma, 4.0f));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_maf_init_rejects_bad_arguments);
    RUN_TEST(test_maf_failed_init_leaves_step_inert);
    RUN_TEST(test_maf_init_accepts_valid_window);
    RUN_TEST(test_maf_null_instance_tolerated_by_void_calls);

    RUN_TEST(test_maf_averages_over_arrived_samples_while_filling);
    RUN_TEST(test_maf_slides_the_window_once_full);
    RUN_TEST(test_maf_window_of_one_is_a_passthrough);
    RUN_TEST(test_maf_dc_gain_is_unity);
    RUN_TEST(test_maf_nulls_a_disturbance_matching_the_window);
    RUN_TEST(test_maf_running_sum_does_not_drift);
    RUN_TEST(test_maf_resync_repairs_a_corrupted_sum);
    RUN_TEST(test_maf_resync_ignores_slots_beyond_count);

    RUN_TEST(test_maf_reset_empties_the_window);
    RUN_TEST(test_maf_prime_fills_at_full_length);
    RUN_TEST(test_maf_prime_nonfinite_clears_instead);

    RUN_TEST(test_maf_nonfinite_sample_held_and_never_latched);
    RUN_TEST(test_maf_nonfinite_sample_rejected_while_filling);

    return UNITY_END();
}
