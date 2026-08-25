/**
 * @file test_support.h
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <math.h>

#include "unity.h"

/* ==========================================================================
 * Shared helpers for the 06_utils host test suites
 * ==========================================================================
 *
 * Every suite here tests a module that compiles on the host: 06_utils has no
 * HAL dependency by design, which is what makes these tests possible at all.
 * The two modules that do reach outside (util_log wants SEGGER_RTT, util_msgbus
 * wants the platform layer) are served by the stubs in stubs/, so they can be
 * tested here too rather than only on hardware.
 *
 * @par Float comparison
 * Everything in 06_utils is single-precision, so an absolute epsilon is the
 * right tool for values near unity and the wrong one for values near zero or
 * far from it. TEST_ASSERT_FLOAT_WITHIN takes the tolerance explicitly for
 * exactly that reason; these macros only name the tolerances the suites keep
 * reaching for, so a reader sees "this is a loose check" without decoding a
 * literal.
 * ==========================================================================
 */

/** @brief Tolerance for a value that should be exact but went through floats. */
#define TEST_EPS_TIGHT 1e-5f

/** @brief Tolerance for a converged filter output or an accumulated sum. */
#define TEST_EPS_LOOSE 1e-3f

/** @brief Tolerance for a fast/approximate math routine against libm. */
#define TEST_EPS_APPROX 2e-3f

/**
 * @brief Assert a float is finite — neither NaN nor an infinity.
 *
 * Used after feeding a module a hostile input: the contract throughout 06_utils
 * is that a bad sample is rejected or held, never latched into the state, and
 * the observable form of a violation is a non-finite value appearing later.
 */
#define TEST_ASSERT_FINITE(v) TEST_ASSERT_TRUE_MESSAGE(isfinite(v), "value is NaN or Inf")

/** @brief Assert a float is exactly zero, bit-wise (not "close to zero"). */
#define TEST_ASSERT_EXACTLY_ZERO(v) TEST_ASSERT_EQUAL_FLOAT(0.0f, (v))

#endif /* TEST_SUPPORT_H */
