/**
 * @file util_math_backend.h
 * @author Gao Xing
 * @date 2026/9/29
 * @version 1.0
 *
 * @brief One name per math operation the solver needs, bound to a backend.
 *
 * @par Why this file exists rather than #if at each call site
 * util_ahrs and util_kf between them call five distinct math operations. Putting
 * the switch at each call would mean ten preprocessor branches through code whose
 * correctness depends on reading it as arithmetic, and a reader would have to
 * evaluate the preprocessor to know what runs. Here the choice is made once per
 * operation and the call sites stay readable.
 *
 * @par Operations that are NOT switched, and why
 * Deliberately absent: a sqrt entry, an asin entry, and sin/cos entries. The
 * reasoning is in 00_config/config.h next to the switch — briefly, sqrtf is
 * already one VSQRT.F32 instruction on this target, CMSIS-DSP has no asin at all,
 * and its sin_cos takes degrees. Call sqrtf and asinf directly; a wrapper that
 * forwards to the same function under both settings would only suggest that
 * something varies.
 *
 * @par What a backend must guarantee
 * The matrix entries operate on ROW-MAJOR storage of plain floats. The DSP path
 * wraps them in arm_matrix_instance_f32 descriptors internally, so no caller ever
 * handles one.
 *
 * Aliasing rules differ per operation, and the difference is load-bearing:
 *
 *   - The two PRODUCTS forbid it. Element (i,j) of the result reads a whole row
 *     and a whole column, so writing it would corrupt inputs still to be read.
 *     util_kf writes into a separate scratch for this reason.
 *   - The SUM permits @c dst==a or @c dst==b. Both backends walk the buffers
 *     strictly element by element and forward, so element i reads only element i;
 *     verified in arm_mat_add_f32.c, whose whole body is
 *     @c *pOut++ = *pInA++ + *pInB++ (plus a four-way unrolled copy of it). This
 *     is what lets util_kf accumulate Q into P without a third buffer.
 */

#ifndef UTIL_MATH_BACKEND_H
#define UTIL_MATH_BACKEND_H

#include <stdint.h>

#include "config.h"

#if UTIL_MATH_USE_CMSIS_DSP

/* NOT "arm_math.h" -- that name is ambiguous in this tree and resolves to the
 * WRONG header. CubeMX installs two DSP packages and puts both on the include
 * path: Middlewares/ST/ARM/DSP (V1.7.0, one monolithic arm_math.h, no atan2) and
 * Drivers/CMSIS/DSP (current, declarations split into a dsp/ subdirectory). The
 * vendor subtree's own CMakeLists adds ST's Inc unconditionally, so which
 * arm_math.h wins depends on include order -- and when it loses, the failure is
 * `implicit declaration of function 'arm_atan2_f32'`, which reads like a missing
 * library rather than the wrong one.
 *
 * The two headers below exist ONLY in the current package, so naming them is
 * unambiguous no matter what else is on the path. They are also self-sufficient:
 * each pulls in arm_math_types.h itself (verified on both arm-none-eabi-gcc and
 * the host gcc, -Wall -Wextra clean). */
#include "dsp/fast_math_functions.h"
#include "dsp/matrix_functions.h"

#else
#include <math.h>
#endif

/* ========================================================================= */
/*  Matrix operations                                                        */
/* ========================================================================= */

/**
 * @brief Row-major matrix product: dst(rows x cols) = a(rows x inner) * b(inner x cols).
 *
 * @param a      Left operand, @p rows by @p inner.
 * @param b      Right operand, @p inner by @p cols.
 * @param dst    Destination, @p rows by @p cols. Must not alias @p a or @p b.
 * @param rows   Rows of @p a and of @p dst.
 * @param inner  Columns of @p a, rows of @p b.
 * @param cols   Columns of @p b and of @p dst.
 */
static inline void UTIL_MatMul(const float* a, const float* b, float* dst, uint16_t rows,
                               uint16_t inner, uint16_t cols)
{
#if UTIL_MATH_USE_CMSIS_DSP
    /* The descriptors carry no data, so building them per call costs three stores
     * each and lets the caller keep plain arrays. Casting away const is safe and
     * unavoidable: arm_matrix_instance_f32 has a non-const pData even for an
     * operand the function only reads. */
    arm_matrix_instance_f32 ma = {rows, inner, (float*) (const void*) a};
    arm_matrix_instance_f32 mb = {inner, cols, (float*) (const void*) b};
    arm_matrix_instance_f32 md = {rows, cols, dst};

    (void) arm_mat_mult_f32(&ma, &mb, &md);
#else
    for (uint16_t i = 0u; i < rows; i++)
    {
        const float* a_row = &a[(uint32_t) i * inner];

        for (uint16_t j = 0u; j < cols; j++)
        {
            float acc = 0.0f;

            for (uint16_t k = 0u; k < inner; k++)
            {
                acc += a_row[k] * b[(uint32_t) k * cols + j];
            }

            dst[(uint32_t) i * cols + j] = acc;
        }
    }
#endif
}

/**
 * @brief Row-major matrix product with the right operand transposed:
 *        dst(rows x cols) = a(rows x inner) * b(cols x inner)'.
 *
 * @par Why this is its own entry rather than a transpose plus a multiply
 * util_kf needs A*P*A'. Materialising A' would need an n*n buffer that the
 * sequential formulation was specifically designed to avoid — the buffer-size
 * saving documented at UTIL_KF_BUF_SIZE. Reading B's rows as the product's
 * columns costs nothing and keeps that saving.
 *
 * Under the DSP backend this walks B's rows directly too: arm_mat_mult_f32 would
 * need the explicit transpose, so using it here would reintroduce the buffer.
 * This one operation is therefore identical under both settings, which is why it
 * is documented rather than silently the same.
 *
 * @param a      Left operand, @p rows by @p inner.
 * @param b      Right operand, @p cols by @p inner — used transposed.
 * @param dst    Destination, @p rows by @p cols. Must not alias @p a or @p b.
 * @param rows   Rows of @p a and of @p dst.
 * @param inner  Columns of both @p a and @p b.
 * @param cols   Rows of @p b, columns of @p dst.
 */
static inline void UTIL_MatMulTransposed(const float* a, const float* b, float* dst, uint16_t rows,
                                         uint16_t inner, uint16_t cols)
{
    for (uint16_t i = 0u; i < rows; i++)
    {
        const float* a_row = &a[(uint32_t) i * inner];

        for (uint16_t j = 0u; j < cols; j++)
        {
            const float* b_row = &b[(uint32_t) j * inner];
            float        acc   = 0.0f;

            for (uint16_t k = 0u; k < inner; k++)
            {
                acc += a_row[k] * b_row[k];
            }

            dst[(uint32_t) i * cols + j] = acc;
        }
    }
}

/**
 * @brief Element-wise sum of two row-major matrices: dst = a + b.
 *
 * @param a      Left operand.
 * @param b      Right operand.
 * @param dst    Destination. MAY be @p a or @p b — the walk is element-wise and
 *               forward under both backends. A partial overlap at some other
 *               offset is not covered.
 * @param rows   Rows.
 * @param cols   Columns.
 */
static inline void UTIL_MatAdd(const float* a, const float* b, float* dst, uint16_t rows,
                               uint16_t cols)
{
#if UTIL_MATH_USE_CMSIS_DSP
    arm_matrix_instance_f32 ma = {rows, cols, (float*) (const void*) a};
    arm_matrix_instance_f32 mb = {rows, cols, (float*) (const void*) b};
    arm_matrix_instance_f32 md = {rows, cols, dst};

    (void) arm_mat_add_f32(&ma, &mb, &md);
#else
    const uint32_t n = (uint32_t) rows * cols;

    for (uint32_t i = 0u; i < n; i++)
    {
        dst[i] = a[i] + b[i];
    }
#endif
}

/* ========================================================================= */
/*  Scalar operations                                                        */
/* ========================================================================= */

/**
 * @brief Arctangent of @p y / @p x over the full circle, in radians.
 *
 * @par The two backends differ in accuracy, and the DSP one is better
 * The standard path is atan2f, exactly rounded. The DSP path is a 10th-order
 * polynomial, accurate to roughly 1e-7 — so switching backends moves the Euler
 * angles in the last few bits, not visibly. Note this is NOT
 * UTIL_FastAtan2: that one carries 0.002 rad of error, which is 0.12 degrees of
 * attitude and too much to spend here. It stays available for call sites whose
 * budget allows it.
 *
 * @param y  Numerator.
 * @param x  Denominator.
 * @return Angle in radians, in [-pi, pi]. Zero when both arguments are zero.
 */
static inline float UTIL_Atan2(float y, float x)
{
#if UTIL_MATH_USE_CMSIS_DSP
    float out = 0.0f;

    /* Returns ARM_MATH_NANINF for (0,0) and leaves out untouched, which is why it
     * is seeded above rather than after the call. */
    (void) arm_atan2_f32(y, x, &out);

    return out;
#else
    return atan2f(y, x);
#endif
}

#endif /* UTIL_MATH_BACKEND_H */
