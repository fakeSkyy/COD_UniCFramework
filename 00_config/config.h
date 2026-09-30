/**
 * @file config.h
 * @author Gao Xing
 * @date 2026/9/29
 * @version 1.0
 *
 * @brief Build-wide switches that no single module owns.
 *
 * @par What belongs here, and what does not
 * A switch belongs here when turning it changes more than one module and no one
 * of them is its natural home. Everything else stays in the header of the module
 * that owns it — IMU setpoints in app_imu.c, PID gains at their call site, task
 * priorities in app_tasks.c. This file is deliberately short, and a growing list
 * of per-module tunables here would be a mistake: a reader looking for why the
 * heater runs at 40 C should find that number next to the heater, not in a
 * central file that knows about everything.
 *
 * This header must stay includable from any layer, so it contains no types, no
 * includes and no code — only object-like macros with literal values.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================= */
/*  Math backend                                                             */
/* ========================================================================= */

/**
 * @brief Route the solver's matrix and arctangent work through CMSIS-DSP.
 *
 * 0 (default) uses the standard library and this repository's own
 * @c util_fast_math. 1 substitutes CMSIS-DSP for the operations where it is
 * actually faster on this core.
 *
 * @par Default is 0 on purpose
 * The DSP path is the variant, not the baseline. Every host test, every figure in
 * docs/ and every measurement taken on this board so far was made with 0, and a
 * default that silently changed the numerical path would invalidate all of them
 * at once. Turn it on deliberately, and re-measure.
 *
 * @par What actually changes, and what does not
 * Enabling this is NOT "use DSP everywhere". Three of the five math operations in
 * the solver are left alone because DSP is not better at them on a Cortex-M7 with
 * an FPU, and pretending otherwise would cost speed for nothing:
 *
 *   - Matrix multiply and add: SUBSTITUTED. arm_mat_mult_f32 unrolls its inner
 *     loop by four where util_kf's is a plain triple loop, and the 6x6 products
 *     in UTIL_KF_Predict run twice per 1 kHz step. This is where the gain is.
 *   - atan2: SUBSTITUTED. arm_atan2_f32 is a 10th-order polynomial, against
 *     UTIL_FastAtan2's interpolated table. Which is faster is not obvious and is
 *     worth measuring; the polynomial is the more accurate of the two.
 *   - Square root: NOT substituted. arm_sqrt_f32 expands, under GCC, to exactly
 *     `*pOut = sqrtf(in)` — and sqrtf on this target compiles to a single
 *     VSQRT.F32. Going through DSP would add a call and a status return around
 *     one instruction. Verified by compiling for cortex-m7 + fpv5-d16.
 *   - asin: NOT substituted. CMSIS-DSP has no arm_asin_f32 at all.
 *   - sin/cos: NOT substituted. arm_sin_cos_f32 takes DEGREES, so every call site
 *     would pay a conversion that the radian-native UTIL_FastSinCos does not.
 *
 * @par Cost of turning it on
 * Four vendor sources join the build (arm_mat_mult_f32, arm_mat_add_f32,
 * arm_mat_init_f32, arm_atan2_f32) and ARM_MATH_CM7 plus ARM_MATH_LOOPUNROLL are
 * defined. Nothing is linked when this is 0 — see the gate in CMakeLists.txt.
 *
 * @par Numerical results differ
 * arm_mat_mult_f32 accumulates in a different order than util_kf's loop, so the
 * last bits of P differ; the atan2 substitution changes the Euler angles by up to
 * the table's own 0.002 rad. Both are within what the filter tolerates, but an
 * exact-equality test against a recorded trace will fail. The dsp-labelled host
 * tests assert the same behaviour with widened tolerances for this reason.
 */
#ifndef UTIL_MATH_USE_CMSIS_DSP
#define UTIL_MATH_USE_CMSIS_DSP 0
#endif

#endif /* CONFIG_H */
