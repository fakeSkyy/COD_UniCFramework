/**
 * @file plat_pwm.h
 * @author Gao Xing
 * @date 2025/7/28
 * @version 1.0
 */

#ifndef PLAT_PWM_H
#define PLAT_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "impl_pwm.h"

typedef struct PWM_Instance_s PWM_Instance_s;

/**
 * @brief A vendor-neutral PWM output handle.
 *
 * Carries an ops vtable, an opaque @c ctx produced by some backend, and the
 * small amount of state the platform layer owns: the last-set duty percent
 * (so it can be restored across a frequency change) and whether the output is
 * currently running. This layer never dereferences @c ctx.
 */
struct PWM_Instance_s
{
    const PWM_Ops_s* ops;          /**< Backend vtable (from *_GetOps).      */
    void*            ctx;          /**< Opaque, backend-owned descriptor.    */
    float            duty_percent; /**< Last-set duty in [0,100], for reload.*/
    bool             running;      /**< True between Start and Stop.         */
    void*            id;           /**< Optional owner tag for the caller.   */
};

/**
 * @brief Enable PWM output (counter + channel).
 * @param pwm  PWM instance.
 * @return true on success, false on failure.
 */
bool PLAT_PWM_Start(PWM_Instance_s* pwm);

/**
 * @brief Disable PWM output.
 * @param pwm  PWM instance.
 */
void PLAT_PWM_Stop(PWM_Instance_s* pwm);

/**
 * @brief Query whether the output is currently running.
 * @param pwm  PWM instance.
 * @return true if running, false otherwise.
 */
bool PLAT_PWM_IsRunning(const PWM_Instance_s* pwm);

/**
 * @brief Set duty cycle as a raw compare value.
 *
 * The value is clamped to [0, period]. This does not update the stored duty
 * percent, so a later PLAT_PWM_SetFrequency restores the percent implied by
 * the most recent percent-based call.
 *
 * @param pwm   PWM instance.
 * @param duty  Compare value (0 .. period).
 */
void PLAT_PWM_SetDuty(PWM_Instance_s* pwm, uint32_t duty);

/**
 * @brief Set duty cycle as a percentage.
 * @param pwm      PWM instance.
 * @param percent  Duty in [0.0, 100.0]; out-of-range values are clamped.
 */
void PLAT_PWM_SetDutyPercent(PWM_Instance_s* pwm, float percent);

/**
 * @brief Set the output frequency, preserving the current duty percent.
 * @param pwm      PWM instance.
 * @param freq_hz  Target frequency in Hz.
 * @return true on success, false if the backend rejected the frequency.
 */
bool PLAT_PWM_SetFrequency(PWM_Instance_s* pwm, uint32_t freq_hz);

/**
 * @brief Set frequency and duty percent together in one update.
 * @param pwm      PWM instance.
 * @param freq_hz  Target frequency in Hz.
 * @param percent  Duty in [0.0, 100.0]; out-of-range values are clamped.
 * @return true on success, false if the backend rejected the frequency.
 */
bool PLAT_PWM_SetFreqAndDuty(PWM_Instance_s* pwm, uint32_t freq_hz, float percent);

/* ========================================================================= */
/*  Construction (composition root only)                                     */
/* ========================================================================= */

/* Building an instance needs an ops vtable and a backend context, and both are
 * vendor symbols — so any caller of these is, by definition, naming a specific
 * chip. That is the composition root's job and nowhere else's.
 *
 * The gate makes that a compile error rather than a convention: an application or
 * device file that reaches for one of these has not defined
 * PLAT_ALLOW_CONSTRUCTION, so the declaration is not visible and the call fails
 * to compile. It gets its handles from the board layer instead, which is the only
 * place allowed to open this.
 *
 * Everything above this line takes an already-built handle and never mentions a
 * vendor, so it stays available to every layer. */
#ifdef PLAT_ALLOW_CONSTRUCTION

/**
 * @brief Initialize a PWM instance over caller-provided storage.
 *
 * The counterpart of PLAT_PWM_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_PWM_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_PWM_Init(PWM_Instance_s* inst, const PWM_Ops_s* ops, void* ctx);

/**
 * @brief Create a PWM instance from a backend-provided ops and context.
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one PWM output.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
PWM_Instance_s* PLAT_PWM_Create(const PWM_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_PWM_H */
