/**
 * @file dev_power_limit.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef DEV_POWER_LIMIT_H
#define DEV_POWER_LIMIT_H

#include <stdbool.h>
#include <stdint.h>

#include "util_rls.h"

/** @brief Largest number of motors one limiter instance may manage. */
#define DEV_POWER_MAX_MOTORS 8u

/* ========================================================================= */
/*  Power model                                                              */
/* ========================================================================= */

/**
 * @brief Coefficients of the per-motor electrical power model.
 *
 * For one motor at angular rate w (rad/s) delivering torque T (N*m):
 *
 *     P = k_copper * T^2 + k_iron * w^2 + w * T
 *
 * The three terms are the copper loss (resistive, proportional to current
 * squared and so to torque squared), the iron and friction loss (which grows
 * with speed), and the mechanical output w*T. The last term needs no coefficient
 * because it is exact by definition; only the two loss terms are fitted.
 *
 * @par Where the numbers come from
 * Both losses depend on winding resistance, which rises with temperature, and on
 * bus voltage. Fixed values taken at bring-up drift over a match, which is why
 * this module can identify them online — see DEV_PowerLimit_FeedMeasured.
 */
typedef struct
{
    float k_copper; /**< Copper-loss coefficient, W per (N*m)^2. MUST be > 0:
                         it is the leading coefficient of the power curve, so a
                         zero or negative value makes the model non-physical. */
    float k_iron;   /**< Iron and friction loss, W per (rad/s)^2. >= 0.       */
    float k_static; /**< Constant drivetrain draw, W, shared across all
                         motors. Covers the parts that do not scale with
                         torque or speed at all.                             */
} DEV_Power_Model_s;

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/**
 * @brief Everything that describes a limiter, separate from its state.
 *
 * @par Zero-initialising is NOT safe
 * Unlike the filters in 06_utils, a zeroed config is meaningless here: a budget
 * of zero would stall the chassis and a @c k_copper of zero would divide by
 * zero. DEV_PowerLimit_Init rejects it rather than substituting defaults,
 * because there is no safe default for "how much power may this robot draw".
 */
typedef struct
{
    uint16_t motor_count; /**< Motors to manage; 1 to @ref DEV_POWER_MAX_MOTORS. */

    DEV_Power_Model_s model; /**< Initial power model coefficients. */

    float power_budget; /**< Total electrical power allowed, W. MUST be > 0. */

    /**
     * @brief Conversion from a controller output unit to motor torque, N*m.
     *
     * The controller works in whatever unit its output register takes — for a DJI
     * ESC that is a raw current command. This factor turns one such unit into
     * newton-metres so the power model can be evaluated, and is applied in
     * reverse on the way out. Getting it wrong scales the whole power estimate,
     * so measure it rather than guessing.
     */
    float torque_per_output;

    /**
     * @brief Motor rate per unit of reported velocity, rad/s.
     *
     * Feedback usually arrives in rpm, in which case this is 2*pi/60 = 0.10472.
     * Kept explicit so a driver reporting rad/s directly can pass 1.0 instead of
     * pre-scaling.
     */
    float rate_per_velocity;

    /**
     * @brief Forgetting factor for the online model identification, in (0, 1].
     *
     * 1.0 never forgets; 0.999 at 1 kHz tracks a change over about a second.
     * Only used when DEV_PowerLimit_FeedMeasured is called; 0 selects a sane
     * default.
     */
    float rls_lambda;

    /**
     * @brief Bounds the identified @c k_copper may not leave.
     *
     * The identifier is a least-squares fit with no notion of physics: fed noisy
     * data it can return a negative leading coefficient, which would invert the
     * power curve and — since @c k_copper is the denominator of the torque
     * solution — divide by zero on the way through. These bounds are what makes
     * online identification safe to enable. Both 0 selects defaults derived from
     * the initial model.
     */
    float k_copper_min;
    float k_copper_max;
} DEV_PowerLimit_Cfg_s;

/* ========================================================================= */
/*  Feedback                                                                 */
/* ========================================================================= */

/**
 * @brief One sample of motor state, as measured.
 *
 * Plain arrays rather than motor handles, matching dev_steer_chassis: the module
 * has no opinion about which ESC or CAN driver produced them, so it stays
 * testable on a host and reusable across motor types.
 */
typedef struct
{
    const float* velocity; /**< Reported velocity per motor, usually rpm.     */
    const float* current;  /**< Reported current per motor, in controller
                                output units — the same unit as the PID output
                                and as @c torque_per_output expects.         */
} DEV_Power_Feedback_s;

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief Chassis electrical power limiter over caller-owned storage.
 *
 * Scales a set of controller outputs so their combined electrical draw stays
 * within a budget, which is what keeps a robot from tripping its own power
 * supply under hard acceleration.
 *
 * @par One global scale factor, not a per-motor budget
 * When the request exceeds the budget, every output is multiplied by the same
 * factor s in [0, 1], found by bisection on total predicted power. The
 * alternative — dividing the budget between motors and solving each one's torque
 * separately — changes the ratio between the four wheels, which on a chassis
 * means the robot turns while it is being limited. A single factor cannot do
 * that: the commanded direction is preserved exactly and only the magnitude
 * falls. The legacy implementation took the per-motor route.
 *
 * @par Why bisection rather than a closed form
 * Total power as a function of s is a sum of upward parabolas, so it is
 * continuous and monotonic in s over [0, 1] wherever it matters, and bisection
 * converges to a relative 1e-3 in about ten iterations with no division and no
 * square root. A closed-form solve of the summed quadratic needs a discriminant
 * that can go negative, which is exactly where the legacy code fell back to the
 * parabola's vertex — a value that is not a root of anything and drove motors
 * backwards.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation, no HAL dependency. One owning
 * context per instance.
 */
typedef struct
{
    DEV_PowerLimit_Cfg_s cfg;   /**< Configuration, copied at Init. */
    DEV_Power_Model_s    model; /**< Live model; tracks cfg.model unless
                                     identification is running.     */

    UTIL_RLS_s rls;                           /**< Online identifier for the two loss terms. */
    float      rls_buf[UTIL_RLS_BUF_SIZE(2)]; /**< Storage for it.         */
    bool       rls_ready;                     /**< True once Init has set the identifier up.  */

    float rate[DEV_POWER_MAX_MOTORS];   /**< Cached rate per motor, rad/s.   */
    float torque[DEV_POWER_MAX_MOTORS]; /**< Cached measured torque, N*m.    */

    float requested_power; /**< Predicted draw of the raw request, W.  */
    float limited_power;   /**< Predicted draw after scaling, W.       */
    float scale;           /**< Last scale factor applied, in [0, 1].  */

    uint32_t limit_count; /**< Steps where scaling was needed.         */
    uint32_t step_count;  /**< Steps run since Init.                   */

    bool initialized; /**< False until Init succeeds. */
} DEV_PowerLimit_s;

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

/**
 * @brief Initialize a limiter from a configuration.
 *
 * The configuration is copied, so the caller's struct may be a temporary.
 *
 * @param pl   Instance to initialize.
 * @param cfg  Configuration to copy. Every MUST-constraint noted on
 *             DEV_PowerLimit_Cfg_s is checked.
 * @return true on success; false if @p pl or @p cfg is NULL, @c motor_count is
 *         out of range, @c power_budget or @c k_copper is not positive, or a
 *         conversion factor is zero — in which case Update passes its input
 *         through unchanged rather than scaling by a nonsense factor.
 */
bool DEV_PowerLimit_Init(DEV_PowerLimit_s* pl, const DEV_PowerLimit_Cfg_s* cfg);

/**
 * @brief Change the power budget without disturbing the model or identifier.
 *
 * For a referee system that raises or lowers the allowance mid-match, or for
 * switching to a capacitor-boosted budget.
 *
 * @param pl      Instance to retune.
 * @param budget  New total allowance, W. A non-positive or non-finite value is
 *                ignored, so a dropped telemetry frame cannot stall the chassis.
 */
void DEV_PowerLimit_SetBudget(DEV_PowerLimit_s* pl, float budget);

/**
 * @brief Overwrite the power model, stopping any online identification drift.
 *
 * @param pl     Instance to configure.
 * @param model  New coefficients. Rejected as a whole if @c k_copper is not
 *               positive or any field is non-finite.
 */
void DEV_PowerLimit_SetModel(DEV_PowerLimit_s* pl, const DEV_Power_Model_s* model);

/**
 * @brief Feed a measured total power so the model can identify itself.
 *
 * Call this once per cycle when a real measurement is available — from a power
 * meter, a supercapacitor board, or the referee system. Without it the model
 * stays at whatever was configured, which is a perfectly usable open-loop mode:
 * identification improves accuracy as the motors warm up, it is not required for
 * the limiter to function.
 *
 * The fit is a two-parameter recursive least squares against the sums of squared
 * torque and squared rate accumulated by the previous Update, with the result
 * constrained to the bounds in the configuration. Call it AFTER Update so the
 * regressor matches the sample the measurement belongs to.
 *
 * @par Only k_copper is bounded, and why
 * The two regressors differ enormously in scale — summed squared torque is order
 * 1e-2 while summed squared rate is order 1e6, a ratio near 1e7 — even though
 * their contributions to total power are comparable. With honest measurements
 * that is harmless: both coefficients identify to about 1e-7 relative error over
 * 30k samples. With noisy or poorly excited data the small-regressor coefficient
 * is the one that degrades first, which is why @c k_copper carries explicit
 * bounds: it is the denominator of the power curve, so a bad fit there is a
 * safety problem rather than an accuracy one. @c k_iron is only floored at zero,
 * because an overestimate of it makes the limiter more conservative, never less.
 *
 * @param pl              Instance to update.
 * @param measured_power  Total measured electrical power, W. A non-finite or
 *                        negative value is ignored.
 * @return true when the sample was used; false when it was rejected or no Update
 *         has run yet.
 */
bool DEV_PowerLimit_FeedMeasured(DEV_PowerLimit_s* pl, float measured_power);

/**
 * @brief Predict the electrical power a set of outputs would draw, in watts.
 *
 * Exposed for logging and for a caller that wants to check a candidate command
 * before issuing it.
 *
 * @param pl      Instance to query.
 * @param fdb     Current motor feedback; the rates come from here.
 * @param outputs Candidate controller outputs, @c motor_count elements.
 * @return Predicted total draw including @c k_static, or 0 if any argument is
 *         NULL or the instance is uninitialized.
 */
float DEV_PowerLimit_Predict(const DEV_PowerLimit_s* pl, const DEV_Power_Feedback_s* fdb,
                             const float* outputs);

/**
 * @brief Scale a set of controller outputs to fit the power budget.
 *
 * When the request already fits, the outputs are copied through untouched and
 * the scale factor is 1 — the limiter is inert until it is actually needed.
 *
 * @param pl       Instance to advance.
 * @param fdb      Motor feedback for this cycle. Both arrays must hold at least
 *                 @c motor_count elements.
 * @param requests Desired controller outputs, @c motor_count elements, in the
 *                 same units the ESC takes.
 * @param outputs  Destination for the scaled outputs, @c motor_count elements.
 *                 May alias @p requests.
 * @return true when a valid command was produced. On false — a NULL argument, an
 *         uninitialized instance, or a non-finite input — @p outputs is filled
 *         with zeros, because a chassis that stops is safer than one driven by
 *         an unchecked value.
 *
 * @note A non-finite entry in @p fdb is treated as a zero rate for that motor
 *       rather than failing the whole call: one silent CAN frame should not cost
 *       control of the other three wheels.
 */
bool DEV_PowerLimit_Update(DEV_PowerLimit_s* pl, const DEV_Power_Feedback_s* fdb,
                           const float* requests, float* outputs);

/* ========================================================================= */
/*  Inspection                                                               */
/* ========================================================================= */

/**
 * @brief Scale factor applied by the last Update.
 * @param pl  Instance to query.
 * @return Factor in [0, 1]; 1 means the request fitted the budget untouched.
 */
static inline float DEV_PowerLimit_GetScale(const DEV_PowerLimit_s* pl) { return pl->scale; }

/**
 * @brief Predicted draw of the last raw request, before scaling, in watts.
 *
 * Compare against the budget to see how far over the controllers were asking.
 *
 * @param pl  Instance to query.
 * @return Requested power, W.
 */
static inline float DEV_PowerLimit_GetRequestedPower(const DEV_PowerLimit_s* pl)
{
    return pl->requested_power;
}

/**
 * @brief Predicted draw of the outputs actually issued, in watts.
 * @param pl  Instance to query.
 * @return Limited power, W; never above the budget by more than the bisection
 *         tolerance.
 */
static inline float DEV_PowerLimit_GetLimitedPower(const DEV_PowerLimit_s* pl)
{
    return pl->limited_power;
}

/**
 * @brief The live power model, including any online identification.
 * @param pl  Instance to query.
 * @return Pointer to the current coefficients.
 */
static inline const DEV_Power_Model_s* DEV_PowerLimit_GetModel(const DEV_PowerLimit_s* pl)
{
    return &pl->model;
}

/**
 * @brief Fraction of steps that needed scaling, in [0, 1].
 *
 * Worth logging: a value near 1 means the chassis is permanently power-starved,
 * i.e. the operator is asking for more than the robot can deliver and the
 * limiter is the only thing deciding what it actually does.
 *
 * @param pl  Instance to query.
 * @return Ratio of limited steps to total steps; 0 before the first Update.
 */
float DEV_PowerLimit_GetLimitRatio(const DEV_PowerLimit_s* pl);

#endif /* DEV_POWER_LIMIT_H */
