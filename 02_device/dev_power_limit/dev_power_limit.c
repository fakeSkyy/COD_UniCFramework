/**
 * @file dev_power_limit.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "dev_power_limit.h"

#include "util_fast_math.h"

/* ========================================================================= */
/*  Tuning constants                                                         */
/* ========================================================================= */

/**
 * @brief Bisection iterations for the scale factor.
 *
 * Each halves the interval, so 12 from a width of 1 leaves 2.4e-4 — finer than
 * the power model itself is accurate, and a fixed count means the cost is
 * constant and the loop cannot fail to terminate.
 */
#define BISECT_ITERATIONS 12u

/** @brief Stop early once the bracket is this narrow; saves the tail iterations. */
#define BISECT_TOLERANCE 1.0e-3f

/** @brief Default forgetting factor for the identifier. */
#define DEFAULT_RLS_LAMBDA 0.999f

/**
 * @brief Initial covariance for the identifier.
 *
 * Modest on purpose: util_rls documents that the covariance update cancels about
 * log10(p_init) decimal digits, so a large value costs accuracy in single
 * precision. The initial model is a real calibration rather than a guess, so
 * declaring near-total ignorance would be wrong anyway.
 */
#define RLS_P_INIT 1.0e3f

/** @brief Default lower bound on the identified copper coefficient, as a ratio. */
#define K_COPPER_MIN_RATIO 0.25f

/** @brief Default upper bound on the identified copper coefficient, as a ratio. */
#define K_COPPER_MAX_RATIO 4.0f

/* ========================================================================= */
/*  Internal helpers                                                         */
/* ========================================================================= */

/**
 * @brief Electrical power of one motor at a given rate and torque.
 *
 * @param model  Coefficients to evaluate with.
 * @param rate   Angular rate, rad/s.
 * @param torque Torque, N*m.
 * @return Power in watts, excluding the static term.
 */
static float motor_power(const DEV_Power_Model_s* model, float rate, float torque)
{
    return model->k_copper * torque * torque + model->k_iron * rate * rate + rate * torque;
}

/**
 * @brief Total power drawn if every request were scaled by @p s.
 *
 * Uses the rates cached by the current Update, so it is only meaningful inside
 * one. Torque scales linearly with the output, so the copper term scales with
 * s^2 and the mechanical term with s — which is why the total is a parabola in s
 * and why bisection is well behaved on it.
 *
 * @param pl        Instance holding the cached rates.
 * @param requests  Raw controller outputs.
 * @param s         Candidate scale factor.
 * @return Predicted total draw including the static term, W.
 */
static float total_power_at(const DEV_PowerLimit_s* pl, const float* requests, float s)
{
    float acc = pl->model.k_static;

    for (uint16_t i = 0u; i < pl->cfg.motor_count; i++)
    {
        float torque = requests[i] * s * pl->cfg.torque_per_output;

        acc += motor_power(&pl->model, pl->rate[i], torque);
    }

    return acc;
}

/**
 * @brief Constrain the identified coefficients to physically valid values.
 *
 * A least-squares fit has no notion of physics: fed noisy or poorly excited data
 * it can return a negative leading coefficient, which inverts the power curve.
 * Since that coefficient is also the denominator of any torque solve, letting it
 * through would be a divide by zero one step later. The legacy code wrote the
 * identifier's raw output straight into the model.
 *
 * @param pl  Instance whose live model is clamped.
 */
static void clamp_model(DEV_PowerLimit_s* pl)
{
    pl->model.k_copper =
        UTIL_Clampf(pl->model.k_copper, pl->cfg.k_copper_min, pl->cfg.k_copper_max);

    /* The iron term is a loss, so it cannot be negative; it has no upper bound
     * that is meaningful independently of the rate range. */
    if (!(pl->model.k_iron >= 0.0f) || !UTIL_IsFinitef(pl->model.k_iron))
    {
        pl->model.k_iron = 0.0f;
    }

    if (!UTIL_IsFinitef(pl->model.k_static))
    {
        pl->model.k_static = 0.0f;
    }
}

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

bool DEV_PowerLimit_Init(DEV_PowerLimit_s* pl, const DEV_PowerLimit_Cfg_s* cfg)
{
    if (pl == NULL)
    {
        return false;
    }

    for (uint16_t i = 0u; i < DEV_POWER_MAX_MOTORS; i++)
    {
        pl->rate[i]   = 0.0f;
        pl->torque[i] = 0.0f;
    }

    pl->requested_power = 0.0f;
    pl->limited_power   = 0.0f;
    pl->scale           = 1.0f;
    pl->limit_count     = 0u;
    pl->step_count      = 0u;
    pl->rls_ready       = false;
    pl->initialized     = false;

    if (cfg == NULL)
    {
        return false;
    }

    /* Every one of these is a value there is no safe substitute for: a zero
     * budget stalls the chassis, a zero k_copper divides by zero, and a zero
     * conversion factor makes the whole power estimate identically the static
     * term. Refuse rather than invent a default. */
    if (cfg->motor_count == 0u || cfg->motor_count > DEV_POWER_MAX_MOTORS)
    {
        return false;
    }
    if (!UTIL_IsFinitef(cfg->power_budget) || cfg->power_budget <= 0.0f)
    {
        return false;
    }
    if (!UTIL_IsFinitef(cfg->model.k_copper) || cfg->model.k_copper <= 0.0f)
    {
        return false;
    }
    if (!UTIL_IsFinitef(cfg->model.k_iron) || cfg->model.k_iron < 0.0f)
    {
        return false;
    }
    if (!UTIL_IsFinitef(cfg->model.k_static))
    {
        return false;
    }
    if (!UTIL_IsFinitef(cfg->torque_per_output) || cfg->torque_per_output <= 0.0f)
    {
        return false;
    }
    if (!UTIL_IsFinitef(cfg->rate_per_velocity) || cfg->rate_per_velocity <= 0.0f)
    {
        return false;
    }

    pl->cfg   = *cfg;
    pl->model = cfg->model;

    if (!UTIL_IsFinitef(pl->cfg.rls_lambda) || pl->cfg.rls_lambda <= 0.0f ||
        pl->cfg.rls_lambda > 1.0f)
    {
        pl->cfg.rls_lambda = DEFAULT_RLS_LAMBDA;
    }

    /* Bounds as a band around the calibrated value: it came from a real
     * measurement, so a fit four times larger or four times smaller is not a
     * refinement, it is a symptom of bad data. */
    if (!(pl->cfg.k_copper_min > 0.0f) || !UTIL_IsFinitef(pl->cfg.k_copper_min))
    {
        pl->cfg.k_copper_min = K_COPPER_MIN_RATIO * cfg->model.k_copper;
    }
    if (!(pl->cfg.k_copper_max > pl->cfg.k_copper_min) || !UTIL_IsFinitef(pl->cfg.k_copper_max))
    {
        pl->cfg.k_copper_max = K_COPPER_MAX_RATIO * cfg->model.k_copper;
    }

    /* The identifier fits the two loss coefficients against the summed squares
     * of torque and rate; the mechanical term needs no coefficient, so it is
     * subtracted from the measurement instead of being fitted. */
    pl->rls_ready = UTIL_RLS_Init(&pl->rls, pl->rls_buf, 2u, pl->cfg.rls_lambda, RLS_P_INIT);

    if (pl->rls_ready)
    {
        float w0[2] = {pl->model.k_copper, pl->model.k_iron};
        UTIL_RLS_SetParams(&pl->rls, w0);
    }

    pl->initialized = true;
    return true;
}

void DEV_PowerLimit_SetBudget(DEV_PowerLimit_s* pl, float budget)
{
    if (pl == NULL || !pl->initialized)
    {
        return;
    }

    /* Ignore a bad value rather than store it: this is fed from telemetry, and a
     * dropped frame must not stall the chassis. */
    if (UTIL_IsFinitef(budget) && budget > 0.0f)
    {
        pl->cfg.power_budget = budget;
    }
}

void DEV_PowerLimit_SetModel(DEV_PowerLimit_s* pl, const DEV_Power_Model_s* model)
{
    if (pl == NULL || !pl->initialized || model == NULL)
    {
        return;
    }

    if (!UTIL_IsFinitef(model->k_copper) || model->k_copper <= 0.0f)
    {
        return;
    }
    if (!UTIL_IsFinitef(model->k_iron) || model->k_iron < 0.0f)
    {
        return;
    }
    if (!UTIL_IsFinitef(model->k_static))
    {
        return;
    }

    pl->model = *model;
    clamp_model(pl);

    /* Re-seed the identifier so it continues from the new model instead of
     * dragging the old estimate back. */
    if (pl->rls_ready)
    {
        float w0[2] = {pl->model.k_copper, pl->model.k_iron};
        UTIL_RLS_SetParams(&pl->rls, w0);
    }
}

/* ========================================================================= */
/*  Prediction and limiting                                                  */
/* ========================================================================= */

float DEV_PowerLimit_Predict(const DEV_PowerLimit_s* pl, const DEV_Power_Feedback_s* fdb,
                             const float* outputs)
{
    if (pl == NULL || !pl->initialized || fdb == NULL || fdb->velocity == NULL || outputs == NULL)
    {
        return 0.0f;
    }

    float acc = pl->model.k_static;

    for (uint16_t i = 0u; i < pl->cfg.motor_count; i++)
    {
        float vel = UTIL_IsFinitef(fdb->velocity[i]) ? fdb->velocity[i] : 0.0f;
        float out = UTIL_IsFinitef(outputs[i]) ? outputs[i] : 0.0f;

        float rate   = vel * pl->cfg.rate_per_velocity;
        float torque = out * pl->cfg.torque_per_output;

        acc += motor_power(&pl->model, rate, torque);
    }

    return acc;
}

bool DEV_PowerLimit_Update(DEV_PowerLimit_s* pl, const DEV_Power_Feedback_s* fdb,
                           const float* requests, float* outputs)
{
    if (pl == NULL || outputs == NULL)
    {
        return false;
    }

    uint16_t n = pl->initialized ? pl->cfg.motor_count : 0u;

    if (!pl->initialized || fdb == NULL || fdb->velocity == NULL || requests == NULL)
    {
        /* Zero the outputs rather than leave whatever the caller had there. A
         * stopped chassis is recoverable; one driven by an unvalidated value is
         * not necessarily. */
        for (uint16_t i = 0u; i < n; i++)
        {
            outputs[i] = 0.0f;
        }
        return false;
    }

    /* ---- Cache the measured state ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        /* A non-finite feedback becomes a zero rate for that motor only: one
         * silent CAN frame should not cost control of the other wheels. */
        float vel = UTIL_IsFinitef(fdb->velocity[i]) ? fdb->velocity[i] : 0.0f;

        pl->rate[i] = vel * pl->cfg.rate_per_velocity;

        float cur = 0.0f;
        if (fdb->current != NULL && UTIL_IsFinitef(fdb->current[i]))
        {
            cur = fdb->current[i];
        }
        pl->torque[i] = cur * pl->cfg.torque_per_output;
    }

    /* A non-finite request is the one thing that cannot be salvaged — it is the
     * value about to reach the motor. */
    for (uint16_t i = 0u; i < n; i++)
    {
        if (!UTIL_IsFinitef(requests[i]))
        {
            for (uint16_t j = 0u; j < n; j++)
            {
                outputs[j] = 0.0f;
            }
            pl->scale         = 0.0f;
            pl->limited_power = 0.0f;
            pl->step_count++;
            return false;
        }
    }

    pl->step_count++;

    /* ---- Does the raw request already fit? ---- */

    float requested = total_power_at(pl, requests, 1.0f);

    pl->requested_power = requested;

    if (requested <= pl->cfg.power_budget)
    {
        /* Inert until actually needed: copy through untouched. */
        for (uint16_t i = 0u; i < n; i++)
        {
            outputs[i] = requests[i];
        }

        pl->scale         = 1.0f;
        pl->limited_power = requested;
        return true;
    }

    /* ---- Bisect for the largest scale that fits ---- */

    /* At s = 0 the draw is k_static plus the iron loss, both independent of the
     * command. If even that exceeds the budget there is nothing to scale — the
     * chassis is coasting above its allowance and the only honest answer is zero
     * torque. */
    float p_zero = total_power_at(pl, requests, 0.0f);

    if (p_zero >= pl->cfg.power_budget)
    {
        for (uint16_t i = 0u; i < n; i++)
        {
            outputs[i] = 0.0f;
        }

        pl->scale         = 0.0f;
        pl->limited_power = p_zero;
        pl->limit_count++;
        return true;
    }

    float lo = 0.0f; /* known to fit     */
    float hi = 1.0f; /* known to exceed  */

    for (uint16_t it = 0u; it < BISECT_ITERATIONS; it++)
    {
        if ((hi - lo) < BISECT_TOLERANCE)
        {
            break;
        }

        float mid = (lo + hi) * 0.5f;

        if (total_power_at(pl, requests, mid) <= pl->cfg.power_budget)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    /* Take the low end: it is the side proven to fit the budget. */
    for (uint16_t i = 0u; i < n; i++)
    {
        outputs[i] = requests[i] * lo;
    }

    pl->scale         = lo;
    pl->limited_power = total_power_at(pl, requests, lo);
    pl->limit_count++;

    return true;
}

/* ========================================================================= */
/*  Online identification                                                    */
/* ========================================================================= */

bool DEV_PowerLimit_FeedMeasured(DEV_PowerLimit_s* pl, float measured_power)
{
    if (pl == NULL || !pl->initialized || !pl->rls_ready)
    {
        return false;
    }

    if (pl->step_count == 0u)
    {
        return false;
    }

    if (!UTIL_IsFinitef(measured_power) || measured_power < 0.0f)
    {
        return false;
    }

    /* Regressor is the summed squares; the mechanical term w*T is exact by
     * definition, so it is removed from the measurement rather than fitted. */
    float x[2]       = {0.0f, 0.0f};
    float mechanical = 0.0f;

    for (uint16_t i = 0u; i < pl->cfg.motor_count; i++)
    {
        x[0] += pl->torque[i] * pl->torque[i];
        x[1] += pl->rate[i] * pl->rate[i];
        mechanical += pl->rate[i] * pl->torque[i];
    }

    float y = measured_power - mechanical - pl->model.k_static;

    UTIL_RLS_Step(&pl->rls, x, y);

    /* Adopt the fit, then constrain it. Reading the parameters back out and
     * clamping is what keeps a bad fit from reaching the torque solve — the
     * legacy code assigned them unconditionally. */
    pl->model.k_copper = UTIL_RLS_GetParam(&pl->rls, 0);
    pl->model.k_iron   = UTIL_RLS_GetParam(&pl->rls, 1);

    clamp_model(pl);

    /* Push the constrained values back into the identifier so it continues from
     * a physical state rather than repeatedly proposing the same bad fit. */
    float w[2] = {pl->model.k_copper, pl->model.k_iron};
    UTIL_RLS_SetParams(&pl->rls, w);

    return true;
}

float DEV_PowerLimit_GetLimitRatio(const DEV_PowerLimit_s* pl)
{
    if (pl == NULL || !pl->initialized || pl->step_count == 0u)
    {
        return 0.0f;
    }

    return (float) pl->limit_count / (float) pl->step_count;
}
