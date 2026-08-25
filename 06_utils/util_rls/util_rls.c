/**
 * @file util_rls.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_rls.h"

/* ========================================================================= */
/*  Internal helpers                                                         */
/* ========================================================================= */

/** @brief Floor on the denominator, so a rank-deficient step cannot divide by zero. */
#define DENOM_FLOOR 1.0e-12f

/** @brief Default trace cap as a multiple of the initial trace. */
#define DEFAULT_P_MAX_RATIO 10.0f

/** @brief Default excitation threshold on |x|^2. */
#define DEFAULT_X_EPS 1.0e-12f

/**
 * @brief Load P with @p diag on its diagonal and zero elsewhere.
 *
 * @param rls   Instance whose covariance is overwritten.
 * @param diag  Value for each diagonal entry.
 */
static void load_diagonal(UTIL_RLS_s* rls, float diag)
{
    uint16_t n = rls->n;

    for (uint16_t i = 0u; i < n; i++)
    {
        for (uint16_t j = 0u; j < n; j++)
        {
            rls->p_mat[(uint32_t) i * n + j] = (i == j) ? diag : 0.0f;
        }
    }
}

/**
 * @brief Rebuild the whole state after it went non-finite.
 *
 * Both the parameters and P are reset, not just the parameters: a NaN left in P
 * re-poisons the parameters on the very next step, which is exactly how the
 * legacy guard — it scanned only the parameter vector — let corruption persist.
 *
 * @param rls  Instance to rebuild.
 */
static void rebuild(UTIL_RLS_s* rls)
{
    for (uint16_t i = 0u; i < rls->n; i++)
    {
        rls->w[i] = 0.0f;
    }

    load_diagonal(rls, rls->p_init);

    rls->y_hat = 0.0f;
    rls->err   = 0.0f;
    rls->reset_count++;
}

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

bool UTIL_RLS_Init(UTIL_RLS_s* rls, float* buf, uint16_t n, float lambda, float p_init)
{
    if (rls == NULL)
    {
        return false;
    }

    rls->p_mat       = NULL;
    rls->w           = NULL;
    rls->k           = NULL;
    rls->v           = NULL;
    rls->n           = 0u;
    rls->lambda      = 1.0f;
    rls->inv_lambda  = 1.0f;
    rls->p_init      = 1.0f;
    rls->p_max       = 0.0f;
    rls->x_eps       = DEFAULT_X_EPS;
    rls->y_hat       = 0.0f;
    rls->err         = 0.0f;
    rls->reset_count = 0u;
    rls->initialized = false;

    if (buf == NULL || n == 0u || n > UTIL_RLS_MAX_DIM)
    {
        return false;
    }

    bool ok = true;

    /* Carve the caller's block: P first so it stays aligned at the front. */
    rls->p_mat = buf;
    rls->w     = buf + (uint32_t) n * n;
    rls->k     = rls->w + n;
    rls->v     = rls->k + n;
    rls->n     = n;

    /* lambda outside (0, 1] is not a mis-tuning but a broken recursion: 0
     * divides by zero and above 1 amplifies P every step. Clamp and report. */
    if (!UTIL_IsFinitef(lambda) || lambda <= 0.0f || lambda > 1.0f)
    {
        rls->lambda = (UTIL_IsFinitef(lambda) && lambda > 1.0f) ? 1.0f : 0.99f;
        ok          = false;
    }
    else
    {
        rls->lambda = lambda;
    }
    rls->inv_lambda = 1.0f / rls->lambda;

    if (!UTIL_IsFinitef(p_init) || p_init <= 0.0f)
    {
        rls->p_init = 1.0e6f;
        ok          = false;
    }
    else
    {
        rls->p_init = p_init;
    }

    /* Default cap at ten times the initial trace: high enough not to interfere
     * with honest convergence, low enough that an unexcited estimator plateaus
     * instead of overflowing. */
    rls->p_max = DEFAULT_P_MAX_RATIO * rls->p_init * (float) n;
    rls->x_eps = DEFAULT_X_EPS;

    rls->initialized = true;

    UTIL_RLS_Reset(rls);

    return ok;
}

void UTIL_RLS_SetGuards(UTIL_RLS_s* rls, float p_max, float x_eps)
{
    if (rls == NULL || !rls->initialized)
    {
        return;
    }

    if (UTIL_IsFinitef(p_max) && p_max >= 0.0f)
    {
        rls->p_max = p_max;
    }
    if (UTIL_IsFinitef(x_eps) && x_eps >= 0.0f)
    {
        rls->x_eps = x_eps;
    }
}

void UTIL_RLS_SetParams(UTIL_RLS_s* rls, const float* w)
{
    if (rls == NULL || !rls->initialized || w == NULL)
    {
        return;
    }

    for (uint16_t i = 0u; i < rls->n; i++)
    {
        if (UTIL_IsFinitef(w[i]))
        {
            rls->w[i] = w[i];
        }
    }
}

void UTIL_RLS_ResetCovariance(UTIL_RLS_s* rls)
{
    if (rls == NULL || !rls->initialized)
    {
        return;
    }

    load_diagonal(rls, rls->p_init);
}

void UTIL_RLS_Reset(UTIL_RLS_s* rls)
{
    if (rls == NULL || !rls->initialized)
    {
        return;
    }

    for (uint16_t i = 0u; i < rls->n; i++)
    {
        rls->w[i] = 0.0f;
        rls->k[i] = 0.0f;
        rls->v[i] = 0.0f;
    }

    load_diagonal(rls, rls->p_init);

    rls->y_hat = 0.0f;
    rls->err   = 0.0f;
}

float UTIL_RLS_GetTrace(const UTIL_RLS_s* rls)
{
    if (rls == NULL || !rls->initialized)
    {
        return 0.0f;
    }

    uint16_t n   = rls->n;
    float    acc = 0.0f;

    for (uint16_t i = 0u; i < n; i++)
    {
        acc += rls->p_mat[(uint32_t) i * n + i];
    }

    return acc;
}

float UTIL_RLS_Predict(const UTIL_RLS_s* rls, const float* x)
{
    if (rls == NULL || !rls->initialized || x == NULL)
    {
        return 0.0f;
    }

    float acc = 0.0f;

    for (uint16_t i = 0u; i < rls->n; i++)
    {
        acc += rls->w[i] * x[i];
    }

    return acc;
}

float UTIL_RLS_Step(UTIL_RLS_s* rls, const float* x, float y)
{
    if (rls == NULL || !rls->initialized || x == NULL)
    {
        return 0.0f;
    }

    if (!UTIL_IsFinitef(y))
    {
        return 0.0f;
    }

    uint16_t n = rls->n;

    /* Screen the regressor before touching any state: one non-finite entry
     * would otherwise spread through P and never leave. */
    float x_energy = 0.0f;
    for (uint16_t i = 0u; i < n; i++)
    {
        if (!UTIL_IsFinitef(x[i]))
        {
            return 0.0f;
        }
        x_energy += x[i] * x[i];
    }

    /* No excitation, no update. Skipping is not merely an optimisation: the P
     * recursion divides by lambda every step, so accepting empty samples is
     * precisely what drives the covariance wind-up that overflows a float in
     * about two seconds at lambda = 0.95 and 1 kHz. */
    if (x_energy <= rls->x_eps)
    {
        return 0.0f;
    }

    /* ---- v = P x, and denom = lambda + x . v ---- */

    float denom = rls->lambda;

    for (uint16_t i = 0u; i < n; i++)
    {
        const float* row = &rls->p_mat[(uint32_t) i * n];
        float        acc = 0.0f;

        for (uint16_t j = 0u; j < n; j++)
        {
            acc += row[j] * x[j];
        }

        rls->v[i] = acc;
        denom += x[i] * acc;
    }

    /* P is positive definite and lambda > 0, so denom should always be positive.
     * Floor its magnitude anyway: a rank-deficient regressor plus rounding can
     * bring it to zero, and one division by zero is enough to lose the state. */
    if (UTIL_Absf(denom) < DENOM_FLOOR)
    {
        denom = (denom < 0.0f) ? -DENOM_FLOOR : DENOM_FLOOR;
    }

    float inv_denom = 1.0f / denom;

    /* ---- k = v / denom ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        rls->k[i] = rls->v[i] * inv_denom;
    }

    /* ---- error against the model's own prediction ---- */

    float y_hat = 0.0f;
    for (uint16_t i = 0u; i < n; i++)
    {
        y_hat += rls->w[i] * x[i];
    }

    float err = y - y_hat;

    /* ---- w = w + k * err ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        rls->w[i] += rls->k[i] * err;
    }

    /* ---- P = (P - k v^T) / lambda, symmetrised ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        float* row_i = &rls->p_mat[(uint32_t) i * n];

        /* Diagonal has no mirror to average against. */
        row_i[i] = (row_i[i] - rls->k[i] * rls->v[i]) * rls->inv_lambda;

        for (uint16_t j = (uint16_t) (i + 1u); j < n; j++)
        {
            float* row_j = &rls->p_mat[(uint32_t) j * n];

            /* P is symmetric in exact arithmetic, but the two off-diagonal
             * halves are computed by different roundings and drift apart. Left
             * alone that asymmetry compounds until P stops being positive
             * definite and the gain points the wrong way, so average the pair
             * and write both — the standard fix, and cheaper than the alternative
             * of a square-root or UD factorisation. */
            float upper = (row_i[j] - rls->k[i] * rls->v[j]) * rls->inv_lambda;
            float lower = (row_j[i] - rls->k[j] * rls->v[i]) * rls->inv_lambda;
            float mean  = (upper + lower) * 0.5f;

            row_i[j] = mean;
            row_j[i] = mean;
        }
    }

    /* ---- Cap the trace ---- */

    if (rls->p_max > 0.0f)
    {
        float trace = 0.0f;
        for (uint16_t i = 0u; i < n; i++)
        {
            trace += rls->p_mat[(uint32_t) i * n + i];
        }

        if (trace > rls->p_max)
        {
            /* Scaled uniformly rather than reloaded as p_init*I: this keeps the
             * relative confidence the estimator has learned between directions,
             * where a reload would throw it away and jolt the adaptation rate. */
            float scale = rls->p_max / trace;

            for (uint32_t idx = 0u; idx < (uint32_t) n * n; idx++)
            {
                rls->p_mat[idx] *= scale;
            }
        }
    }

    /* ---- Commit, or rebuild ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        if (!UTIL_IsFinitef(rls->w[i]) || !UTIL_IsFinitef(rls->p_mat[(uint32_t) i * n + i]))
        {
            rebuild(rls);
            return 0.0f;
        }
    }

    rls->y_hat = y_hat;
    rls->err   = err;

    return err;
}
