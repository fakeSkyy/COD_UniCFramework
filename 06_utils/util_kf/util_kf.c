/**
 * @file util_kf.c
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#include "util_kf.h"

/* ========================================================================= */
/*  Internal helpers                                                         */
/* ========================================================================= */

/** @brief Floor on the innovation variance, so a scalar update cannot divide by zero. */
#define DENOM_FLOOR 1.0e-20f

/** @brief Default diagonal loaded into P when the state has to be rebuilt. */
#define DEFAULT_P_RESET 1.0f

/**
 * @brief Rebuild the state after it went non-finite.
 *
 * Both x and P are reloaded, not just x: a NaN left anywhere in P re-poisons the
 * state on the very next Predict, which is how the legacy guard — it scanned only
 * the state vector — let corruption persist indefinitely.
 *
 * @param kf  Instance to rebuild.
 */
static void rebuild(UTIL_KF_s* kf)
{
    uint16_t n = kf->n;

    for (uint16_t i = 0u; i < n; i++)
    {
        kf->x[i]       = 0.0f;
        kf->x_prior[i] = 0.0f;

        for (uint16_t j = 0u; j < n; j++)
        {
            kf->p_mat[(uint32_t) i * n + j] = (i == j) ? kf->p_reset : 0.0f;
        }
    }

    kf->innovation = 0.0f;
    kf->predicted  = false;
    kf->reset_count++;
}

/**
 * @brief Test whether x and the diagonal of P are all finite.
 *
 * The diagonal is enough to catch corruption: an off-diagonal NaN reaches the
 * diagonal within one Joseph update, because every term there is a product that
 * spans both.
 *
 * @param kf  Instance to check.
 * @return true when the state is usable.
 */
static bool state_is_finite(const UTIL_KF_s* kf)
{
    uint16_t n = kf->n;

    for (uint16_t i = 0u; i < n; i++)
    {
        if (!UTIL_IsFinitef(kf->x[i]) || !UTIL_IsFinitef(kf->p_mat[(uint32_t) i * n + i]))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Average P against its own transpose.
 *
 * The Joseph form is symmetric in exact arithmetic, but the two halves are
 * reached by different roundings and drift apart. Averaging costs n(n-1)/2 adds
 * and removes the drift entirely rather than letting it accumulate into a loss
 * of positive definiteness.
 *
 * @param kf  Instance whose covariance is symmetrised.
 */
static void symmetrise(UTIL_KF_s* kf)
{
    uint16_t n = kf->n;

    for (uint16_t i = 0u; i < n; i++)
    {
        for (uint16_t j = (uint16_t) (i + 1u); j < n; j++)
        {
            uint32_t ij   = (uint32_t) i * n + j;
            uint32_t ji   = (uint32_t) j * n + i;
            float    mean = (kf->p_mat[ij] + kf->p_mat[ji]) * 0.5f;

            kf->p_mat[ij] = mean;
            kf->p_mat[ji] = mean;
        }
    }
}

/* ========================================================================= */
/*  Setup                                                                    */
/* ========================================================================= */

bool UTIL_KF_Init(UTIL_KF_s* kf, float* buf, uint16_t n, uint16_t z)
{
    if (kf == NULL)
    {
        return false;
    }

    kf->a_mat        = NULL;
    kf->h_mat        = NULL;
    kf->p_mat        = NULL;
    kf->q_mat        = NULL;
    kf->r_diag       = NULL;
    kf->x            = NULL;
    kf->x_prior      = NULL;
    kf->k_gain       = NULL;
    kf->scratch_n    = NULL;
    kf->scratch_nn   = NULL;
    kf->innovation   = 0.0f;
    kf->n            = 0u;
    kf->z            = 0u;
    kf->reject_count = 0u;
    kf->reject_run   = 0u;
    kf->gate_max_run = 0u;
    kf->reset_count  = 0u;
    kf->p_reset      = DEFAULT_P_RESET;
    kf->gate_sigma   = 0.0f;
    kf->initialized  = false;
    kf->predicted    = false;

    if (buf == NULL || n == 0u || n > UTIL_KF_MAX_DIM || z == 0u || z > UTIL_KF_MAX_MEAS)
    {
        return false;
    }

    uint32_t nn = (uint32_t) n * n;
    float*   p  = buf;

    /* Carve the caller's block. Matrices first so they stay word-aligned at the
     * front regardless of n and z. */
    kf->a_mat = p;
    p += nn;
    kf->p_mat = p;
    p += nn;
    kf->q_mat = p;
    p += nn;
    kf->scratch_nn = p;
    p += nn;
    kf->h_mat = p;
    p += (uint32_t) z * n;
    kf->r_diag = p;
    p += z;
    kf->x = p;
    p += n;
    kf->x_prior = p;
    p += n;
    kf->k_gain = p;
    p += n;
    kf->scratch_n = p;

    kf->n = n;
    kf->z = z;

    /* A as the identity, everything else zero: the identity is the only A that
     * means "no dynamics assumed", whereas a zeroed A would drive the state to
     * zero on the first Predict. */
    for (uint16_t i = 0u; i < n; i++)
    {
        kf->x[i]       = 0.0f;
        kf->x_prior[i] = 0.0f;
        kf->k_gain[i]  = 0.0f;

        for (uint16_t j = 0u; j < n; j++)
        {
            uint32_t idx   = (uint32_t) i * n + j;
            kf->a_mat[idx] = (i == j) ? 1.0f : 0.0f;
            kf->p_mat[idx] = 0.0f;
            kf->q_mat[idx] = 0.0f;
        }
    }

    for (uint16_t i = 0u; i < z; i++)
    {
        kf->r_diag[i] = 0.0f;

        for (uint16_t j = 0u; j < n; j++)
        {
            kf->h_mat[(uint32_t) i * n + j] = 0.0f;
        }
    }

    kf->initialized = true;
    return true;
}

void UTIL_KF_SetCovarianceDiag(UTIL_KF_s* kf, const float* diag)
{
    if (kf == NULL || !kf->initialized || diag == NULL)
    {
        return;
    }

    uint16_t n = kf->n;

    for (uint16_t i = 0u; i < n; i++)
    {
        /* A zero variance asserts the state is known exactly, which zeroes the
         * gain for it permanently. Refuse it rather than silently deafen the
         * filter to that state. */
        if (UTIL_IsFinitef(diag[i]) && diag[i] > 0.0f)
        {
            kf->p_mat[(uint32_t) i * n + i] = diag[i];
        }
    }
}

void UTIL_KF_SetProcessNoiseDiag(UTIL_KF_s* kf, const float* diag)
{
    if (kf == NULL || !kf->initialized || diag == NULL)
    {
        return;
    }

    uint16_t n = kf->n;

    for (uint16_t i = 0u; i < n; i++)
    {
        if (UTIL_IsFinitef(diag[i]) && diag[i] >= 0.0f)
        {
            kf->q_mat[(uint32_t) i * n + i] = diag[i];
        }
    }
}

void UTIL_KF_SetMeasurementNoise(UTIL_KF_s* kf, const float* diag)
{
    if (kf == NULL || !kf->initialized || diag == NULL)
    {
        return;
    }

    for (uint16_t i = 0u; i < kf->z; i++)
    {
        /* R is the denominator of the gain; zero would divide by zero. */
        if (UTIL_IsFinitef(diag[i]) && diag[i] > 0.0f)
        {
            kf->r_diag[i] = diag[i];
        }
    }
}

void UTIL_KF_SetGuards(UTIL_KF_s* kf, float gate_sigma, float p_reset)
{
    if (kf == NULL || !kf->initialized)
    {
        return;
    }

    if (UTIL_IsFinitef(gate_sigma) && gate_sigma >= 0.0f)
    {
        kf->gate_sigma = gate_sigma;
    }
    if (UTIL_IsFinitef(p_reset) && p_reset > 0.0f)
    {
        kf->p_reset = p_reset;
    }
}

void UTIL_KF_SetGateMaxRun(UTIL_KF_s* kf, uint32_t max_run)
{
    if (kf == NULL || !kf->initialized)
    {
        return;
    }

    kf->gate_max_run = max_run;
    kf->reject_run   = 0u;
}

void UTIL_KF_Reset(UTIL_KF_s* kf)
{
    if (kf == NULL || !kf->initialized)
    {
        return;
    }

    uint32_t saved = kf->reset_count;

    rebuild(kf);

    /* rebuild() counts an involuntary recovery; an explicit Reset is not one. */
    kf->reset_count = saved;
}

float UTIL_KF_GetTrace(const UTIL_KF_s* kf)
{
    if (kf == NULL || !kf->initialized)
    {
        return 0.0f;
    }

    uint16_t n   = kf->n;
    float    acc = 0.0f;

    for (uint16_t i = 0u; i < n; i++)
    {
        acc += kf->p_mat[(uint32_t) i * n + i];
    }

    return acc;
}

/* ========================================================================= */
/*  Filtering                                                               */
/* ========================================================================= */

bool UTIL_KF_Predict(UTIL_KF_s* kf)
{
    if (kf == NULL || !kf->initialized)
    {
        return false;
    }

    uint16_t n = kf->n;

    /* ---- x = A x ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        const float* a_row = &kf->a_mat[(uint32_t) i * n];
        float        acc   = 0.0f;

        for (uint16_t j = 0u; j < n; j++)
        {
            acc += a_row[j] * kf->x[j];
        }

        kf->x_prior[i] = acc;
    }

    for (uint16_t i = 0u; i < n; i++)
    {
        kf->x[i] = kf->x_prior[i];
    }

    /* ---- P = A P A' + Q ---- */

    /* scratch = A P */
    for (uint16_t i = 0u; i < n; i++)
    {
        const float* a_row = &kf->a_mat[(uint32_t) i * n];

        for (uint16_t j = 0u; j < n; j++)
        {
            float acc = 0.0f;

            for (uint16_t k = 0u; k < n; k++)
            {
                acc += a_row[k] * kf->p_mat[(uint32_t) k * n + j];
            }

            kf->scratch_nn[(uint32_t) i * n + j] = acc;
        }
    }

    /* P = scratch * A' + Q. A' is read as a column walk of A rather than being
     * materialised, which is what lets this run without the explicit transpose
     * buffer the legacy version carried. */
    for (uint16_t i = 0u; i < n; i++)
    {
        const float* s_row = &kf->scratch_nn[(uint32_t) i * n];

        for (uint16_t j = 0u; j < n; j++)
        {
            const float* a_row_j = &kf->a_mat[(uint32_t) j * n];
            float        acc     = 0.0f;

            for (uint16_t k = 0u; k < n; k++)
            {
                acc += s_row[k] * a_row_j[k];
            }

            kf->p_mat[(uint32_t) i * n + j] = acc + kf->q_mat[(uint32_t) i * n + j];
        }
    }

    symmetrise(kf);

    if (!state_is_finite(kf))
    {
        rebuild(kf);
        return false;
    }

    kf->predicted = true;
    return true;
}

/**
 * @brief Fold in one scalar measurement using row @p row of H.
 *
 * @param kf     Instance to update.
 * @param row    Row index into H, also the index into R.
 * @param z_val  Measured value for this component.
 * @return true when the component was applied, false when the gate rejected it.
 */
static bool correct_scalar(UTIL_KF_s* kf, uint16_t row, float z_val)
{
    uint16_t     n     = kf->n;
    const float* h_row = &kf->h_mat[(uint32_t) row * n];
    float        r_val = kf->r_diag[row];

    /* ---- v = P h', and denom = h P h' + R ---- */

    float denom = r_val;

    for (uint16_t i = 0u; i < n; i++)
    {
        const float* p_row = &kf->p_mat[(uint32_t) i * n];
        float        acc   = 0.0f;

        for (uint16_t j = 0u; j < n; j++)
        {
            acc += p_row[j] * h_row[j];
        }

        kf->scratch_n[i] = acc;
        denom += h_row[i] * acc;
    }

    /* P is positive semi-definite and R positive, so denom is positive. Floor it
     * anyway: a caller who left R at zero and P at zero for this direction would
     * otherwise divide by zero. */
    if (denom < DENOM_FLOOR)
    {
        denom = DENOM_FLOOR;
    }

    /* ---- innovation ---- */

    float h_x = 0.0f;
    for (uint16_t i = 0u; i < n; i++)
    {
        h_x += h_row[i] * kf->x[i];
    }

    float innovation = z_val - h_x;

    /* Gate on the innovation's own predicted spread: denom IS its variance, so
     * the test needs no extra state. This is what stops a single wild sensor
     * reading from dragging the estimate with it. */
    if (kf->gate_sigma > 0.0f)
    {
        float limit = kf->gate_sigma * UTIL_FastSqrt(denom);

        if (UTIL_Absf(innovation) > limit)
        {
            kf->reject_count++;
            kf->reject_run++;

            /* Reject, unless this is the run-th in a row. A gate that has refused this
             * many consecutive measurements is no longer filtering outliers -- it is
             * disagreeing with the sensor persistently, which means the filter's own
             * state is the likelier thing to be wrong. Letting one through breaks the
             * deadlock; see UTIL_KF_SetGateMaxRun. */
            if (kf->gate_max_run == 0u || kf->reject_run < kf->gate_max_run)
            {
                return false;
            }

            /* Forced through. Reset the run here rather than below so the next
             * rejection starts a fresh count instead of forcing every subsequent
             * measurement once the cap has been reached. */
            kf->reject_run = 0u;
        }
        else
        {
            kf->reject_run = 0u;
        }
    }

    float inv_denom = 1.0f / denom;

    /* ---- k = P h' / denom, then x = x + k * innovation ---- */

    for (uint16_t i = 0u; i < n; i++)
    {
        kf->k_gain[i] = kf->scratch_n[i] * inv_denom;
        kf->x[i] += kf->k_gain[i] * innovation;
    }

    /* ---- Joseph form: P = (I - k h) P (I - k h)' + k R k' ---- */

    /* scratch_nn = (I - k h) P, built row by row. Row i of (I - k h) is
     * e_i - k[i]*h, so row i of the product is P_row_i - k[i] * (h P), and h P
     * is the transpose of P h' which scratch_n already holds — P is symmetric,
     * so no second product is needed. */
    for (uint16_t i = 0u; i < n; i++)
    {
        const float* p_row = &kf->p_mat[(uint32_t) i * n];
        float*       s_row = &kf->scratch_nn[(uint32_t) i * n];
        float        ki    = kf->k_gain[i];

        for (uint16_t j = 0u; j < n; j++)
        {
            s_row[j] = p_row[j] - ki * kf->scratch_n[j];
        }
    }

    /* P = scratch_nn * (I - k h)' + k R k'. Column j of (I - k h)' is
     * e_j - k[j]*h, so element (i,j) is scratch_row_i[j] - k[j]*(scratch_i . h). */
    for (uint16_t i = 0u; i < n; i++)
    {
        const float* s_row = &kf->scratch_nn[(uint32_t) i * n];
        float        s_h   = 0.0f;

        for (uint16_t k = 0u; k < n; k++)
        {
            s_h += s_row[k] * h_row[k];
        }

        float  ki    = kf->k_gain[i];
        float* p_row = &kf->p_mat[(uint32_t) i * n];

        for (uint16_t j = 0u; j < n; j++)
        {
            p_row[j] = s_row[j] - kf->k_gain[j] * s_h + ki * r_val * kf->k_gain[j];
        }
    }

    symmetrise(kf);

    kf->innovation = innovation;
    return true;
}

bool UTIL_KF_Correct(UTIL_KF_s* kf, const float* z)
{
    if (kf == NULL || !kf->initialized || z == NULL)
    {
        return false;
    }

    bool any = false;

    for (uint16_t i = 0u; i < kf->z; i++)
    {
        /* Screen per component and keep going: rejecting the whole vector because
         * one channel is bad would discard the information the others carry. */
        if (!UTIL_IsFinitef(z[i]))
        {
            continue;
        }

        if (correct_scalar(kf, i, z[i]))
        {
            any = true;
        }
    }

    kf->predicted = false;

    if (!state_is_finite(kf))
    {
        rebuild(kf);
        return false;
    }

    return any;
}
