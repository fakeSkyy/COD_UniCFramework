/**
 * @file performance_benchmark.c
 * @author COD Framework Team
 * @date 2026/8/24
 * @version 1.0
 */

#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "util_ahrs.h"
#include "util_pid.h"
#include "util_ringbuf.h"

/* ========================================================================= */
/*  Benchmark configuration                                                  */
/* ========================================================================= */

#define WARMUP_BATCHES 3u
#define MEASURE_BATCHES 9u
#define TARGET_BATCH_NS UINT64_C(12000000)
#define MIN_BATCH_NS UINT64_C(6000000)
#define MAX_ITERATIONS UINT64_C(1073741824)

#if defined(__GNUC__) || defined(__clang__)
#define PERF_NOINLINE __attribute__((noinline))
#else
#define PERF_NOINLINE
#endif

typedef void (*Workload_Fn)(void* context, uint64_t iterations);

typedef struct
{
    const char* name;
    uint64_t    iterations;
    double      median_ns;
    double      minimum_ns;
    double      maximum_ns;
    double      spread_ratio;
} Benchmark_Result_s;

typedef struct
{
    uint32_t state;
} Calibration_Context_s;

typedef struct
{
    UTIL_RingBuf_s ring;
    uint8_t        storage[256];
    uint32_t       sequence;
} RingBuf_Context_s;

typedef struct
{
    UTIL_PID_s pid;
    uint32_t   phase;
} PID_Context_s;

typedef struct
{
    UTIL_AHRS_s ahrs;
    float       storage[UTIL_AHRS_BUF_SIZE];
    uint32_t    phase;
} AHRS_Context_s;

static volatile uint64_t benchmark_sink_u64;
static volatile float    benchmark_sink_float;

/* ========================================================================= */
/*  Timing helpers                                                           */
/* ========================================================================= */

/**
 * @brief Read a monotonic host clock in nanoseconds.
 * @return Current timestamp, or zero when the clock read failed.
 */
static uint64_t monotonic_ns(void)
{
    struct timespec stamp;

    if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0)
    {
        return 0u;
    }
    return (uint64_t) stamp.tv_sec * UINT64_C(1000000000) + (uint64_t) stamp.tv_nsec;
}

/**
 * @brief Compare doubles for qsort.
 * @param lhs Left value.
 * @param rhs Right value.
 * @return Ordering result.
 */
static int compare_double(const void* lhs, const void* rhs)
{
    double left  = *(const double*) lhs;
    double right = *(const double*) rhs;

    return (left > right) - (left < right);
}

/**
 * @brief Time one workload batch on the same monotonic clock.
 * @param workload Workload callback.
 * @param context Workload state.
 * @param iterations Logical operation count.
 * @return Elapsed nanoseconds, or zero on failure.
 */
static uint64_t time_batch(Workload_Fn workload, void* context, uint64_t iterations)
{
    uint64_t start = monotonic_ns();
    workload(context, iterations);
    uint64_t end = monotonic_ns();

    return (end > start) ? (end - start) : 0u;
}

/**
 * @brief Auto-size a workload to a stable millisecond-scale batch.
 *
 * Independent sizing prevents the expensive AHRS path from inheriting the
 * iteration count suitable for the tiny RingBuf path.
 *
 * @param workload Workload callback.
 * @param context Workload state.
 * @return Iteration count, or zero on clock failure.
 */
static uint64_t calibrate_iterations(Workload_Fn workload, void* context)
{
    uint64_t iterations = 1024u;
    uint64_t elapsed    = 0u;

    while (iterations <= MAX_ITERATIONS)
    {
        elapsed = time_batch(workload, context, iterations);
        if (elapsed == 0u)
        {
            return 0u;
        }
        if (elapsed >= MIN_BATCH_NS)
        {
            break;
        }
        iterations *= 2u;
    }

    if (iterations > MAX_ITERATIONS)
    {
        iterations = MAX_ITERATIONS;
    }
    if (elapsed > 0u)
    {
        long double scaled =
            (long double) iterations * (long double) TARGET_BATCH_NS / (long double) elapsed;
        if (scaled >= 1024.0L && scaled <= (long double) MAX_ITERATIONS)
        {
            iterations = (uint64_t) scaled;
        }
    }
    return (iterations == 0u) ? 1u : iterations;
}

/**
 * @brief Warm, sample, and reduce one workload to robust statistics.
 *
 * Warmup is excluded and the median drives the ratio gate. Min/max are retained
 * so host scheduling noise remains visible.
 *
 * @param name Stable result name.
 * @param workload Workload callback.
 * @param context Workload state.
 * @param result Statistics destination.
 * @return true when all clock reads succeeded.
 */
static bool measure(const char* name, Workload_Fn workload, void* context,
                    Benchmark_Result_s* result)
{
    double   samples[MEASURE_BATCHES];
    uint64_t iterations = calibrate_iterations(workload, context);

    if (iterations == 0u)
    {
        return false;
    }
    for (uint32_t batch = 0u; batch < WARMUP_BATCHES; batch++)
    {
        if (time_batch(workload, context, iterations) == 0u)
        {
            return false;
        }
    }
    for (uint32_t batch = 0u; batch < MEASURE_BATCHES; batch++)
    {
        uint64_t elapsed = time_batch(workload, context, iterations);
        if (elapsed == 0u)
        {
            return false;
        }
        samples[batch] = (double) elapsed / (double) iterations;
    }

    qsort(samples, MEASURE_BATCHES, sizeof(samples[0]), compare_double);
    result->name       = name;
    result->iterations = iterations;
    result->minimum_ns = samples[0];
    result->median_ns  = samples[MEASURE_BATCHES / 2u];
    result->maximum_ns = samples[MEASURE_BATCHES - 1u];

    double deviations[MEASURE_BATCHES];
    for (uint32_t batch = 0u; batch < MEASURE_BATCHES; batch++)
    {
        double delta      = samples[batch] - result->median_ns;
        deviations[batch] = (delta < 0.0) ? -delta : delta;
    }
    qsort(deviations, MEASURE_BATCHES, sizeof(deviations[0]), compare_double);
    result->spread_ratio = deviations[MEASURE_BATCHES / 2u] / result->median_ns;
    return true;
}

/* ========================================================================= */
/*  Workloads                                                                */
/* ========================================================================= */

/**
 * @brief Run the serial xorshift calibration workload.
 *
 * The recurrence prevents vectorization and closed-form folding. Publishing the
 * final state makes the loop observable without volatile traffic per operation.
 *
 * @param context Calibration state.
 * @param iterations Xorshift operation count.
 */
static PERF_NOINLINE void calibration_workload(void* context, uint64_t iterations)
{
    Calibration_Context_s* calibration = (Calibration_Context_s*) context;
    uint32_t               state       = calibration->state;

    for (uint64_t i = 0u; i < iterations; i++)
    {
        state ^= state << 13u;
        state ^= state >> 17u;
        state ^= state << 5u;
    }
    calibration->state = state;
    benchmark_sink_u64 ^= state;
}

/**
 * @brief Exercise one successful RingBuf enqueue/dequeue pair per operation.
 * @param context RingBuf state.
 * @param iterations Pair count.
 */
static PERF_NOINLINE void ringbuf_workload(void* context, uint64_t iterations)
{
    RingBuf_Context_s* ring_context = (RingBuf_Context_s*) context;
    uint32_t           checksum     = 0u;

    for (uint64_t i = 0u; i < iterations; i++)
    {
        uint8_t input  = (uint8_t) ring_context->sequence++;
        uint8_t output = 0u;
        bool    put_ok = UTIL_RingBuf_Put(&ring_context->ring, input);
        bool    get_ok = UTIL_RingBuf_Get(&ring_context->ring, &output);

        checksum += (uint32_t) output + (put_ok ? 1u : 0u) + (get_ok ? 2u : 0u);
    }
    benchmark_sink_u64 ^= checksum;
}

/**
 * @brief Exercise the fully configured PID hot path.
 * @param context PID state.
 * @param iterations Controller step count.
 */
static PERF_NOINLINE void pid_workload(void* context, uint64_t iterations)
{
    PID_Context_s* pid_context = (PID_Context_s*) context;
    float          checksum    = 0.0f;

    for (uint64_t i = 0u; i < iterations; i++)
    {
        uint32_t phase  = pid_context->phase++;
        float    target = (float) (int32_t) (phase & 255u) * 0.01f;
        float    meas   = (float) (int32_t) ((phase * 17u) & 127u) * 0.008f;

        checksum += UTIL_PID_Step(&pid_context->pid, target, meas, 0.001f);
    }
    benchmark_sink_float += checksum;
}

/**
 * @brief Exercise complete AHRS propagation and Kalman correction.
 * @param context AHRS state.
 * @param iterations Estimator update count.
 */
static PERF_NOINLINE void ahrs_workload(void* context, uint64_t iterations)
{
    AHRS_Context_s* ahrs_context = (AHRS_Context_s*) context;
    float           checksum     = 0.0f;

    for (uint64_t i = 0u; i < iterations; i++)
    {
        uint32_t phase = ahrs_context->phase++;
        float gyro[3]  = {(float) (phase & 7u) * 0.0002f, (float) ((phase >> 3u) & 7u) * -0.00015f,
                          0.01f};
        float accel[3] = {(float) (int32_t) (phase & 3u) * 0.0005f, 0.0f, 9.794f};

        if (UTIL_AHRS_Update(&ahrs_context->ahrs, gyro, accel, 0.001f))
        {
            checksum += UTIL_AHRS_GetRoll(&ahrs_context->ahrs) +
                        UTIL_AHRS_GetPitch(&ahrs_context->ahrs) +
                        UTIL_AHRS_GetYaw(&ahrs_context->ahrs);
        }
    }
    benchmark_sink_float += checksum;
}

/* ========================================================================= */
/*  Setup and output                                                         */
/* ========================================================================= */

/**
 * @brief Initialize all three real production workload contexts.
 * @param ring_context RingBuf context.
 * @param pid_context PID context.
 * @param ahrs_context AHRS context.
 * @return true on complete initialization.
 */
static bool initialize_contexts(RingBuf_Context_s* ring_context, PID_Context_s* pid_context,
                                AHRS_Context_s* ahrs_context)
{
    UTIL_RingBuf_Init(&ring_context->ring, ring_context->storage,
                      (uint16_t) sizeof(ring_context->storage));
    ring_context->sequence = 1u;

    UTIL_PID_Cfg_s pid_config            = {0};
    pid_config.kp                        = 2.0f;
    pid_config.ki                        = 0.4f;
    pid_config.kd                        = 0.02f;
    pid_config.deadband                  = 0.001f;
    pid_config.limit_integral            = 5.0f;
    pid_config.limit_output              = 10.0f;
    pid_config.limit_slew                = 100.0f;
    pid_config.dt_min                    = 0.0005f;
    pid_config.dt_max                    = 0.002f;
    pid_config.derivative_fc_hz          = 30.0f;
    pid_config.output_fc_hz              = 80.0f;
    pid_config.integral_fade_start       = 0.5f;
    pid_config.integral_fade_band        = 1.0f;
    pid_config.derivative_on_measurement = true;
    pid_config.trapezoid_integral        = true;
    pid_context->phase                   = 0u;

    if (!UTIL_PID_Init(&pid_context->pid, &pid_config, UTIL_PID_POSITION))
    {
        return false;
    }

    ahrs_context->phase = 0u;
    if (!UTIL_AHRS_Init(&ahrs_context->ahrs, ahrs_context->storage, 9.794f))
    {
        return false;
    }
    UTIL_AHRS_SetNoise(&ahrs_context->ahrs, 10.0f, 0.001f, 1.0f);

    const float level[3] = {0.0f, 0.0f, 9.794f};
    return UTIL_AHRS_AlignToAccel(&ahrs_context->ahrs, level);
}

/**
 * @brief Emit one benchmark result as JSON.
 * @param result Result to emit.
 * @param calibration Same-process calibration result.
 * @param trailing Whether a comma follows the object.
 */
static void print_result(const Benchmark_Result_s* result, const Benchmark_Result_s* calibration,
                         bool trailing)
{
    printf("    \"%s\": {\"iterations\": %" PRIu64
           ", \"median_ns_per_op\": %.6f, \"min_ns_per_op\": %.6f, "
           "\"max_ns_per_op\": %.6f, \"spread_ratio\": %.6f, \"ratio\": %.6f}%s\n",
           result->name, result->iterations, result->median_ns, result->minimum_ns,
           result->maximum_ns, result->spread_ratio, result->median_ns / calibration->median_ns,
           trailing ? "," : "");
}

int main(void)
{
    Calibration_Context_s calibration_context = {.state = 0x9E3779B9u};
    RingBuf_Context_s     ring_context        = {0};
    PID_Context_s         pid_context         = {0};
    AHRS_Context_s        ahrs_context        = {0};
    Benchmark_Result_s    calibration_result;
    Benchmark_Result_s    ring_result;
    Benchmark_Result_s    pid_result;
    Benchmark_Result_s    ahrs_result;

    if (!initialize_contexts(&ring_context, &pid_context, &ahrs_context))
    {
        fputs("production workload initialization failed\n", stderr);
        return 2;
    }
    if (!measure("calibration", calibration_workload, &calibration_context, &calibration_result) ||
        !measure("ringbuf_put_get", ringbuf_workload, &ring_context, &ring_result) ||
        !measure("pid_step", pid_workload, &pid_context, &pid_result) ||
        !measure("ahrs_update", ahrs_workload, &ahrs_context, &ahrs_result))
    {
        fputs("clock_gettime failed or returned a non-increasing timestamp\n", stderr);
        return 3;
    }

    printf("{\n  \"clock\": \"CLOCK_MONOTONIC\",\n  \"warmup_batches\": %u,\n"
           "  \"measurement_batches\": %u,\n  \"calibration\": {\"iterations\": %" PRIu64
           ", \"median_ns_per_op\": %.6f, \"min_ns_per_op\": %.6f, "
           "\"max_ns_per_op\": %.6f, \"spread_ratio\": %.6f},\n"
           "  \"benchmarks\": {\n",
           WARMUP_BATCHES, MEASURE_BATCHES, calibration_result.iterations,
           calibration_result.median_ns, calibration_result.minimum_ns,
           calibration_result.maximum_ns, calibration_result.spread_ratio);
    print_result(&ring_result, &calibration_result, true);
    print_result(&pid_result, &calibration_result, true);
    print_result(&ahrs_result, &calibration_result, false);
    printf("  },\n  \"sink\": {\"integer\": %" PRIu64 ", \"float\": %.6f}\n}\n", benchmark_sink_u64,
           (double) benchmark_sink_float);
    return 0;
}
