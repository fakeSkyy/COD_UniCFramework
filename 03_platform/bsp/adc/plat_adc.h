/**
 * @file plat_adc.h
 * @author Gao Xing
 * @date 2025/7/21
 * @version 1.0
 */

#ifndef PLAT_ADC_H
#define PLAT_ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_dma_buf.h"

#include "impl_adc.h"

typedef struct ADC_Instance_s ADC_Instance_s;

/**
 * @brief User callback invoked when an interrupt-driven conversion completes.
 * @param adc    The ADC instance that produced the sample.
 * @param value  Raw ADC code from the completed conversion.
 */
typedef void (*PLAT_ADC_Callback)(ADC_Instance_s* adc, uint32_t value);

/**
 * @brief User callback invoked on a DMA buffer event (half / full).
 * @param adc  The ADC instance.
 * @param buf  DMA destination buffer.
 * @param len  Buffer length in samples.
 * @note On circular DMA the "half" callback signals buf[0 .. len/2) is ready,
 *       and the "full" callback signals buf[len/2 .. len) is ready.
 */
typedef void (*PLAT_ADC_BufCallback)(ADC_Instance_s* adc, const uint16_t* buf, uint32_t len);

/**
 * @brief A vendor-neutral ADC input handle.
 *
 * Carries only an ops vtable and an opaque @c ctx produced by some backend.
 * This layer never dereferences @c ctx, so it stays fully decoupled from any
 * chip vendor: one backend may store a (peripheral handle, channel) pair inside
 * @c ctx, while another with no such handle stores a bare channel index.
 */
struct ADC_Instance_s
{
    const ADC_Ops_s*     ops;     /**< Backend vtable (from *_GetOps).        */
    void*                ctx;     /**< Opaque, backend-owned input descriptor.*/
    PLAT_ADC_Callback    user_cb; /**< Optional single-conversion callback.   */
    PLAT_ADC_BufCallback full_cb; /**< Optional DMA full-buffer callback.     */
    PLAT_ADC_BufCallback half_cb; /**< Optional DMA half-buffer callback.     */
    void*                id;      /**< Optional owner tag for registry use.   */
};

/**
 * @brief Perform one blocking conversion.
 *
 * @par Why the code comes back through a pointer
 * There is no reserved value that could mean "failed": a converter's full-scale
 * code is a legitimate reading, and which code that is depends on the backend's
 * resolution. Returning status separately is correct at any width — see the
 * read() contract in the ops vtable.
 *
 * @param adc      ADC instance.
 * @param timeout  Backend-defined timeout budget (typically milliseconds).
 * @param out      Receives the raw code on success; untouched on failure.
 * @return true on a completed conversion; false on a NULL argument, timeout, or
 *         hardware error.
 */
bool PLAT_ADC_Read(ADC_Instance_s* adc, uint32_t timeout, uint16_t* out);

/**
 * @brief Read one sample and convert it to millivolts.
 *
 * Pure, vendor-neutral scaling: mV = raw * vref_mv / (2^resolution_bits - 1).
 *
 * @param adc              ADC instance.
 * @param timeout          Backend-defined timeout budget.
 * @param vref_mv          Reference voltage in millivolts (e.g. 3300).
 * @param resolution_bits  Converter resolution in bits, 1..16. Must match how the
 *                         backend's converter is configured; a mismatch scales
 *                         every reading by a constant factor and looks like a
 *                         reference-voltage error.
 * @param out_mv           Receives the voltage in millivolts; untouched on failure.
 * @return true on success; false on a NULL argument, an out-of-range
 *         @p resolution_bits, or a failed conversion.
 */
bool PLAT_ADC_ReadVoltage(ADC_Instance_s* adc, uint32_t timeout, uint32_t vref_mv,
                          uint8_t resolution_bits, uint32_t* out_mv);

/**
 * @brief Register a completion callback for interrupt-driven conversions.
 *
 * The callback runs in interrupt context; keep it short. Call before
 * PLAT_ADC_StartIT.
 *
 * @param adc  ADC instance.
 * @param cb   Callback to invoke on each completed conversion (NULL to clear).
 */
void PLAT_ADC_OnComplete(ADC_Instance_s* adc, PLAT_ADC_Callback cb);

/**
 * @brief Start interrupt-driven conversion on this input.
 * @param adc  ADC instance.
 * @return true on success, false on failure.
 */
bool PLAT_ADC_StartIT(ADC_Instance_s* adc);

/**
 * @brief Stop interrupt-driven conversion on this input.
 * @param adc  ADC instance.
 */
void PLAT_ADC_StopIT(ADC_Instance_s* adc);

/**
 * @brief Register DMA buffer-event callbacks (run in interrupt context).
 *
 * Call before PLAT_ADC_StartDMA. For a single-channel streaming setup, use
 * both callbacks for ping-pong processing; either may be NULL.
 *
 * @param adc   ADC instance.
 * @param full  Called when the (second half of the) buffer is filled.
 * @param half  Called when the first half of the buffer is filled.
 */
void PLAT_ADC_OnBuffer(ADC_Instance_s* adc, PLAT_ADC_BufCallback full, PLAT_ADC_BufCallback half);

/**
 * @brief Start DMA transfer into a caller-owned buffer.
 *
 * The buffer must remain valid until PLAT_ADC_StopDMA. The backend typically
 * runs circular, so @p half / @p full callbacks fire repeatedly.
 *
 * @param adc  ADC instance.
 * @param buf  Caller-owned destination buffer (static storage recommended).
 * @param len  Buffer length in samples.
 * @return true on success, false on failure.
 */
bool PLAT_ADC_StartDMA(ADC_Instance_s* adc, uint16_t* buf, uint32_t len);

/**
 * @brief Stop the DMA transfer on this input.
 * @param adc  ADC instance.
 */
void PLAT_ADC_StopDMA(ADC_Instance_s* adc);

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
 * @brief Initialize a ADC instance over caller-provided storage.
 *
 * The counterpart of PLAT_ADC_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_ADC_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_ADC_Init(ADC_Instance_s* inst, const ADC_Ops_s* ops, void* ctx);

/**
 * @brief Create an ADC instance from a backend-provided ops and context.
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one analog input.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
ADC_Instance_s* PLAT_ADC_Create(const ADC_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_ADC_H */
