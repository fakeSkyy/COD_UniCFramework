/**
 * @file rtos_fault.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include <stdint.h>

#include "stm32h7xx.h"

#include "SEGGER_RTT.h"

#include "rtos_fault.h"

/* ==========================================================================
 * Turning a hang into a diagnosis
 * ==========================================================================
 *
 * CubeMX generates the four fault handlers as bare `while (1)`, so a fault
 * reports nothing at all: the debugger shows a PC somewhere in the handler and
 * every register that mattered has already been replaced. What the core actually
 * preserved — the faulting PC, the fault-status bits, the offending address — is
 * sitting in memory and in SCB, unread.
 *
 * That is what this file is for. Each handler is `naked` so no prologue disturbs
 * the stack, passes the stack frame the core pushed to one common reporter, and
 * that reporter decodes SCB and prints over RTT before stopping.
 *
 * @par Why the handlers live here and not in stm32h7xx_it.c
 * They used to be there. stm32h7xx_it.c belongs to CubeMX, and its versions are
 * plain `while (1)` bodies outside any USER CODE region — so a Generate Code
 * would restore the silent ones. Defining them in framework code makes the
 * duplicate-definition a link error instead: loud, and pointing at the right
 * file. The CubeMX copies must therefore be deleted, which is a one-time edit
 * recorded in this project's porting notes.
 *
 * @par Why RTT rather than a UART
 * A fault can happen with interrupts masked and with the UART's DMA half
 * configured. RTT is a memory write plus a debugger poll, needs no peripheral to
 * be working, and is already this firmware's log path.
 *
 * @par Why raw SEGGER_RTT_printf here rather than util_log
 * Everything else in the framework logs through UTIL_LOG_*, and this file
 * deliberately does not. A fault reporter runs after something has already gone
 * wrong, on a stack that may be nearly exhausted and possibly with the fault
 * being a corrupted call through a function pointer. Every layer it goes through
 * is another chance to fault while reporting a fault, which produces a silent
 * lockup instead of a diagnosis — the exact failure this file exists to prevent.
 *
 * util_log would also add a runtime level check able to suppress this output, and
 * a prefix per line where what is wanted is one labelled block. So the trade is
 * made the other way here: fewer frames, no filtering, no shared state.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Stack frame the core pushes on exception entry                           */
/* ========================================================================= */

/**
 * @brief The eight words Cortex-M pushes before entering a fault handler.
 *
 * Order is fixed by the architecture. @c pc is the instruction that faulted,
 * which is the single most useful value here — everything else is context for
 * it.
 */
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr; /**< Return address of the function that faulted. */
    uint32_t pc; /**< The faulting instruction. */
    uint32_t psr;
} Fault_Frame_s;

/* ========================================================================= */
/*  Reporter                                                                 */
/* ========================================================================= */

/**
 * @brief Decode SCB, print, and stop.
 *
 * @param frame  Stacked frame, from whichever stack was active.
 * @param name   Which fault fired.
 */
static void report(const Fault_Frame_s* frame, const char* name)
{
    const uint32_t cfsr = SCB->CFSR;
    const uint32_t hfsr = SCB->HFSR;

    SEGGER_RTT_printf(0, "\r\n*** %s ***\r\n", name);

    if (frame != NULL)
    {
        SEGGER_RTT_printf(0, "  pc  0x%08X   lr  0x%08X   psr 0x%08X\r\n", frame->pc, frame->lr,
                          frame->psr);
        SEGGER_RTT_printf(0, "  r0  0x%08X   r1  0x%08X   r2  0x%08X   r3 0x%08X\r\n", frame->r0,
                          frame->r1, frame->r2, frame->r3);
    }

    SEGGER_RTT_printf(0, "  CFSR 0x%08X  HFSR 0x%08X\r\n", cfsr, hfsr);

    /* FORCED means an escalated fault: a configurable fault fired while its own
     * handler was masked or disabled, so CFSR below is what actually went wrong
     * and this HardFault is only the messenger. That is the case for almost every
     * HardFault in a default configuration, because MemManage/BusFault/UsageFault
     * are not enabled unless something enables them. */
    if (hfsr & SCB_HFSR_FORCED_Msk)
    {
        SEGGER_RTT_printf(0, "  escalated from a configurable fault\r\n");
    }

    if (hfsr & SCB_HFSR_VECTTBL_Msk)
    {
        SEGGER_RTT_printf(0, "  VECTTBL: bad vector fetch — VTOR or the vector table is wrong\r\n");
    }

    /* Usage faults. INVSTATE is the one that catches a corrupted function
     * pointer: it means the target address had its Thumb bit clear, which is what
     * a jump into data looks like. */
    if (cfsr & SCB_CFSR_UNDEFINSTR_Msk)
    {
        SEGGER_RTT_printf(0, "  UNDEFINSTR: not an instruction — executing data?\r\n");
    }
    if (cfsr & SCB_CFSR_INVSTATE_Msk)
    {
        SEGGER_RTT_printf(0, "  INVSTATE: Thumb bit clear — bad function pointer\r\n");
    }
    if (cfsr & SCB_CFSR_INVPC_Msk)
    {
        SEGGER_RTT_printf(0, "  INVPC: bad EXC_RETURN — corrupted exception stack\r\n");
    }
    if (cfsr & SCB_CFSR_UNALIGNED_Msk)
    {
        SEGGER_RTT_printf(0, "  UNALIGNED access\r\n");
    }
    if (cfsr & SCB_CFSR_DIVBYZERO_Msk)
    {
        SEGGER_RTT_printf(0, "  DIVBYZERO\r\n");
    }

    /* Bus faults. PRECISERR is the useful one — BFAR holds the address. */
    if (cfsr & SCB_CFSR_PRECISERR_Msk)
    {
        SEGGER_RTT_printf(0, "  PRECISERR at 0x%08X\r\n", SCB->BFAR);
    }
    if (cfsr & SCB_CFSR_IMPRECISERR_Msk)
    {
        /* Asynchronous, so BFAR is meaningless and the reported PC has already
         * moved past the store that caused it. On H7 this is most often a write
         * to a peripheral whose clock is off, or a DMA/cache interaction. */
        SEGGER_RTT_printf(0, "  IMPRECISERR: async bus error, PC and BFAR unreliable\r\n");
    }
    if (cfsr & SCB_CFSR_IBUSERR_Msk)
    {
        SEGGER_RTT_printf(0, "  IBUSERR: instruction fetch failed\r\n");
    }

    /* Memory-management faults. Reachable here even with no MPU regions of our
     * own: the H7 default map makes some addresses simply illegal. */
    if (cfsr & SCB_CFSR_DACCVIOL_Msk)
    {
        SEGGER_RTT_printf(0, "  DACCVIOL: data access denied at 0x%08X\r\n", SCB->MMFAR);
    }
    if (cfsr & SCB_CFSR_IACCVIOL_Msk)
    {
        SEGGER_RTT_printf(0, "  IACCVIOL: instruction access denied\r\n");
    }
    if (cfsr & SCB_CFSR_MSTKERR_Msk)
    {
        SEGGER_RTT_printf(0, "  MSTKERR: stacking failed — stack overflow\r\n");
    }
    if (cfsr & SCB_CFSR_MUNSTKERR_Msk)
    {
        SEGGER_RTT_printf(0, "  MUNSTKERR: unstacking failed\r\n");
    }

    /* Where the faulting code was running. A PSP outside a task's stack array,
     * or an MSP that has run past _estack, is a stack overflow regardless of what
     * CFSR says. */
    SEGGER_RTT_printf(0, "  msp 0x%08X  psp 0x%08X\r\n", __get_MSP(), __get_PSP());

    __disable_irq();

    for (;;)
    {
    }
}

/* ========================================================================= */
/*  Handlers                                                                 */
/* ========================================================================= */

/* Each handler is naked so that nothing runs before the stack pointer is read: a
 * compiler-generated prologue would push registers over the very frame being
 * recovered. The body is a tail call passing LR, which at handler entry holds
 * EXC_RETURN.
 *
 * The fault kind is passed as a small integer rather than a string pointer: a
 * literal's address cannot be formed by a bare `mov` on this architecture, and
 * building it through a literal pool from inside naked asm is exactly the kind of
 * fragility a fault handler must not have. */

/** @brief Which fault fired; index into fault_names. */
enum
{
    FAULT_HARD = 0,
    FAULT_MEMMANAGE,
    FAULT_BUS,
    FAULT_USAGE,
};

static const char* const fault_names[] = {
    "HardFault",
    "MemManage fault",
    "BusFault",
    "UsageFault",
};

/**
 * @brief Pick the stack the frame was pushed on, then report.
 *
 * Not static: the naked handlers below branch to it by name from asm, and a
 * static symbol could be inlined or renamed out from under them.
 *
 * @param exc_return  EXC_RETURN, whose bit 2 says which stack was in use.
 * @param kind        Index into fault_names.
 */
void rtos_fault_dispatch(uint32_t exc_return, uint32_t kind);

void rtos_fault_dispatch(uint32_t exc_return, uint32_t kind)
{
    /* Bit 2 of EXC_RETURN: set means the frame is on the process stack, clear
     * means the main stack. Reading the wrong one prints eight words of garbage,
     * which is worse than printing none. */
    const Fault_Frame_s* frame = (exc_return & 0x4u) ? (const Fault_Frame_s*) __get_PSP()
                                                     : (const Fault_Frame_s*) __get_MSP();

    const char* name =
        (kind < (sizeof fault_names / sizeof fault_names[0])) ? fault_names[kind] : "unknown fault";

    report(frame, name);
}

#if defined(__arm__) || defined(__thumb__)
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile("mov r0, lr           \n"
                   "movs r1, #0          \n"
                   "b   rtos_fault_dispatch\n");
}

__attribute__((naked)) void MemManage_Handler(void)
{
    __asm volatile("mov r0, lr           \n"
                   "movs r1, #1          \n"
                   "b   rtos_fault_dispatch\n");
}

__attribute__((naked)) void BusFault_Handler(void)
{
    __asm volatile("mov r0, lr           \n"
                   "movs r1, #2          \n"
                   "b   rtos_fault_dispatch\n");
}

__attribute__((naked)) void UsageFault_Handler(void)
{
    __asm volatile("mov r0, lr           \n"
                   "movs r1, #3          \n"
                   "b   rtos_fault_dispatch\n");
}
#endif

/* ========================================================================= */
/*  Enabling the handlers above                                              */
/* ========================================================================= */

void RTOS_FaultInit(void)
{
    /* Without these three bits the handlers above are unreachable: the core leaves
     * the configurable faults disabled at reset and escalates every one of them to
     * HardFault instead. The report is still produced — HardFault decodes CFSR
     * regardless, and prints FORCED to say the fault was escalated — but the frame
     * it recovers can belong to the escalation rather than to the access that
     * caused it, and that frame's pc is the whole value of the report.
     *
     * Enabling them costs one store and changes nothing when no fault occurs. */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;

    /* Trap integer division by zero rather than defining it as 0.
     *
     * Off at reset, and the default is the dangerous one for control code: SDIV/UDIV
     * by zero yields 0 and execution continues, so a divide by a stale or
     * uninitialised divisor produces a plausible-looking number instead of stopping.
     *
     * UNALIGN_TRP is deliberately left off. Unaligned access is legal on this core
     * for ordinary loads and stores, the compiler emits it, and enabling the trap
     * would fault on correct code — including inside the vendor libraries. */
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

    /* The stores above target Device memory, which is not reordered against
     * subsequent accesses on this core — but a fault taken on the very next
     * instruction should already see the new configuration, and these barriers are
     * what the architecture reference asks for after writing SHCSR or CCR. This runs
     * once, so their cost does not matter. */
    __DSB();
    __ISB();
}
