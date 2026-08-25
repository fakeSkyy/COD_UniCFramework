/**
 * @file rtos_hooks.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#include "FreeRTOS.h"
#include "task.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "util_log.h"

/* RTT masks interrupts while it writes to its control block, and the level it masks
 * to must cover everything that can log — which is the same band FreeRTOS defines as
 * "may call a FromISR API". SEGGER's own documentation says to match the two.
 *
 * They are separate literals because SEGGER_RTT_Conf.h is reached from code that
 * cannot include FreeRTOSConfig.h, so this is where the agreement is enforced.
 * Raising configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY without updating the RTT
 * config would otherwise leave a band of interrupts free to log from inside RTT's
 * own critical section, interleaving into a half-written message. */
_Static_assert(SEGGER_RTT_MAX_INTERRUPT_PRIORITY == configMAX_SYSCALL_INTERRUPT_PRIORITY,
               "SEGGER_RTT_MAX_INTERRUPT_PRIORITY must equal "
               "configMAX_SYSCALL_INTERRUPT_PRIORITY");

/* ==========================================================================
 * The callbacks the kernel requires, in one place
 * ==========================================================================
 *
 * FreeRTOS resolves these by name at link time; there is no registration call,
 * so a missing one is an undefined reference and a wrong signature is a silent
 * mismatch. CubeMX used to generate the first of them into freertos.c and
 * nothing at all for the other two, because it left both hooks off.
 *
 * All three report through RTT, which is already linked. That is the whole
 * point of turning the hooks on: each of these conditions is otherwise a board
 * that stops doing anything, with no way to tell which one happened.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Static allocation                                                        */
/* ========================================================================= */

/* configSUPPORT_STATIC_ALLOCATION makes the kernel ask the application where to
 * put the idle task, since it will not allocate one itself. The timer task has
 * an equivalent hook, not needed here because configUSE_TIMERS is 0. */

/** @brief Idle task control block. Never freed; the idle task never exits. */
static StaticTask_t idle_tcb;

/** @brief Idle task stack, in words. */
static StackType_t idle_stack[configMINIMAL_STACK_SIZE];

/**
 * @brief Supply the idle task's storage.
 *
 * The idle task runs whenever nothing else can, and is also what reclaims the
 * stacks of deleted tasks — so it exists even in a build that never deletes
 * one.
 *
 * @param tcb_buffer    Receives the control block.
 * @param stack_buffer  Receives the stack.
 * @param stack_size    Receives the stack size in words, not bytes.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t** tcb_buffer, StackType_t** stack_buffer,
                                   uint32_t* stack_size)
{
    *tcb_buffer   = &idle_tcb;
    *stack_buffer = idle_stack;

    /* In words. configMINIMAL_STACK_SIZE is already a word count, so the array
     * length and this value are the same number — which is exactly the
     * confusion PLAT_Task_Create exists to keep out of the platform layer. */
    *stack_size = configMINIMAL_STACK_SIZE;
}

/* ========================================================================= */
/*  Failure hooks                                                            */
/* ========================================================================= */

/**
 * @brief Called from the context switch when a task has overrun its stack.
 *
 * @par Why this is worth the cycles it costs
 * PLAT_Task_Create takes the stack from its caller, so the size is the one
 * parameter the API cannot check — a stack that is too small does not fail at
 * creation, it quietly overwrites whatever the linker placed after the array.
 * The corruption then surfaces somewhere unrelated, often much later. This hook
 * is what turns that into a named task and a stop.
 *
 * Interrupts are already disabled and the scheduler is mid-switch, so there is
 * no returning from here: the stack that would be returned onto is the damaged
 * one.
 *
 * @param task  The offending task; its TCB is still intact.
 * @param name  Its name, as passed to PLAT_Task_Create.
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char* name)
{
    (void) task;

    UTIL_LOG_E("rtos", "STACK OVERFLOW in task '%s'", (name != NULL) ? name : "?");

    taskDISABLE_INTERRUPTS();

    for (;;)
    {
    }
}

/**
 * @brief Called when pvPortMalloc could not satisfy a request.
 *
 * Every PLAT_*_Create allocates its instance, and they all check for NULL and
 * propagate a failure up to Board_Init — so this is not the only line of
 * defence. What it adds is the reason: without it, a heap 32 bytes too small
 * and a peripheral that genuinely failed to initialise are the same
 * Error_Handler() call.
 *
 * Does not return. Continuing would report a peripheral fault for what is
 * really a configTOTAL_HEAP_SIZE that needs raising.
 */
void vApplicationMallocFailedHook(void)
{
    UTIL_LOG_E("rtos", "heap exhausted — raise configTOTAL_HEAP_SIZE (now %u)",
               (unsigned) configTOTAL_HEAP_SIZE);

    taskDISABLE_INTERRUPTS();

    for (;;)
    {
    }
}

/**
 * @brief Body of configASSERT.
 *
 * Declared in FreeRTOSConfig.h, which cannot include a header of its own — the
 * startup assembly parses that file too.
 *
 * The message goes out before interrupts are disabled: RTT writes into a
 * control block the debugger polls, and with interrupts already off a
 * half-written message would be the last thing ever emitted.
 *
 * @param file  __FILE__ at the failing assertion.
 * @param line  __LINE__ at the failing assertion.
 */
void RTOS_AssertFailed(const char* file, unsigned long line)
{
    UTIL_LOG_E("rtos", "assert failed: %s:%u", file, (unsigned) line);

    taskDISABLE_INTERRUPTS();

    for (;;)
    {
    }
}

/* ========================================================================= */
/*  The kernel tick exception                                                */
/* ========================================================================= */

/* port.c defines this but no public FreeRTOS header exports it — the port expects
 * FreeRTOSConfig.h to have renamed it to SysTick_Handler, which this project
 * deliberately does not do (see the note there). So it is declared by hand. */
void xPortSysTickHandler(void);

/**
 * @brief SysTick exception, forwarded to the kernel once it is running.
 *
 * @par Why this is here and not in stm32h7xx_it.c
 * It used to be there, inside a USER CODE region. The H7 regeneration produced an
 * it.c with no SysTick_Handler at all, which left the startup file's weak
 * Default_Handler — an infinite loop — as the only definition, so the kernel tick
 * would never have arrived and nothing would ever have been scheduled. Defining it
 * on this side of the vendor boundary means a future Generate Code cannot drop it.
 *
 * If CubeMX is ever configured to put the HAL timebase back on SysTick it will
 * generate its own SysTick_Handler and the link will fail with a duplicate
 * definition. That is the desired outcome: the two uses are mutually exclusive and
 * the conflict should be loud.
 *
 * @par Why the scheduler is checked first
 * SysTick only fires once vPortSetupTimerInterrupt has enabled it, which happens
 * inside vTaskStartScheduler — but the peripheral can also be left running by a
 * debugger or a warm reset. xPortSysTickHandler enters a critical section and walks
 * the delayed-task list, neither of which exists before the scheduler starts.
 */
void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}
