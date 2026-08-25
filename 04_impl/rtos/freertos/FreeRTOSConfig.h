/**
 * @file FreeRTOSConfig.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ==========================================================================
 * Why this file lives here and not in 05_vender/Core/Inc
 * ==========================================================================
 *
 * CubeMX used to own this file. Every value in it came from the .ioc, and the
 * only editable regions were the USER CODE markers — so opening the RTOS panel
 * and pressing Generate Code silently reverted anything set by hand. That is
 * the same mechanism that destroyed the root Makefile once already.
 *
 * FreeRTOS is now disabled in the .ioc, so CubeMX neither generates nor
 * consults this file. It is a framework-owned header like any other, and the
 * values below are chosen for this project rather than transcribed from a GUI.
 *
 * The kernel is upstream FreeRTOS V11.3.0 from FreeRTOS/FreeRTOS-Kernel, not
 * the V10.3.1 copy CubeMX used to vendor. See 07_rtos/freertos/kernel/VERSION.
 *
 * Two settings in particular are the reason this move was worth making: they
 * were off under CubeMX (as FreeRTOS defaults, not as deliberate choices) and
 * could not be turned on durably.
 *
 *   configCHECK_FOR_STACK_OVERFLOW   PLAT_Task_Create takes the stack from its
 *                                    caller, so the stack size is the one thing
 *                                    the API cannot verify. Without this a stack
 *                                    that is too small corrupts whatever sits
 *                                    next to it, with no symptom at the point
 *                                    of failure.
 *
 *   configUSE_MALLOC_FAILED_HOOK     Every PLAT_*_Create allocates. The callers
 *                                    all check for NULL, but a hook turns a
 *                                    heap that is merely too small into a named
 *                                    stop rather than a Board_Init that fails
 *                                    for no stated reason.
 *
 * Both hooks are implemented in 07_rtos/rtos_hooks.c.
 * ==========================================================================
 */

/* Assembler-safe section. The startup file and port assembly include this
 * header, and neither can parse C declarations. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif

/* ========================================================================= */
/*  Scheduler                                                                */
/* ========================================================================= */

#define configUSE_PREEMPTION 1

/* Single core. V11 added SMP support, and several of its asserts and the
 * configUSE_PASSIVE_IDLE_HOOK requirement only apply above 1 — stating it
 * explicitly documents that this is a choice rather than a default. */
#define configNUMBER_OF_CORES 1

/* The port's CLZ-based ready-list scan, valid because configMAX_PRIORITIES is
 * at most 32. Cheaper and constant-time versus the generic loop. */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#define configCPU_CLOCK_HZ (SystemCoreClock)

/* 1 kHz. Chosen to match the control loops, so a period expressed in
 * milliseconds converts to ticks exactly and pdMS_TO_TICKS never rounds. */
#define configTICK_RATE_HZ ((TickType_t) 1000)

/* Priorities 0..6. PLAT_Task_Create clamps rather than asserting, so raising
 * this later cannot break an existing caller. */
#define configMAX_PRIORITIES (7)

/* In words, not bytes: 256 words = 1 KB. This is the floor PLAT_Task_Create
 * enforces on every task, and the size of the idle task's stack. */
#define configMINIMAL_STACK_SIZE ((uint16_t) 256)

/* V11 renamed the tick type control: configUSE_16_BIT_TICKS is deprecated in
 * favour of an explicit width. 32-bit, so the tick counter wraps after 49 days
 * at 1 kHz rather than 65 seconds — PLAT_Task_DelayUntil does modular
 * arithmetic that is wrap-correct either way, but a 16-bit tick would make
 * every timeout above 65 s unrepresentable. */
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS

#define configIDLE_SHOULD_YIELD 1

/* ========================================================================= */
/*  Memory                                                                   */
/* ========================================================================= */

/* Static allocation is what the framework actually uses: PLAT_Task_Create and
 * PLAT_Mutex_Init both hand the kernel caller-owned storage, so no task or
 * mutex can fail to be created for lack of heap. */
#define configSUPPORT_STATIC_ALLOCATION 1

/* Dynamic allocation stays on because the platform layer's own Create
 * functions (PLAT_SPI_Create and friends) call PLAT_malloc, which is
 * pvPortMalloc. Turning it off would take those with it. */
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/* 24 KB out of the 128 KB of main SRAM. Consumed by the platform-layer
 * instances built during Board_Init, not by tasks — those are static. */
#define configTOTAL_HEAP_SIZE ((size_t) 24576)

#define configAPPLICATION_ALLOCATED_HEAP 0

/* ========================================================================= */
/*  Features in use                                                          */
/* ========================================================================= */

/* Priority inheritance, which is the reason to use a mutex rather than a
 * binary semaphore: a low-priority holder is lifted to the waiter's priority
 * instead of being preempted by everything in between. */
#define configUSE_MUTEXES 1

/* Task notifications are the wake mechanism behind PLAT_Task_Notify and
 * PLAT_Task_Wait. Lighter than a semaphore — the state lives in the TCB, so
 * there is no separate object to allocate. */
#define configUSE_TASK_NOTIFICATIONS 1

/* Counting and binary semaphores, behind PLAT_Sem_*. The reason to have them
 * alongside task notifications is that a notification targets one specific task,
 * so it cannot serve several waiters on one event, and it carries no count — "N
 * buffers free" is not expressible. Both live in queue.c, which is already built
 * for the mutex, so this costs no new source file. */
#define configUSE_COUNTING_SEMAPHORES 1

/* Recursive mutexes, behind PLAT_Mutex_InitRecursive. A distinct queue type with
 * its own take/give, not a flag on the plain mutex. */
#define configUSE_RECURSIVE_MUTEXES 1

/* ========================================================================= */
/*  Features deliberately off                                                */
/* ========================================================================= */
/*  Each of these compiles a kernel module the framework never calls. They     */
/*  are dropped by --gc-sections either way, so this costs no flash — but      */
/*  saying so here is what stops a future reader assuming the framework has    */
/*  queues or software timers to build on. Turn one on when something needs    */
/*  it, and add the corresponding source to the Makefile.                     */

#define configUSE_TIMERS 0 /* No software timers; timers.c not built.  */
#define configUSE_QUEUE_SETS 0
#define configUSE_CO_ROUTINES 0

/* V11 introduced explicit switches for these two, both defaulting to 1. Their
 * source files are not built, so leaving the defaults in place would have the
 * config claim a feature the image does not contain — harmless today (nothing
 * references them, and the link proves it) but exactly the kind of drift that
 * makes a config file stop describing the build. */
#define configUSE_EVENT_GROUPS 0
#define configUSE_STREAM_BUFFERS 0

#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configGENERATE_RUN_TIME_STATS 0
#define configUSE_TICKLESS_IDLE 0
#define configQUEUE_REGISTRY_SIZE 0

/* Newlib's reentrancy structure is not per-task here; nothing in the framework
 * uses errno or strtok across a context switch. */
#define configUSE_NEWLIB_REENTRANT 0

/* ========================================================================= */
/*  Error detection                                                          */
/* ========================================================================= */

/* Mode 2: the kernel fills each stack with a known byte at creation and checks
 * the last 20 bytes on every context switch, on top of mode 1's stack-pointer
 * bounds test. Mode 1 alone misses an overflow that happened and unwound
 * between two switches, which is the common case for a deep call chain.
 *
 * The cost is a memset per task at creation and a 20-byte comparison per
 * switch. At 1 kHz with a handful of tasks that is not measurable, and it is
 * the only thing standing between a mis-sized stack and silent memory
 * corruption — see the note at the top of this file. */
#define configCHECK_FOR_STACK_OVERFLOW 2

/* Calls vApplicationMallocFailedHook when pvPortMalloc returns NULL. */
#define configUSE_MALLOC_FAILED_HOOK 1

/* Optional API. INCLUDE_vTaskSuspend also governs whether a portMAX_DELAY
 * block is truly indefinite rather than a very long timeout, which is what
 * PLAT_TASK_WAIT_FOREVER promises. */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle 0
#define INCLUDE_eTaskGetState 0
#define INCLUDE_xTimerPendFunctionCall 0
#define INCLUDE_xTaskAbortDelay 0
#define INCLUDE_xSemaphoreGetMutexHolder 0
#define INCLUDE_xTaskResumeFromISR 0

/* V11 renamed vTaskDelayUntil to xTaskDelayUntil, which returns whether it
 * actually delayed. impl_task.c calls the x-form and uses that return value
 * directly instead of recomputing the overrun by hand, which is what it had to
 * do under V10.3.1 where only the void form existed. The old name survives as
 * a compatibility macro; nothing here uses it. */
#define INCLUDE_xTaskDelayUntil 1

/* ========================================================================= */
/*  Cortex-M7 port                                                           */
/* ========================================================================= */

/* @par Why the ARM_CM4F port is used on a Cortex-M7
 * Upstream ships two options for M7 (portable/GCC/ARM_CM7/ReadMe.txt): the M4F
 * port, or an ARM_CM7/r0p1 port that adds a workaround for errata 837070. The
 * r0p1 port is required only on that core revision and is safe everywhere; the
 * M4F port is what upstream *recommends* for any later revision.
 *
 * This H723 reports r1p2 (__CM7_REV 0x0110 in stm32h723xx.h), so the M4F port is
 * the right choice and the kernel tree needs no second port installed. If this
 * ever moves to an r0p1 part, swap in ARM_CM7/r0p1 — the config below does not
 * change, only the port source and its include path.
 *
 * The FPU differs from F4 and that is handled by the compiler flags, not here:
 * M7 has fpv5-d16 (double precision) versus M4F's fpv4-sp-d16. The port only
 * needs to know an FPU exists, which is what configENABLE_FPU says. */
#define configENABLE_FPU 1
#define configENABLE_MPU 0

/* Number of implemented priority bits. The STM32H7 NVIC uses 4, giving 16
 * levels — same as F4, so the priority values below carry over unchanged.
 * HAL_Init selects NVIC_PRIORITYGROUP_4, so all four are preemption bits and
 * none are subpriority. */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

/* Priority of the kernel's own tick and yield exceptions: the lowest, so any
 * interrupt can preempt a context switch. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

/* The highest-priority interrupt that may call a FromISR API. Every peripheral
 * interrupt in this project is set to 5 by CubeMX, so all of them qualify —
 * and note how little headroom that leaves: an interrupt raised to 4 becomes
 * unable to call PLAT_Task_Notify, and vPortValidateInterruptPriority stops
 * the firmware the first time it tries.
 *
 * Interrupts numerically above this are never masked by a kernel critical
 * section, so they may not touch the kernel at all. Nothing in the project
 * uses that band today. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY                                                            \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Must not be zero: zero is the highest possible priority, which would make
 * every FromISR call illegal. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY                                                       \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* ========================================================================= */
/*  Assertions                                                               */
/* ========================================================================= */

/**
 * @brief Stop with the failure recorded, rather than in an anonymous spin.
 *
 * @par Why not the plain for(;;)
 * The CubeMX default was `taskDISABLE_INTERRUPTS(); for(;;);` — correct, but it
 * stops with interrupts off and nothing recorded, so every kernel assertion
 * looks identical from outside: a board that does nothing. Since RTT is already
 * linked, the file and line are worth more than the two instructions they cost,
 * and this is a path that by definition runs once.
 *
 * The RTT write happens before interrupts are disabled: SEGGER_RTT_printf
 * touches a control block the debugger polls, and with interrupts already off
 * a half-written message would be the last thing ever emitted.
 */
void RTOS_AssertFailed(const char* file, unsigned long line);

#define configASSERT(x)                                                                            \
    if ((x) == 0)                                                                                  \
    {                                                                                              \
        RTOS_AssertFailed(__FILE__, __LINE__);                                                     \
    }

/* ========================================================================= */
/*  Exception handler names                                                  */
/* ========================================================================= */
/*  The port implements the three kernel exceptions under its own names, but   */
/*  the vector table built by startup_stm32h723xx.s refers to the CMSIS ones.  */
/*  These macros are the standard way to reconcile that, and they are why      */
/*  stm32h7xx_it.c must NOT also define these three handlers — see the note    */
/*  in that file.                                                             */

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* Deliberately NOT mapped: xPortSysTickHandler is not SysTick_Handler here.
 *
 * The HAL timebase for this project is TIM2, not SysTick — CubeMX was
 * configured that way so that HAL_Delay keeps working at a priority the kernel
 * does not mask. The kernel still wants the SysTick exception for its own tick,
 * and it configures the peripheral itself in vPortSetupTimerInterrupt. So
 * SysTick belongs entirely to FreeRTOS and TIM2 entirely to the HAL, and
 * mapping this macro would put the kernel's handler where its own setup code
 * does not expect it.
 *
 * SysTick_Handler is therefore defined in rtos_hooks.c, on this side of the
 * vendor boundary. Under F4 it lived in a USER CODE region of stm32f4xx_it.c;
 * the H7 regeneration produced an it.c without it, which left the startup file's
 * weak Default_Handler — an infinite loop — as the only definition. The kernel
 * tick would simply never have arrived. Defining it in framework code means a
 * future Generate Code cannot drop it again. */

#endif /* FREERTOS_CONFIG_H */
