/**
 * @file rtos_fault.h
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 */

#ifndef RTOS_FAULT_H
#define RTOS_FAULT_H

/**
 * @brief Route memory-management, bus and usage faults to their own handlers.
 *
 * @par Why this is needed at all
 * Cortex-M leaves the three configurable fault exceptions disabled at reset. While
 * they are disabled every such fault escalates to HardFault, so rtos_fault.c's
 * MemManage_Handler, BusFault_Handler and UsageFault_Handler exist, are linked, sit
 * in the vector table — and can never be entered. Everything is still reported,
 * because the HardFault handler decodes CFSR either way, but the fault kind in the
 * message is always "HardFault" and the escalation costs the one thing a fault
 * report is for: on an escalated fault the stacked frame can be the escalation's
 * rather than the original access's.
 *
 * This matters more than usual on this board because the MPU is enabled (main.c
 * configures region 0 over AXI SRAM), so an MPU violation is a reachable fault
 * whose own handler is switched off.
 *
 * Also enables division-by-zero trapping, which is off at reset: an integer divide
 * by zero otherwise yields 0 and carries on, which is a wrong number rather than a
 * fault and correspondingly harder to find.
 *
 * Idempotent and safe before the scheduler. Reached from application code as
 * PLAT_Task_FaultInit rather than by name — this header exists for the ops table
 * that binds the two.
 */
void RTOS_FaultInit(void);

#endif /* RTOS_FAULT_H */
