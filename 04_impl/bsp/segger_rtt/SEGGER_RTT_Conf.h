/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 2014 - 2024  SEGGER Microcontroller GmbH                 *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
----------------------------------------------------------------------
Purpose : User configuration for SEGGER RTT, owned by this project.
----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/* ==========================================================================
 * Why this file is here and not under 05_vender
 * ==========================================================================
 *
 * Same split as FreeRTOS: 05_vender/segger_rtt/ is upstream V8.58.0, never
 * edited, and everything this project decides lives here. Upstream V8 made that
 * split its own design — SEGGER_RTT_ConfDefaults.h holds every default and says
 * "do not change this file", while SEGGER_RTT_Conf.h is the user's and ships
 * empty. So this file only states what differs from the defaults.
 *
 * The V7 layout that used to be vendored here was the opposite: one 429-line
 * Conf.h containing all the defaults, which had to be edited in place. Anything
 * changed there was indistinguishable from upstream text.
 *
 * Upstream's Config/ directory is not installed, so this is the only
 * SEGGER_RTT_Conf.h on the include path — ConfDefaults.h can only find this one.
 * Installing upstream's would create a second candidate resolved by -I order,
 * which is why the Makefile does not copy it.
 * ==========================================================================
 */

/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */

/* Interrupt priority that RTT masks while it writes to its buffers.
 *
 * @par Why this is not left at the default
 * The default is 0x20, which on this 4-priority-bit NVIC masks everything from
 * priority 2 down — including every peripheral interrupt in this project, all of
 * which CubeMX sets to 5. Safe, but more than required: it also blocks the two
 * levels above anything that can reach RTT.
 *
 * SEGGER's own documentation says to match FreeRTOS's
 * configMAX_SYSCALL_INTERRUPT_PRIORITY, which here is
 *
 *     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS)
 *         = 5 << 4
 *         = 0x50
 *
 * That masks exactly the band that may call into the kernel — and therefore the
 * band that may log — and leaves priorities 0..4 running. Nothing in that band
 * touches RTT today, so this narrows the critical section without weakening it.
 *
 * Stated numerically rather than by including FreeRTOSConfig.h: this header is
 * pulled in by RTT's own assembly-adjacent code paths, and the FreeRTOS config
 * carries C declarations. If the FreeRTOS value ever changes, the static assert
 * in rtos_hooks.c catches the divergence. */
#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY (0x50)

#endif
/*************************** End of file ****************************/
