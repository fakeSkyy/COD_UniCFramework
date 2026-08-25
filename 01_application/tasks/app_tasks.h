/**
 * @file app_tasks.h
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdbool.h>

/**
 * @brief Create every application task and hand control to the scheduler.
 *
 * Call after Board_Init, as the last thing main does. Does not return: the
 * scheduler runs from here on, so reaching the line after the call means it could
 * not start.
 *
 * @par Where the tasks are
 * Each in its own module — app_indicator.c, app_imu.c — which is where a task's
 * stack, period and body belong, since all three follow from what it does. What
 * app_tasks.c holds is the priority table, because a priority is the one property of
 * a task that is meaningless in isolation and is drawn from a shared supply of seven.
 *
 * Neither names a vendor: tasks are created through PLAT_Task_Create and the
 * scheduler through PLAT_Task_StartScheduler, so both read the same whatever the impl
 * layer is bound to.
 *
 * @par This file is not CubeMX's
 * main.c is, and everything outside its USER CODE markers is rewritten on Generate
 * Code — task creation being exactly the kind of thing that would be lost. Keeping
 * it here means main.c holds one call inside a USER CODE region.
 *
 * @return false only if the scheduler failed to start, which in practice means the
 *         heap could not supply the kernel's own structures. A caller has nothing
 *         useful to do with this besides reporting and stopping.
 */
bool App_StartTasks(void);

#endif /* APP_TASKS_H */
