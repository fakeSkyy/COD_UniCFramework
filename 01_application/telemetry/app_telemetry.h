/**
 * @file app_telemetry.h
 * @author Gao Xing
 * @date 2026/8/14
 * @version 1.0
 */

#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Resolve the debug port and register the completion callback.
 *
 * @return true when telemetry can run; false if the board has no debug UART, in
 *         which case Step does nothing and the rest of the firmware is unaffected.
 *         A caller should treat false as "no plot", not as an error.
 */
bool App_Telemetry_Init(void);

/**
 * @brief Build and send one frame, at the divided rate.
 *
 * Call from the attitude loop, after the estimate has been updated. Cheap on the
 * cycles it does nothing: a decrement and a compare. Angles arrive in radians and
 * go out in degrees, which is what a plot is read in.
 *
 * @par The receiver has to be told the channel count
 * VOFA+ justFloat carries no header and no channel count — the receiver infers it
 * from how many bytes arrived before the frame terminator. A mismatch is therefore
 * detected nowhere: every trace silently shows the wrong quantity. TELEM_CHANNELS
 * in app_telemetry.c is the number to configure.
 *
 * @param roll_rad   Roll, radians.
 * @param pitch_rad  Pitch, radians.
 * @param yaw_rad    Yaw, radians.
 * @param rate_rads  Angular rate, 3 elements, rad/s. May be NULL, which sends zeros
 *                   for those channels rather than skipping the frame — dropping it
 *                   would leave the plot with gaps whose cause is indistinguishable
 *                   from a lost frame.
 * @param temp_c     Die temperature, degrees Celsius.
 */
void App_Telemetry_Step(float roll_rad, float pitch_rad, float yaw_rad, const float* rate_rads,
                        float temp_c);

/**
 * @brief Frames dropped because the port was still busy.
 *
 * A slowly growing count is normal at a divider that puts the line near capacity. A
 * count climbing as fast as the frame rate means nothing is getting out — check the
 * baud rate against TELEM_DIVIDER.
 *
 * @return Total skipped since Init.
 */
uint32_t App_Telemetry_Skipped(void);

#endif /* APP_TELEMETRY_H */
