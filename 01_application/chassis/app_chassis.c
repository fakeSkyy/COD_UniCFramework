/**
 * @file app_chassis.c
 * @author Gao Xing
 * @date 2026/9/3
 * @version 1.0
 */

#include "app_chassis.h"

#include <stddef.h>

#include "board.h"
#include "dev_dji_motor.h"
#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_task.h"
#include "util_log.h"

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

#define CHASSIS_MOTOR_COUNT 4u

/**
 * @brief ESC identifier of the first wheel; the rest follow consecutively.
 *
 * The single range claim below depends on this: an M3508 reports under 0x200 + its ESC
 * id, so ids 1..4 give the contiguous span 0x201..0x204 that one filter element can
 * cover. Wheels dialled to non-consecutive ids (1, 2, 5, 6) would need either a wider
 * range — admitting identifiers nothing here owns — or one node per wheel. Nothing
 * detects that at build time, so the ids on the hardware are the contract.
 */
#define CHASSIS_FIRST_ESC_ID 1u

/** @brief Control frame identifier for M3508 IDs 1..4. */
#define CHASSIS_TX_ID 0x200u

/* The transmit node needs a receive identifier it will never actually use, because
 * every node claims one. 0x2FF is the other control identifier the DJI protocol
 * defines, so nothing on this bus reports under it — unlike, say, 0x205, which is a
 * GM6020's feedback and would collide the day one is added. */
#define CHASSIS_TX_NODE_RX_ID 0x2FFu

#define CHASSIS_PERIOD_MS 1u

/* Past this, a motor counts as absent. Four missed frames at the ESC's 1 kHz feedback
 * rate: long enough that one dropped frame on a busy bus is not an outage, short
 * enough that a real disconnection is caught within a control cycle or two. */
#define CHASSIS_OFFLINE_MS 4u

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

static DEV_DJI_Bus_s   bus;
static DEV_DJI_Motor_s motors[CHASSIS_MOTOR_COUNT];

/* One receive node for all four wheels: their feedback identifiers are consecutive
 * (0x201..0x204), so a single range claim covers them. Held only so nothing can
 * garbage-collect the pointer; after bring-up frames arrive through the callback. */
static CAN_Instance_s* rx_node;

static Task_s  task;
static uint8_t task_stack[1024];

/* Written by the task, read by App_Chassis_SetSpeed's caller. One float on a
 * single-core M7 is an aligned 32-bit store, so a torn read is not possible; a
 * mutex here would only add a way for a 1 kHz loop to block. */
static volatile float target_rpm;

static DWT_Instance_s* timebase;

/* ========================================================================= */
/*  Receive path                                                             */
/* ========================================================================= */

/**
 * @brief Route one feedback frame to the motor that reports under its identifier.
 *
 * Runs in interrupt context, so it does no logging and takes no lock —
 * DEV_DJIMotor_OnFeedback touches only its own motor's fields.
 *
 * @par Why this scans
 * Every wheel's feedback arrives through the one range node, so the identifier is the
 * only thing distinguishing them — a per-node token could not help even if one existed.
 * A linear scan over four motors is a handful of integer compares; if the count ever
 * grows enough to matter, index the span directly (id - fb_first), which the range
 * claim makes trivially correct.
 *
 * @param can   Node the frame arrived on; unused, since the identifier already
 *              says which motor it belongs to.
 * @param id    Frame identifier.
 * @param data  Payload.
 * @param len   Payload length.
 */
static void on_feedback(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len)
{
    (void) can;

    /* Read once, outside the loop: this is the DWT timeline every driver kicks from,
     * and taking two different readings for frames in one batch would make two
     * motors' ages disagree by the cost of the scan. */
    const uint32_t now_ms = (uint32_t) PLAT_DWT_GetTimeline_ms(timebase);

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if (DEV_DJIMotor_FeedbackId(&motors[i]) == id)
        {
            DEV_DJIMotor_OnFeedback(&motors[i], data, len, now_ms);
            return;
        }
    }
}

/* ========================================================================= */
/*  Bring-up                                                                 */
/* ========================================================================= */

/**
 * @brief Create every CAN node and attach every motor.
 *
 * @return true when both nodes came up and joined the bus.
 */
static bool chassis_init(void)
{
    timebase = Board_Timebase();

    if (timebase == NULL)
    {
        UTIL_LOG_E("chassis", "no timebase; cannot age feedback");
        return false;
    }

    /* The transmit node first, because DEV_DJIMotor_BusInit needs it before any motor
     * can be attached. */
    CAN_Instance_s* tx = Board_CANCreate(BOARD_CAN1, CHASSIS_TX_ID, CHASSIS_TX_NODE_RX_ID);

    if (tx == NULL)
    {
        UTIL_LOG_E("chassis", "transmit node refused");
        return false;
    }

    if (!DEV_DJIMotor_BusInit(&bus, tx, CHASSIS_TX_ID))
    {
        UTIL_LOG_E("chassis", "bus init refused");
        return false;
    }

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const uint8_t esc_id = (uint8_t) (CHASSIS_FIRST_ESC_ID + i);

        /* NULL controller: these wheels take a current command straight from
         * App_Chassis_SetSpeed. A velocity loop belongs here once there is one to
         * tune, and adding it is passing a DEV_DJI_Controller_s instead of NULL. */
        if (!DEV_DJIMotor_Attach(&motors[i], &bus, DEV_DJI_M3508, esc_id, NULL))
        {
            UTIL_LOG_E("chassis", "motor %u refused by bus", (unsigned) esc_id);
            return false;
        }
    }

    /* One range claim for every wheel's feedback, rather than a node each. The span is
     * derived from the motors themselves rather than written as a literal, so it cannot
     * drift from the ESC ids attached above. */
    const uint32_t fb_first = DEV_DJIMotor_FeedbackId(&motors[0]);
    const uint32_t fb_last  = DEV_DJIMotor_FeedbackId(&motors[CHASSIS_MOTOR_COUNT - 1u]);

    /* The span must cover exactly the wheels and nothing else. A mis-dialled ESC makes
     * the identifiers non-consecutive, and a range wide enough to still contain them
     * would admit identifiers this task does not own — so refuse rather than claim
     * traffic belonging to another device. */
    if ((fb_last - fb_first) != (CHASSIS_MOTOR_COUNT - 1u))
    {
        UTIL_LOG_E("chassis", "feedback ids 0x%03X..0x%03X are not %u consecutive",
                   (unsigned) fb_first, (unsigned) fb_last, (unsigned) CHASSIS_MOTOR_COUNT);
        return false;
    }

    rx_node = Board_CANCreateRange(BOARD_CAN1, CHASSIS_TX_ID, fb_first, fb_last);

    if (rx_node == NULL)
    {
        /* Some identifier in the span is already claimed on this bus, or the filter list
         * is full. Two nodes need two of FDCAN1's eight elements, so a failure here is a
         * collision with another device family rather than arithmetic. */
        UTIL_LOG_E("chassis", "receive node for 0x%03X..0x%03X refused", (unsigned) fb_first,
                   (unsigned) fb_last);
        return false;
    }

    /* Callback before Start, not after: a frame admitted by the filter before there is
     * somewhere to deliver it is discarded, and the first feedback frame arrives within
     * a millisecond of the peripheral starting. */
    PLAT_CAN_OnReceive(rx_node, on_feedback);

    if (!PLAT_CAN_Start(rx_node))
    {
        UTIL_LOG_E("chassis", "receive node could not start");
        return false;
    }

    /* Started last. Starting is idempotent per bus, so the receive node above has
     * already brought FDCAN1 up and this only adds the transmit node's own filter. */
    return PLAT_CAN_Start(tx);
}

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

/**
 * @brief Command the wheels once per period.
 *
 * @param arg  Unused.
 */
static void body(void* arg)
{
    (void) arg;

    uint32_t prev_tick = PLAT_Task_TickNow();
    uint32_t dt_cursor = PLAT_DWT_GetTick(timebase);

    for (;;)
    {
        /* Measured, not CHASSIS_PERIOD_MS: a controller stepped with a constant dt
         * silently changes its effective gains whenever the task runs late, and this
         * task sits below the attitude loop so it will be preempted. */
        const float dt = PLAT_DWT_GetDeltaT(timebase, &dt_cursor);

        for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
        {
            DEV_DJIMotor_SetTarget(&motors[i], target_rpm);
        }

        /* False means the mailboxes were full. The commands are retained, so the next
         * cycle transmits them; retransmission is disabled on this bus, so a frame
         * that loses arbitration is dropped rather than retried and this counter is
         * the only signal that the bus is oversubscribed. */
        (void) DEV_DJIMotor_CommitBus(&bus, dt);

        PLAT_Task_DelayUntil(&prev_tick, CHASSIS_PERIOD_MS);
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

bool App_Chassis_StartTask(uint8_t priority)
{
    /* Bring-up before the task exists, so a failure means no task rather than a task
     * commanding motors whose feedback never arrives. */
    if (!chassis_init())
    {
        return false;
    }

    return PLAT_Task_Create(&task, body, NULL, "chassis", task_stack, sizeof task_stack, priority);
}

bool App_Chassis_Online(void)
{
    /* Callable before bring-up — app_tasks treats a chassis failure as non-fatal, so a
     * caller can reach this with timebase still NULL, and PLAT_DWT_GetTimeline_ms
     * dereferences its argument without testing it. Reporting offline is also the
     * honest answer: no feedback has arrived. */
    if (timebase == NULL)
    {
        return false;
    }

    const uint32_t now_ms = (uint32_t) PLAT_DWT_GetTimeline_ms(timebase);

    for (uint8_t i = 0u; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if (DEV_DJIMotor_IsOffline(&motors[i], now_ms, CHASSIS_OFFLINE_MS))
        {
            return false;
        }
    }

    return true;
}

void App_Chassis_SetSpeed(float rpm) { target_rpm = rpm; }
