/**
 * @file util_msgbus.h
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 */

#ifndef UTIL_MSGBUS_H
#define UTIL_MSGBUS_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/** @brief How many distinct topics the bus can hold. */
#define UTIL_MSGBUS_MAX_TOPICS 16u

/** @brief Largest single message, in bytes. */
#define UTIL_MSGBUS_MAX_MSG_BYTES 128u

/** @brief Longest topic name, including the terminator. */
#define UTIL_MSGBUS_MAX_NAME 16u

/** @brief Returned by Register and Find when no topic could be produced. */
#define UTIL_MSGBUS_INVALID_ID 0xFFu

/** @brief Block indefinitely in UTIL_MsgBus_Wait. */
#define UTIL_MSGBUS_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief How many times a seqlock read is retried before giving up.
 *
 * Each retry means a publish interrupted the copy. Three is generous: a fourth
 * consecutive collision implies the publisher is running about as fast as the
 * reader can copy, at which point reporting failure is more useful than spinning.
 */
#define UTIL_MSGBUS_MAX_RETRIES 3u

/* ========================================================================= */
/*  Types                                                                    */
/* ========================================================================= */

/** @brief Handle to a topic. Compare against UTIL_MSGBUS_INVALID_ID. */
typedef uint8_t UTIL_MsgBus_Id;

/**
 * @brief One subscriber's view of a topic.
 *
 * Holds the generation this subscriber last saw, which is what lets several
 * subscribers consume the same topic at their own rates without interfering. Treat
 * the fields as opaque; place one wherever the subscribing code lives, typically a
 * file-scope static in the task that reads the topic.
 */
typedef struct
{
    UTIL_MsgBus_Id id;       /**< Topic this refers to.                        */
    uint32_t       seen;     /**< Generation last copied out.                  */
    bool           attached; /**< False until a successful Subscribe.          */
    void*          waiter;   /**< Task to wake; NULL unless WantWake was used. */
} UTIL_MsgBus_Sub_s;

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

/**
 * @brief Bring up the bus. Call once, before any Register.
 *
 * Safe to call before the scheduler starts, which is the usual case: topics are
 * normally registered during bring-up so that no task has to order itself after
 * another task's registration.
 *
 * @return true on success. false means the mutex could not be created, and every
 *         later call will refuse rather than run unprotected.
 */
bool UTIL_MsgBus_Init(void);

/* ========================================================================= */
/*  Publishing side                                                          */
/* ========================================================================= */

/**
 * @brief Create a topic, or return the existing one with this name.
 *
 * Idempotent by name, so two modules that both publish or subscribe to "imu" can
 * each register it without coordinating, and neither has to run first.
 *
 * @par Why a mismatched size is refused rather than accepted
 * Returning the existing topic when @p msg_bytes disagrees would let one side
 * publish a 32-byte struct that the other reads as 48 bytes, which is a silent
 * memory error rather than a message-bus problem. The second registration fails
 * instead, at bring-up, where it is attributable.
 *
 * @param name       Topic name, copied. Must be shorter than UTIL_MSGBUS_MAX_NAME.
 * @param msg_bytes  Message size, 1..UTIL_MSGBUS_MAX_MSG_BYTES.
 * @return Topic id, or UTIL_MSGBUS_INVALID_ID if the bus is not initialized, the
 *         arguments are out of range, the table is full, or a topic of this name
 *         exists with a different size.
 */
UTIL_MsgBus_Id UTIL_MsgBus_Register(const char* name, uint8_t msg_bytes);

/**
 * @brief Create a topic that publishes and reads without taking the bus lock.
 *
 * Same as UTIL_MsgBus_Register in every respect except how concurrency is handled,
 * so the returned id is used with the same Publish / Check / Copy / Wait.
 *
 * @par What this buys
 * The ordinary path serialises every topic on one bus-wide mutex, so publishing
 * "imu" briefly blocks a task reading "gimbal_cmd" even though the two are
 * unrelated. The worst case is bounded and small — one memcpy, roughly a
 * microsecond at 128 bytes — which is irrelevant at 1 kHz and starts to matter well
 * above it. A seqlock topic removes that coupling: the publisher never blocks and
 * never waits, so its timing cannot be perturbed by any subscriber or by any other
 * topic's traffic.
 *
 * @par What it costs
 * A reader may have to repeat its copy when a publish lands in the middle of it.
 * Retries are bounded (see UTIL_MsgBus_Copy), so a reader still completes in
 * bounded time, but its worst case is a few copies rather than one. That trade is
 * worth making for a high-rate topic read by loops that must not block, and not
 * worth making for a low-rate one.
 *
 * @par The single-publisher requirement
 * A seqlock has exactly one writer. Publish claims the topic with an atomic
 * compare-exchange, so a second publisher is refused rather than allowed to
 * interleave its payload with the first — but that is a diagnostic, not a licence:
 * the refused message is simply lost. Design for one publisher and treat a false
 * return as the bug it indicates.
 *
 * Interrupt handlers may publish to a seqlock topic, which the locked path forbids.
 * That still counts as the one publisher: an interrupt and a task both publishing
 * the same topic is the multi-publisher case.
 *
 * @param name       Topic name, copied. Must be shorter than UTIL_MSGBUS_MAX_NAME.
 * @param msg_bytes  Message size, 1..UTIL_MSGBUS_MAX_MSG_BYTES.
 * @return Topic id, or UTIL_MSGBUS_INVALID_ID on the same conditions as
 *         UTIL_MsgBus_Register, or if a topic of this name exists with the other
 *         concurrency mode — the two are not interchangeable, and silently handing
 *         back a locked topic to a caller that asked for a lock-free one would
 *         reintroduce exactly the blocking it was avoiding.
 */
UTIL_MsgBus_Id UTIL_MsgBus_RegisterSeqlock(const char* name, uint8_t msg_bytes);

/**
 * @brief Look up an existing topic by name.
 *
 * @param name  Topic name.
 * @return Topic id, or UTIL_MSGBUS_INVALID_ID if no such topic is registered.
 */
UTIL_MsgBus_Id UTIL_MsgBus_Find(const char* name);

/**
 * @brief Publish a message.
 *
 * @par Cost
 * One memcpy plus a counter increment, under the lock. Deliberately independent of
 * how many subscribers exist: nothing is dispatched here, so a publisher's period
 * cannot be stretched by adding subscribers or by one subscriber being slow. That
 * is the main structural difference from a callback-dispatch bus.
 *
 * @par Publishing during bring-up
 * Before the scheduler starts there is no concurrency, so the copy proceeds without
 * the lock rather than failing. From an interrupt it refuses, because there a lock
 * genuinely cannot be taken and a torn message is possible — publish from a task
 * the interrupt notifies instead.
 *
 * @param id   Topic to publish to.
 * @param msg  Message to copy in. Must be at least the topic's registered size.
 * @return true when the message was stored.
 */
bool UTIL_MsgBus_Publish(UTIL_MsgBus_Id id, const void* msg);

/* ========================================================================= */
/*  Subscribing side                                                         */
/* ========================================================================= */

/**
 * @brief Attach a subscription to a topic.
 *
 * @par Starting position
 * The subscription starts having seen nothing, so if the topic already holds data
 * the first Check reports new data and the first Copy yields that data. A
 * subscriber that starts late therefore gets the current value rather than waiting
 * for the next publish — which is what a control loop reading a sensor topic wants.
 *
 * @param sub  Subscription to initialize.
 * @param id   Topic to follow.
 * @return true on success; false on a NULL @p sub or an unknown topic.
 */
bool UTIL_MsgBus_Subscribe(UTIL_MsgBus_Sub_s* sub, UTIL_MsgBus_Id id);

/**
 * @brief Whether this subscription has data it has not copied yet.
 *
 * Cheap: one comparison against the topic's generation, no copy. Use it to skip
 * work when nothing has arrived, rather than re-processing the same message.
 *
 * @param sub  Subscription to test.
 * @return true when a Copy would yield something new.
 */
bool UTIL_MsgBus_Check(const UTIL_MsgBus_Sub_s* sub);

/**
 * @brief Copy the current message out, and mark it seen.
 *
 * Copies into caller storage rather than handing out a pointer, so the caller can
 * read the message at leisure while publishers keep running. Handing out a pointer
 * into the topic is what makes a bus like this tear under concurrency.
 *
 * Succeeds even when no new message has arrived, yielding the current value again;
 * gate on Check when repeat processing matters.
 *
 * @par On a seqlock topic
 * The copy is retried when a publish lands in the middle of it, up to
 * UTIL_MSGBUS_MAX_RETRIES times. Exceeding that returns false rather than looping:
 * an unbounded retry would trade the blocking this topic type exists to avoid for
 * an unbounded spin, which is worse. A false here means the publisher is running at
 * a rate comparable to the reader's own copy, which is a design problem the caller
 * should see rather than absorb.
 *
 * @param sub  Subscription to read through.
 * @param dst  Storage of at least the topic's registered size.
 * @return true when @p dst was filled.
 */
bool UTIL_MsgBus_Copy(UTIL_MsgBus_Sub_s* sub, void* dst);

/**
 * @brief Ask to be woken when this topic is published to.
 *
 * @par When this is better than polling
 * A control loop already runs on a fixed period, so Check costs it one comparison
 * per cycle and waiting buys nothing. An event-driven task is the opposite: without
 * this it must either spin or sleep on a timer, which trades latency against wasted
 * cycles. Waiting removes that trade — the task runs when data arrives and not
 * otherwise.
 *
 * @par What it does not change
 * Publishing stays O(1) and stays out of the subscriber's code: a publish performs
 * one notification per waiting subscriber, which is a constant-time scheduler call,
 * and the subscriber's own processing still happens on the subscriber's stack at the
 * subscriber's priority. That is the difference from a callback bus, where the
 * subscriber's work lands on the publisher's context.
 *
 * Call from the task that will wait, since the task handle is taken from the caller.
 * At most one task can wait on a subscription; a second call replaces the first,
 * which is what a task restart needs.
 *
 * @param sub  Subscription to arm. Must already be attached.
 * @return true on success; false on a NULL or unattached subscription, or when
 *         called from outside a task (an interrupt, or before the scheduler).
 */
bool UTIL_MsgBus_WantWake(UTIL_MsgBus_Sub_s* sub);

/**
 * @brief Block until this subscription has new data.
 *
 * Returns as soon as data is pending, then leaves the copy to the caller — so the
 * usual shape is Wait followed by Copy. Returning without copying keeps the payload
 * off this function's stack and lets a caller that woke for several reasons decide
 * what to read.
 *
 * @par Already-pending data returns immediately
 * A publish that landed between the last Copy and this call does not cause a wait,
 * so a subscriber cannot miss an update by being slow to come back. This is checked
 * before blocking rather than relying on the notification alone.
 *
 * @param sub         Subscription previously armed with UTIL_MsgBus_WantWake.
 * @param timeout_ms  Milliseconds to wait, or UTIL_MSGBUS_WAIT_FOREVER.
 * @return true when new data is available to Copy. false on timeout, on a
 *         subscription that was never armed, or when called from a context that
 *         cannot block.
 */
bool UTIL_MsgBus_Wait(UTIL_MsgBus_Sub_s* sub, uint32_t timeout_ms);

/* ========================================================================= */
/*  Introspection                                                            */
/* ========================================================================= */

/**
 * @brief How many messages have ever been published to a topic.
 *
 * Intended for bring-up and health checks: a topic whose count stays at zero has a
 * subscriber but no publisher, which is otherwise silent.
 *
 * @param id  Topic to query.
 * @return Publish count, or 0 for an unknown topic.
 */
uint32_t UTIL_MsgBus_Generation(UTIL_MsgBus_Id id);

/**
 * @brief Whether a topic uses the lock-free (seqlock) path.
 *
 * @param id  Topic to query.
 * @return true for a seqlock topic; false for a locked one or an unknown id.
 */
bool UTIL_MsgBus_IsSeqlock(UTIL_MsgBus_Id id);

/**
 * @brief How many seqlock reads have had to retry, across all topics.
 *
 * A health counter, not an error count: an occasional retry is the mechanism
 * working. Steady growth means a reader is losing races often enough that the topic
 * is a poor fit for seqlock, or that its message is too large.
 *
 * @return Cumulative retry count since Init.
 */
uint32_t UTIL_MsgBus_RetryCount(void);

/**
 * @brief Registered message size of a topic.
 *
 * @param id  Topic to query.
 * @return Size in bytes, or 0 for an unknown topic.
 */
uint8_t UTIL_MsgBus_MsgBytes(UTIL_MsgBus_Id id);

/**
 * @brief How many topics are registered.
 *
 * @return Count, 0..UTIL_MSGBUS_MAX_TOPICS.
 */
uint8_t UTIL_MsgBus_TopicCount(void);

#endif /* UTIL_MSGBUS_H */
