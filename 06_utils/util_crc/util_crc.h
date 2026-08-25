/**
 * @file util_crc.h
 * @author Gao Xing
 * @date 2026/8/3
 * @version 1.0
 */

#ifndef UTIL_CRC_H
#define UTIL_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ========================================================================= */
/*  Algorithm parameters                                                     */
/* ========================================================================= */

/**
 * @brief Initial CRC-8 register value.
 *
 * Not the 0x00 of a textbook CRC-8: a non-zero seed is what makes leading zero
 * bytes affect the result, so a frame that gains or loses them is still caught.
 */
#define UTIL_CRC8_INIT 0xFFu

/** @brief Initial CRC-16 register value, for the same reason. */
#define UTIL_CRC16_INIT 0xFFFFu

/**
 * @brief Reflected CRC-8 polynomial, 0x8C — normal form 0x31, x^8+x^5+x^4+1.
 *
 * Documented for identification only; the implementation is table-driven and
 * does not read this. The pairing with @ref UTIL_CRC8_INIT is the wire format
 * this module has to stay compatible with, not a free choice.
 */
#define UTIL_CRC8_POLY_REFLECTED 0x8Cu

/**
 * @brief Reflected CRC-16 polynomial, 0x8408 — normal form 0x1021 (CCITT).
 *
 * With an init of 0xFFFF and no final xor this is the variant catalogued as
 * CRC-16/MCRF4XX. Note it is NOT CRC-16/MODBUS, which shares the init but uses
 * polynomial 0xA001 — the two disagree on every input, so do not substitute a
 * MODBUS routine for this one.
 */
#define UTIL_CRC16_POLY_REFLECTED 0x8408u

/* ========================================================================= */
/*  CRC-8                                                                    */
/* ========================================================================= */

/**
 * @brief Compute the CRC-8 of a byte range.
 *
 * Table-driven, one lookup per byte.
 *
 * @par Length convention
 * @p len is the number of bytes to feed in, and nothing else. This differs from
 * the Append and Verify calls below, which take a whole frame length including
 * its checksum byte — a distinction worth keeping straight, since passing a
 * frame length here would fold the old checksum into the new one.
 *
 * @param data  Bytes to feed in. May be NULL only when @p len is 0.
 * @param len   Number of bytes to process; 0 returns @p init unchanged, which is
 *              the identity the algorithm requires for an empty range.
 * @param init  Starting register value; use @ref UTIL_CRC8_INIT for a fresh
 *              checksum, or the result of a previous call to continue across a
 *              scattered buffer.
 * @return CRC-8 over the range.
 */
uint8_t UTIL_CRC8_Calc(const uint8_t* data, size_t len, uint8_t init);

/**
 * @brief Write the CRC-8 of a frame into its last byte.
 *
 * @param frame      Frame buffer; the first @p frame_len - 1 bytes are the
 *                   payload and the last receives the checksum.
 * @param frame_len  Total frame length INCLUDING the checksum byte, so it must
 *                   be at least 2 — one payload byte plus the checksum. A
 *                   shorter frame has no payload to protect.
 * @return true when the checksum was written; false if @p frame is NULL or
 *         @p frame_len is below 2, so a caller learns the frame went out
 *         unprotected rather than silently sending a zero checksum.
 */
bool UTIL_CRC8_Append(uint8_t* frame, size_t frame_len);

/**
 * @brief Check a frame's trailing CRC-8 byte.
 *
 * @param frame      Frame to check, checksum included.
 * @param frame_len  Total frame length INCLUDING the checksum byte; minimum 2.
 * @return true only when @p frame is non-NULL, @p frame_len is at least 2, and
 *         the stored checksum matches the computed one.
 */
bool UTIL_CRC8_Verify(const uint8_t* frame, size_t frame_len);

/* ========================================================================= */
/*  CRC-16                                                                   */
/* ========================================================================= */

/**
 * @brief Compute the CRC-16 of a byte range.
 *
 * Table-driven, one lookup per byte. The same length convention as
 * UTIL_CRC8_Calc applies: @p len is bytes to process, not a frame length.
 *
 * @param data  Bytes to feed in. May be NULL only when @p len is 0.
 * @param len   Number of bytes to process; 0 returns @p init unchanged.
 * @param init  Starting register value; @ref UTIL_CRC16_INIT for a fresh
 *              checksum, or a previous result to continue a running CRC.
 * @return CRC-16 over the range.
 */
uint16_t UTIL_CRC16_Calc(const uint8_t* data, size_t len, uint16_t init);

/**
 * @brief Write the CRC-16 of a frame into its last two bytes, little-endian.
 *
 * Byte order is low byte first, matching the wire format this module exists to
 * speak. Do not assume it follows the host's endianness.
 *
 * @param frame      Frame buffer; the first @p frame_len - 2 bytes are the
 *                   payload and the last two receive the checksum.
 * @param frame_len  Total frame length INCLUDING the two checksum bytes, so at
 *                   least 3.
 * @return true when the checksum was written; false if @p frame is NULL or
 *         @p frame_len is below 3.
 */
bool UTIL_CRC16_Append(uint8_t* frame, size_t frame_len);

/**
 * @brief Check a frame's trailing little-endian CRC-16.
 *
 * @param frame      Frame to check, checksum included.
 * @param frame_len  Total frame length INCLUDING the two checksum bytes;
 *                   minimum 3.
 * @return true only when @p frame is non-NULL, @p frame_len is at least 3, and
 *         the stored checksum matches the computed one.
 */
bool UTIL_CRC16_Verify(const uint8_t* frame, size_t frame_len);

#endif /* UTIL_CRC_H */
