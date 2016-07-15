/*
 * koruza-driver - KORUZA driver
 *
 * Copyright (C) 2016 Jernej Kos <jernej@kos.mx>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef KORUZA_MESSAGE_H
#define KORUZA_MESSAGE_H

#include <stdint.h>
#include <sys/types.h>

// Maximum number of TLVs inside a message.
#define MAX_TLV_COUNT 25

/**
 * TLVs supported by the protocol.
 */
typedef enum {
  TLV_COMMAND = 1,
  TLV_REPLY = 2,
  TLV_CHECKSUM = 3,
  TLV_MOTOR_POSITION = 4,
  TLV_CURRENT_READING = 5,
} tlv_type_t;

/**
 * Commands supported by the command TLV.
 */
typedef enum {
  COMMAND_GET_STATUS = 1,
  COMMAND_MOVE_MOTOR = 2,
  COMMAND_SEND_IR = 3,
  COMMAND_REBOOT = 4,
  COMMAND_FIRMWARE_UPGRADE = 5,
} tlv_command_t;

/**
 * Replies supported by the reply TLV.
 */
typedef enum {
  REPLY_STATUS_REPORT = 1
} tlv_reply_t;

/**
 * Contents of the motor position TLV.
 */
typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} tlv_motor_position_t;

/**
 * Message operations result codes.
 */
typedef enum {
  MESSAGE_SUCCESS = 0,
  MESSAGE_ERROR_TOO_MANY_TLVS = -1,
  MESSAGE_ERROR_OUT_OF_MEMORY = -2,
  MESSAGE_ERROR_BUFFER_TOO_SMALL = -3,
  MESSAGE_ERROR_PARSE_ERROR = -4,
  MESSAGE_ERROR_CHECKSUM_MISMATCH = -5,
  MESSAGE_ERROR_TLV_NOT_FOUND = -6
} message_result_t;

/**
 * Representation of a TLV.
 */
typedef struct {
  uint8_t type;
  uint16_t length;
  uint8_t *value;
} tlv_t;

/**
 * Representation of a protocol message.
 */
typedef struct {
  size_t length;
  tlv_t tlv[MAX_TLV_COUNT];
} message_t;

/**
 * Initializes a protocol message. This function should be called on any newly
 * created message to ensure that the underlying memory is properly initialized.
 *
 * @param message Message instance to initialize
 * @return Operation result code
 */
message_result_t message_init(message_t *message);

/**
 * Frees a protocol message.
 *
 * @param message Message instance to free
 */
void message_free(message_t *message);

/**
 * Parses a protocol message.
 *
 * @param message Destination message instance to parse into
 * @param data Raw data to parse
 * @param length Size of the data buffer
 * @return Operation result code
 */
message_result_t message_parse(message_t *message, const uint8_t *data, size_t length);

/**
 * Adds a raw TLV to a protocol message.
 *
 * @param message Destination message instance to add the TLV to
 * @param type TLV type
 * @param length TLV length
 * @param value TLV value
 * @return Operation result code
 */
message_result_t message_tlv_add(message_t *message, uint8_t type, uint16_t length, const uint8_t *value);

/**
 * Adds a command TLV to a protocol message.
 *
 * @param message Destination message instance to add the TLV to
 * @param command Command argument
 * @return Operation result code
 */
message_result_t message_tlv_add_command(message_t *message, tlv_command_t command);

/**
 * Adds a reply TLV to a protocol message.
 *
 * @param message Destination message instance to add the TLV to
 * @param reply Reply argument
 * @return Operation result code
 */
message_result_t message_tlv_add_reply(message_t *message, tlv_reply_t reply);

/**
 * Adds a motor position TLV to a protocol message.
 *
 * @param message Destination message instance to add the TLV to
 * @param position Motor position structure
 * @return Operation result code
 */
message_result_t message_tlv_add_motor_position(message_t *message, const tlv_motor_position_t *position);

/**
 * Adds a current reading TLV to a protocol message.
 *
 * @param message Destination message instance to add the TLV to
 * @param current Current reading
 * @return Operation result code
 */
message_result_t message_tlv_add_current_reading(message_t *message, uint16_t current);

/**
 * Adds a checksum TLV to a protocol message. The checksum value is automatically
 * computed over all the TLVs currently contained in the message.
 *
 * @param message Destination message instance to add the TLV to
 * @return Operation result code
 */
message_result_t message_tlv_add_checksum(message_t *message);

/**
 * Find the first TLV of a given type in a message and copies it.
 *
 * @param message Message instance to get the TLV from
 * @param type Type of TLV that should be returned
 * @param destination Destination buffer
 * @param length Length of the destination buffer
 * @return Operation result code
 */
message_result_t message_tlv_get(const message_t *message, uint8_t type, uint8_t *destination, size_t length);

/**
 * Find the first command TLV in a message and copies it.
 *
 * @param message Message instance to get the TLV from
 * @param command Destination command variable
 * @return Operation result code
 */
message_result_t message_tlv_get_command(const message_t *message, tlv_command_t *command);

/**
 * Find the first reply TLV in a message and copies it.
 *
 * @param message Message instance to get the TLV from
 * @param reply Destination reply variable
 * @return Operation result code
 */
message_result_t message_tlv_get_reply(const message_t *message, tlv_reply_t *reply);

/**
 * Find the first motor position TLV in a message and copies it.
 *
 * @param message Message instance to get the TLV from
 * @param position Destination position variable
 * @return Operation result code
 */
message_result_t message_tlv_get_motor_position(const message_t *message, tlv_motor_position_t *position);

/**
 * Find the first current reading TLV in a message and copies it.
 *
 * @param message Message instance to get the TLV from
 * @param current Destination current variable
 * @return Operation result code
 */
message_result_t message_tlv_get_current_reading(const message_t *message, uint16_t *current);

/**
 * Returns the size a message would take in its serialized form.
 *
 * @param message Message instance to calculate the size for
 */
size_t message_serialized_size(const message_t *message);

/**
 * Serializes a protocol message into a destination buffer.
 *
 * @param buffer Destination buffer
 * @param length Length of the destination buffer
 * @param message Message instance to serialize
 * @return Number of bytes written serialized to the buffer or error code
 */
ssize_t message_serialize(uint8_t *buffer, size_t length, const message_t *message);

/**
 * Prints a debug representation of a protocol message.
 *
 * @param message Protocol message to print
 */
void message_print(const message_t *message);

#endif
