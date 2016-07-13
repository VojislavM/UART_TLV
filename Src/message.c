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
#include "message.h"
#include "crc32.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "inet.h"
//#include <arpa/inet.h>

// Forward declarations.
uint32_t message_checksum(const message_t *message);

message_result_t message_init(message_t *message)
{
  memset(message, 0, sizeof(message_t));
  return MESSAGE_SUCCESS;
}

void message_free(message_t *message)
{
  for (size_t i = 0; i < message->length; i++) {
    free(message->tlv[i].value);
  }

  message_init(message);
}

message_result_t message_parse(message_t *message, const uint8_t *data, size_t length)
{
  message_init(message);

  size_t offset = 0;
  while (offset < length) {
    if (message->length >= MAX_TLV_COUNT) {
      message_free(message);
      return MESSAGE_ERROR_TOO_MANY_TLVS;
    }

    size_t i = message->length;

    // Parse type.
    message->tlv[i].type = data[offset];
    offset += sizeof(uint8_t);

    if (offset + sizeof(uint16_t) > length) {
      message_free(message);
      return MESSAGE_ERROR_PARSE_ERROR;
    }

    // Parse length.
    memcpy(&message->tlv[i].length, &data[offset], sizeof(uint16_t));
    message->tlv[i].length = ntohs(message->tlv[i].length);
    offset += sizeof(uint16_t);

    if (offset + message->tlv[i].length > length) {
      message_free(message);
      return MESSAGE_ERROR_PARSE_ERROR;
    }

    // Parse value.
    message->tlv[i].value = (uint8_t*) malloc(message->tlv[i].length);
    if (!message->tlv[i].value) {
      message_free(message);
      return MESSAGE_ERROR_OUT_OF_MEMORY;
    }

    memcpy(message->tlv[i].value, &data[offset], message->tlv[i].length);
    offset += message->tlv[i].length;

    // If this is a checksum TLV, do checksum verification immediately.
    if (message->tlv[i].type == TLV_CHECKSUM) {
      uint32_t checksum = message_checksum(message);
      if (memcmp(&checksum, message->tlv[i].value, sizeof(uint32_t) != 0)) {
        message_free(message);
        return MESSAGE_ERROR_CHECKSUM_MISMATCH;
      }
    }

    message->length++;
  }

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add(message_t *message, uint8_t type, uint16_t length, uint8_t *value)
{
  if (message->length >= MAX_TLV_COUNT) {
    return MESSAGE_ERROR_TOO_MANY_TLVS;
  }

  size_t i = message->length;
  message->tlv[i].value = (uint8_t*) malloc(length);
  if (!message->tlv[i].value) {
    return MESSAGE_ERROR_OUT_OF_MEMORY;
  }

  message->tlv[i].type = type;
  message->tlv[i].length = length;
  memcpy(message->tlv[i].value, value, length);
  message->length++;

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add_command(message_t *message, uint8_t command)
{
  return message_tlv_add(message, TLV_COMMAND, sizeof(uint8_t), (uint8_t*) &command);
}

message_result_t message_tlv_add_checksum(message_t *message)
{
  uint32_t checksum = message_checksum(message);
  return message_tlv_add(message, TLV_CHECKSUM, sizeof(uint32_t), (uint8_t*) &checksum);
}

size_t message_serialize(uint8_t *buffer, size_t length, const message_t *message)
{
  size_t offset = 0;
  size_t remaining_buffer = length;
  for (size_t i = 0; i < message->length; i++) {
    size_t tlv_length = sizeof(uint8_t) + sizeof(uint16_t) + message->tlv[i].length;
    if (remaining_buffer < tlv_length) {
      return MESSAGE_ERROR_BUFFER_TOO_SMALL;
    }

    uint16_t hlength = htons(message->tlv[i].length);

    memcpy(buffer + offset, &message->tlv[i].type, sizeof(uint8_t));
    memcpy(buffer + offset + sizeof(uint8_t), &hlength, sizeof(uint16_t));
    memcpy(buffer + offset + sizeof(uint8_t) + sizeof(uint16_t), message->tlv[i].value, message->tlv[i].length);

    offset += tlv_length;
    remaining_buffer -= tlv_length;
  }

  return offset;
}

void message_print(const message_t *message)
{
  printf("<Message tlvs(%u)=[", (unsigned int) message->length);
  for (size_t i = 0; i < message->length; i++) {
    uint8_t *data = message->tlv[i].value;
    size_t data_length = message->tlv[i].length;

    printf("{%u, \"", message->tlv[i].type);
    for (size_t j = 0; j < data_length; j++) {
      printf("%02X%s", data[j], (j < data_length - 1) ? " " : "");
    }
    printf("\"}%s", (i < message->length - 1) ? "," : "");
  }
  printf("]>");
}

uint32_t message_checksum(const message_t *message)
{
  uint32_t checksum = 0;
  for (size_t i = 0; i < message->length; i++) {
    checksum = crc32(checksum, message->tlv[i].value, message->tlv[i].length);
  }
  return htonl(checksum);
}
