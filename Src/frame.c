/*
 * frame.c
 *
 *  Created on: 14. jul. 2016
 *      Author: vojis
 */

#include "frame.h"

#include <stdlib.h>

void frame_parser_add_to_frame(parser_t *parser, uint8_t byte);

void frame_parser_init(parser_t *parser)
{
  parser->handler = NULL;
  parser->state = SERIAL_STATE_WAIT_START;
  parser->length = 0;
  parser->buffer_size = 1024;
  parser->buffer = (uint8_t*) malloc(parser->buffer_size);
  if (!parser->buffer) {
    // Out of memory abort.
    abort();
  }
}

void frame_parser_free(parser_t *parser)
{
  parser->state = SERIAL_STATE_WAIT_START;
  parser->length = 0;
  parser->buffer_size = 0;
  free(parser->buffer);
}

void frame_parser_push_buffer(parser_t *parser, uint8_t *buffer, size_t length)
{
  for (size_t i = 0; i < length; i++) {
    frame_parser_push_byte(parser, buffer[i]);
  }
}

void frame_parser_add_to_frame(parser_t *parser, uint8_t byte)
{
  // If there is already too many bytes in the buffer, discard them and resync.
  if (parser->length > FRAME_MAX_LENGTH) {
    parser->state = SERIAL_STATE_WAIT_START;
    parser->length = 0;
    return;
  }

  // Increase buffer when needed.
  if (parser->length == parser->buffer_size) {
    parser->buffer_size += 1024;
    parser->buffer = (uint8_t*) realloc(parser->buffer, parser->buffer_size);
    if (!parser->buffer) {
      // Out of memory abort.
      abort();
    }
  }

  parser->buffer[parser->length] = byte;
  parser->length++;
}

void frame_parser_push_byte(parser_t *parser, uint8_t byte)
{
  switch (parser->state) {
    case SERIAL_STATE_WAIT_START: {
      // Waiting for frame start marker.
      if (byte == FRAME_MARKER_ESCAPE) {
        parser->state = SERIAL_STATE_WAIT_START_ESCAPE;
      } else if (byte == FRAME_MARKER_START) {
        parser->state = SERIAL_STATE_IN_FRAME;
      }
      break;
    }
    case SERIAL_STATE_WAIT_START_ESCAPE: {
      parser->state = SERIAL_STATE_WAIT_START;
      break;
    }
    case SERIAL_STATE_IN_FRAME: {
      if (byte == FRAME_MARKER_ESCAPE) {
        // Next byte is part of frame content.
        parser->state = SERIAL_STATE_AFTER_ESCAPE;
      } else if (byte == FRAME_MARKER_START) {
        // Encountered start marker while parsing frame, resynchronize.
        parser->length = 0;
      } else if (byte == FRAME_MARKER_END) {
        // End of frame.
        if (parser->handler != NULL) {
          message_t message;
          message_init(&message);
          if (message_parse(&message, parser->buffer, parser->length) == MESSAGE_SUCCESS) {
            parser->handler(&message);
          }
          message_free(&message);
        }

        parser->length = 0;
        parser->state = SERIAL_STATE_WAIT_START;
      } else {
        // Frame content.
        frame_parser_add_to_frame(parser, byte);
      }
      break;
    }
    case SERIAL_STATE_AFTER_ESCAPE: {
      frame_parser_add_to_frame(parser, byte);
      parser->state = SERIAL_STATE_IN_FRAME;
      break;
    }
  }
}

ssize_t frame_message(uint8_t *frame, size_t length, const message_t *message)
{
  // First check if there is not enough space in the output buffer. This is
  // an optimistic estimate (assumes no escaping is needed).
  size_t buffer_size = message_serialized_size(message);
  if (length < buffer_size + 2) {
    return -1;
  }

  // Serialize message into buffer.
  uint8_t *buffer = (uint8_t*) malloc(buffer_size);
  if (!buffer) {
    abort();
  }

  ssize_t result = message_serialize(buffer, buffer_size, message);
  if (result != buffer_size) {
    free(buffer);
    return -1;
  }

  // Frame the message, inserting escape markers when needed.
  size_t index = 0;
  frame[index++] = FRAME_MARKER_START;
  for (size_t i = 0; i < buffer_size; i++) {
    if (index >= length) {
      free(buffer);
      return -1;
    }

    // Escape frame markers.
    if (buffer[i] == FRAME_MARKER_START ||
        buffer[i] == FRAME_MARKER_END ||
        buffer[i] == FRAME_MARKER_ESCAPE) {
      frame[index++] = FRAME_MARKER_ESCAPE;
    }

    frame[index++] = buffer[i];
  }
  frame[index++] = FRAME_MARKER_END;

  free(buffer);
  return index;
}
