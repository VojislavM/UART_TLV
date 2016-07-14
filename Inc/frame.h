/*
 * frame.h
 *
 *  Created on: 14. jul. 2016
 *      Author: vojis
 */

#ifndef FRAME_H_
#define FRAME_H_

#include "message.h"

/**
 * Handler for messages received in frames. After the message is handled, it will
 * be freed, so no external references should be kept.
 */
typedef void (*frame_message_handler)(const message_t *message);

/**
 * Parser states.
 */
typedef enum {
  SERIAL_STATE_WAIT_START = 0,
  SERIAL_STATE_WAIT_START_ESCAPE,
  SERIAL_STATE_IN_FRAME,
  SERIAL_STATE_AFTER_ESCAPE,
} parser_state_t;

#define FRAME_MAX_LENGTH 131070
#define FRAME_MARKER_START 0xF1
#define FRAME_MARKER_END 0xF2
#define FRAME_MARKER_ESCAPE 0xF3

/**
 * Frame parser.
 */
typedef struct {
  /// Handler that will be used to emit parsed messages.
  frame_message_handler handler;

  // Internal parser state.
  parser_state_t state;
  uint8_t *buffer;
  size_t buffer_size;
  size_t length;
} parser_t;

/**
 * Initializes the frame parser.
 *
 * @param parser Parser instance
 */
void frame_parser_init(parser_t *parser);

/**
 * Frees the frame parser.
 *
 * @param parser Parser instance
 */
void frame_parser_free(parser_t *parser);

/**
 * Pushes a buffer to the frame parser.
 *
 * @param parser Parser instance
 * @param buffer Buffer to push
 * @param length Size of the buffer
 */
void frame_parser_push_buffer(parser_t *parser, uint8_t *buffer, size_t length);

/**
 * Pushes a single byte to the frame parser.
 *
 * @param parser Parser instance
 * @param byte Byte to push
 */
void frame_parser_push_byte(parser_t *parser, uint8_t byte);

/**
 * Frames the given message.
 *
 * @param frame Destination buffer
 * @param length Destination buffer length
 * @param message Message to frame
 * @return Size of the output frame
 */
ssize_t frame_message(uint8_t *frame, size_t length, const message_t *message);



#endif /* FRAME_H_ */
