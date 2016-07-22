#ifndef KORUZA_DRIVER_FRAME_H
#define KORUZA_DRIVER_FRAME_H

#include "message.h"

/* Uncomment to get debug messages in the UART2 terminal*/
//#define DEBUG_MODE


/**
 * Parser states.
 */
typedef enum {
  SERIAL_STATE_WAIT_START,
  SERIAL_STATE_WAIT_START_ESCAPE,
  SERIAL_STATE_IN_FRAME,
  SERIAL_STATE_AFTER_ESCAPE,
} parser_state_t;

#define FRAME_MAX_LENGTH 131070
#define FRAME_MARKER_START 0xF1
#define FRAME_MARKER_END 0xF2
#define FRAME_MARKER_ESCAPE 0xF3


/**
 * Parses received buffer
 *
 * @param frame buffer
 * @param message data
 */
void frame_parser(uint8_t *buffer, uint8_t length, message_t *msg);

/**
 * Frames the given message.
 *
 * @param frame Destination buffer
 * @param length Destination buffer length
 * @param message Message to frame
 * @return Size of the output frame
 */
ssize_t frame_message(uint8_t *frame, size_t length, const message_t *message);

#endif
