#include "frame.h"
#include "message.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>


void frame_parser(uint8_t *buffer, uint8_t length, message_t *msg){
	parser_state_t state = SERIAL_STATE_WAIT_START;
	uint8_t message[100];
	int msg_count = 0;
	for (size_t i = 0; i < length; i++){
		switch (state) {
			case SERIAL_STATE_WAIT_START: {
				// Waiting for frame start marker.
				if (buffer[i] == FRAME_MARKER_ESCAPE) {
					state = SERIAL_STATE_WAIT_START_ESCAPE;
				} else if (buffer[i] == FRAME_MARKER_START) {
					state = SERIAL_STATE_IN_FRAME;
				}
				break;
			}
			case SERIAL_STATE_WAIT_START_ESCAPE: {
				state = SERIAL_STATE_WAIT_START;
				break;
			}
			case SERIAL_STATE_IN_FRAME: {
				if (buffer[i] == FRAME_MARKER_ESCAPE) {
					// Next byte is part of frame content.
					state = SERIAL_STATE_AFTER_ESCAPE;
				} else if (buffer[i] == FRAME_MARKER_START) {
					// Encountered start marker while parsing frame, resynchronize.
					length = 0;
				} else if (buffer[i] == FRAME_MARKER_END) {
					// End of frame.
					message_result_t result = message_parse(msg, message, msg_count);
					if (result == MESSAGE_SUCCESS) {
						printf("Parsed protocol message: ");
						message_print(msg);
						printf("\n");
					} else {
						printf("Failed to parse serialized message: %d\n", result);
					}
					length = 0;
					state = SERIAL_STATE_WAIT_START;
				} else {
					// Frame content.
					message[msg_count++] = buffer[i];
					//frame_parser_add_to_frame(parser, byte);
				}
				break;
			}
			case SERIAL_STATE_AFTER_ESCAPE: {
				message[msg_count++] = buffer[i];
				state = SERIAL_STATE_IN_FRAME;
				break;
			}
		}//end switch
	}//end for
}//end function


ssize_t frame_message(uint8_t *frame, size_t length, const message_t *message){
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
};

