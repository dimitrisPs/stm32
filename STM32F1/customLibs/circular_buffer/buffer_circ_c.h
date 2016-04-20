/*
 * buffer_circ_c.h
 *
 *  Created on: Nov 10, 2015
 *      Author: dimitris
 */

#ifndef BUFFER_CIRC_C_H_
#define BUFFER_CIRC_C_H_

#include <stddef.h>
#include <strings.h>
#include <stdbool.h>

#define BUFFER_SIZE	1024
#define OVERWRITE 0 //if set it overites bytes in buffer if is full
//else it discard

typedef enum
{
	BUFFER_EMPTY,
	BUFFER_NOT_EMPTY,
	BUFFER_NOT_FULL,
	BUFFER_FULL,
	BUFFER_WRITE_SUCCESS,
	BUFFER_READ_SUCCESS,
	BUFFER_OVERWRITE_DATA,
	BUFFER_DISCARD_DATA
} BUFFER_STATE;






typedef struct buff
{
	/*the buffer is implemented as a fixed size uint8 array.
	 * the size of the buffer is defined in this header file
	 * as BUFFER_SIZE. there are two indexes to show the from
	 *  it to read (idxRead) and to it to write (idxWrite).
	 *  the read and write to de buffer is implemeted with two
	 *  helper function.
	 *  write to buffer function 	:buffer_circular_write_byte
	 *  read to buffer function 	:buffer_circular_read_byte
	 */
	uint idxWrite,idxRead;
	uint8_t b[BUFFER_SIZE];
	uint numOfLines;
}circularBuffer;


BUFFER_STATE buffer_circular_write_byte(circularBuffer *buffer, uint8_t byte);
BUFFER_STATE buffer_circular_read_byte(circularBuffer *buffer,uint8_t *byte);
void buffer_circular_flush(circularBuffer *buffer);
BUFFER_STATE buffer_circular_write(circularBuffer *buffer,uint8_t *data, uint data_size);
BUFFER_STATE buffer_circular_read(circularBuffer *buffer,uint8_t *data);
BUFFER_STATE buffer_circular_read_line(circularBuffer *buffer,uint8_t *data);
bool is_buffer_empty(circularBuffer *buffer);
bool is_buffer_full(circularBuffer *buffer);
uint buffer_data_size(circularBuffer *buffer);


#endif /* BUFFER_CIRC_C_H_ */
