/*
 * buffer_cir_c.c
 *
 *  Created on: Nov 10, 2015
 *      Author: dimitris
 */
#include "buffer_circ_c.h"

/* write a byte to a circular buffer. if there is a data overflow it discards new byte
 * or it overwrite previous the oldest byte in the buffer.
 * @inputs 	*buffer	: a pointer to buffer struct in which you want to write
 * 			byte 	:the byte you want to be written.
 * @outputs	BUFFER_STATE	:the outcome of the function dictates
 * 							if the data is written,(return BUFFER_WRITE_SUCCESS)
 * 							if it overwrote previous data(return BUFFER_OVERWRITE_DATA)
 * 							if new byte was discarded(return BUFFER_DISCARD_DATA)
 */
BUFFER_STATE buffer_circular_write_byte(circularBuffer *buffer, uint8_t byte)
{
	if ((buffer->idxWrite==buffer->idxRead-1) || ((buffer->idxWrite==(BUFFER_SIZE-1)) && (buffer->idxRead==0)))
	{

		if (OVERWRITE)
		{
			/*in this case we will overwrite the oldest element of the buffer by moving
			 * the read pointer to make space
			 */
			buffer->b[buffer->idxWrite]=byte;
			/* if the buffer index you want to increase is already the last element of the buffer,
			 * give the index the value 0 to start from the beginning, else just increase its value
			 */
			buffer->idxWrite=(buffer->idxWrite==(BUFFER_SIZE-1)) ? 0 : buffer->idxWrite+1;
			buffer->idxRead=(buffer->idxRead==(BUFFER_SIZE-1)) ? 0 : buffer->idxRead+1;
			return BUFFER_OVERWRITE_DATA;
		}
		else
		{
			/*discard data and return BUFFER_DISCARD_DATA to inform the user */
			return BUFFER_DISCARD_DATA;
		}
	}
	else
	{
		/*the buffer is not full. in this case we just write the byte and increase the index for
		 * the next byte to be written
		 */
		buffer->b[buffer->idxWrite]=byte;
		/* if the buffer index you want to increase is already the last element of the buffer,
		 * give the index the value 0 to start from the beginning, else just increase its value
		 */
		if (buffer->idxWrite==10)
		{
			buffer->idxWrite=10;
			buffer->idxWrite=10;
		}
		buffer->idxWrite=(buffer->idxWrite==(BUFFER_SIZE-1)) ? 0 : buffer->idxWrite+1;
		return BUFFER_WRITE_SUCCESS;
	}
}

/* read a byte from a circular buffer.if the buffer is empty returns BUFFER_EMPTY
 * @inputs 	*buffer	: a pointer to buffer struct from which you want to read
 * 			*byte 	:a pointer to byte which, with the completion of the function will
 * 					have the value of the byte you read.
 * @outputs	BUFFER_STATE	:the outcome of the function dictates
 * 							if read was success ,(return BUFFER_READ_SUCCESS)
 * 							if buffer was empty(return BUFFER_EMPTY)
 */
BUFFER_STATE buffer_circular_read_byte(circularBuffer *buffer,uint8_t *byte)
{

	if (buffer->idxWrite!=buffer->idxRead)
	{	/*if the buffer is not empty read the byte and then move the read index on place.
		 *if the buffer index you want to increase is already the last element of the buffer,
		 *give the index the value 0 to start from the beginning, else just increase its value
		 */
		*byte=buffer->b[buffer->idxRead];
		buffer->idxRead=(buffer->idxRead==(BUFFER_SIZE-1)) ? 0 : buffer->idxRead+1;
		return BUFFER_READ_SUCCESS;
	}
	else
		/*the buffer is empty return BUFFER_EMPTY */
		return BUFFER_EMPTY;
}

/*this function empties the buffer. to do that it resets write and read pointer to 0.
 * It should be called write after a new buffer is made.
 * @inputs *buffer: a pointer to the buffer you want to flush
 * @outputs none
 */
void buffer_circular_flush(circularBuffer *buffer)
{
	buffer->idxRead=0;
	buffer->idxWrite=0;
}

/* write to a circular buffer. if the available space in the buffer is not enough for
 * all the input data to be written, the function exits without write anything.
 * @inputs 	*buffer	:a pointer to buffer struct to  which you want to write
 * 			*data 	:a pointer to uint8_t array with the data you want to write to buffer
 * 			data_size: the amount of data you need to write
 * @outputs	BUFFER_STATE	:the outcome of the function dictates
 * 							if write was success ,(return BUFFER_WRITE_SUCCESS)
 * 							if buffer's available space is not enough(return BUFFER_FULL)
 */
BUFFER_STATE buffer_circular_write(circularBuffer *buffer,uint8_t *data, uint data_size)
{
	uint buffer_space;
	uint counter=data_size;
	/*the function has to determine at first if there is enough available space in the buffer
	 * for data array to be written.to to that , first we find the difference between the
	 * write and read index. if this difference is not negative that means that the read
	 * pointer follows the write pointer and so the available space is the buffer
	 * size (-)this difference. if the difference is negative that means that the write
	 *  index has already reach the end of the buffer and has been reseted to count from the beginning.
	 *  in this case the available space is just the difference between the two indexes
	 */
	int diff=buffer->idxWrite-buffer->idxRead;
	if (diff>=0)
	{
		buffer_space=BUFFER_SIZE-diff;
	}
	else
	{
		buffer_space=-diff;
	}
	/*if there is enough space in the buffer to write the the array, we white each byte
	 * with buffer_circular_write_byte() function
	 */
	if (buffer_space>data_size)
	{
		while(counter)
		{
			buffer_circular_write_byte(buffer,*data);
			data++;
			counter--;
		}
		/*after we complete all the writes, the function returns BUFFER_WRITE_SUCCESS to indicate success*/
		return BUFFER_WRITE_SUCCESS;
	}
	/*if there was not enough space in the buffer the function return BUFFER_FULL to indicate failure*/
	return BUFFER_FULL;

}

/* read all the contents from a circular buffer.if the buffer is empty returns BUFFER_EMPTY
 * @inputs 	*buffer	: a pointer to buffer struct from which you want to read
 * 			*data 	:a pointer to byte which, with the completion of the function will
 * 					have the value of the byte you read.
 * @outputs	BUFFER_STATE	:the outcome of the function dictates
 * 							if read was success ,(return BUFFER_READ_SUCCESS)
 * 							if buffer was empty(return BUFFER_EMPTY)
 */
BUFFER_STATE buffer_circular_read(circularBuffer *buffer,uint8_t *data)
{
	int diff;
	int data_size;
	if (buffer->idxWrite==buffer->idxRead)
	{
		/*empty buffer, there is nothing to read and should return BUFFER_EMPTY.*/
		return BUFFER_EMPTY;
	}
	else
	{
		/*if there are data in the buffer, first we see the amount of data we have to read*/
		diff=buffer->idxWrite-buffer->idxRead;
		if (diff>0)
		{
			/*because diff cumputed with reference to idxwrite, which indicates the cell
			 *to write data (not a cell with data already written) the size of data is diff
			 */
			data_size=diff;
		}
		else
		{
			data_size=BUFFER_SIZE+diff;
		}
	}
	/*read data_s0ze bytes from the buffer with the help of buffer_circular_read_byte() function
	 * it each iteration move the pointer and the counter accordingly.
	 */
	while(data_size)
	{
		buffer_circular_read_byte(buffer,data);
		data++;
		data_size--;
	}
	/*add as the last element the null element */
	data[data_size]=0;
	return BUFFER_READ_SUCCESS;
}

BUFFER_STATE buffer_circular_read_line(circularBuffer *buffer,uint8_t *data)
{
	int diff;
	int data_idx;

	if (buffer->numOfLines==0)
		return BUFFER_EMPTY;
	if (buffer->idxWrite==buffer->idxRead)
	{
		/*empty buffer, there is nothing to read and should return BUFFER_EMPTY.*/
		return BUFFER_EMPTY;
	}
	do
	{
		buffer_circular_read_byte(buffer,data);
		data++;
	}while(*(data-1)!='\n');
	*(data-1)=0;
	buffer->numOfLines--;
	return BUFFER_READ_SUCCESS;
}

bool is_buffer_empty(circularBuffer *buffer)
{
	return (buffer->idxRead==buffer->idxWrite)?true:false;
}

bool is_buffer_full(circularBuffer *buffer)
{
	if ((buffer->idxWrite==buffer->idxRead-1) || ((buffer->idxWrite==(BUFFER_SIZE-1)) && (buffer->idxRead==0)))
	{
		return true;
	}
	return false;
}

uint buffer_data_size(circularBuffer *buffer)
{
	int diff;
	diff=buffer->idxWrite-buffer->idxRead;
	if (diff>0)
	{
		/*because diff cumputed with reference to idxwrite, which indicates the cell
		 *to write data (not a cell with data already written) the size of data is diff
		 */
		return diff;
	}
	else
	{
		return BUFFER_SIZE+diff;
	}
	return 0;
}


