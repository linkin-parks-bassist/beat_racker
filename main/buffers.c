#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "buffers.h"
#include "mic.h"

#define N_BUFFERS 4

sample_t buffers[BUFFER_SIZE][N_BUFFERS];

sample_t *buffer_queue[N_BUFFERS];

int head = 0;
int tail = N_BUFFERS - 1;
SemaphoreHandle_t sem;

int init_buffer_queue()
{
	for (int i = 0; i < N_BUFFERS; i++)
		buffer_queue[i] = buffers[i];
	
	sem = xSemaphoreCreateBinary();
	
	if (sem == NULL)
		return 1;
	
	xSemaphoreGive(sem);
	
	return 0;
}

sample_t *get_buffer()
{
	xSemaphoreTake(sem, portMAX_DELAY);
	
	//printf("Giving buffer. Buffers left: %d\n", (tail < head) ? tail + N_BUFFERS - head : tail - head);
	sample_t *ret_buf = NULL;
	
	if (head != tail)
	{
		ret_buf = buffer_queue[head];
		head = (head + 1) % N_BUFFERS;
	}
	
	xSemaphoreGive(sem);
	return ret_buf;
}

void return_buffer(sample_t *buffer)
{
	xSemaphoreTake(sem, portMAX_DELAY);
	tail = (tail + 1) % N_BUFFERS;
	buffer_queue[tail] = buffer;
	xSemaphoreGive(sem);
}

int buffer_queue_empty()
{
	return head == tail;
}
