#ifndef BUFFERS_H_
#define BUFFERS_H_

#define BUFFER_SIZE 512

#define BUFFER_DURATION_SEC (double)BUFFER_SIZE / (double)SAMPLE_RATE

typedef int32_t sample_t;

int init_buffer_queue();
sample_t *get_buffer();
void return_buffer(sample_t *buffer);
int buffer_queue_empty();

#endif
