#ifndef BEAT_DETECTION_H_
#define BEAT_DETECTION_H_

int init_beat_detection();

void beat_detection_task(void *params);

extern QueueHandle_t data_queue;

#endif
