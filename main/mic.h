#ifndef MIC_H_
#define MIC_H_

#define SAMPLE_RATE 22050
#define I2S_SCK  33
#define I2S_WS   25
#define I2S_SD   32

int init_mic();

void mic_read_task(void *params);

extern QueueHandle_t data_queue;

#endif
