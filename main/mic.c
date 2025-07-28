#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <assert.h>

#include "driver/i2s.h"

#include "beat_detection.h"
#include "buffers.h"
#include "mic.h"

QueueHandle_t data_queue;

static const TickType_t buffer_duration_ticks = (const TickType_t)((BUFFER_DURATION_SEC * 1000) / portTICK_PERIOD_MS);

int init_mic()
{
    i2s_config_t i2s_config = {
            .mode                   = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate            = SAMPLE_RATE,
            .bits_per_sample        = I2S_BITS_PER_SAMPLE_32BIT,
            .channel_format         = I2S_CHANNEL_FMT_ONLY_LEFT,
            .communication_format   = I2S_COMM_FORMAT_I2S_MSB,
            .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count          = 8,
            .dma_buf_len            = 64,
            .use_apll               = false,
            .tx_desc_auto_clear     = false,
            .fixed_mclk             = 0
        };

    // Set up I2S pin configuration
    i2s_pin_config_t pin_config = {
            .bck_io_num   = I2S_SCK,
            .ws_io_num    = I2S_WS,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num  = I2S_SD
        };

    // Install and start I2S driver
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    
    // Initialise data queue
    data_queue = xQueueCreate(N_BUFFERS, sizeof(sample_t*));
    
    return 0;
}

int read_from_mic(sample_t *buffer)
{
	assert(buffer != NULL);
	
	size_t bytes_read;
	int error = i2s_read(I2S_NUM_0, buffer, BUFFER_SIZE * sizeof(sample_t), &bytes_read, portMAX_DELAY);
	
	if (error == ESP_OK)
		return (int)bytes_read;
	else
		return -1;
}

void mic_read_task(void *params)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	sample_t *buffer;
	
	while (true)
	{
		do {buffer = get_buffer();} while (!buffer);
		read_from_mic(buffer);
		xQueueSend(data_queue, &buffer, buffer_duration_ticks);
		vTaskDelayUntil(&last_wake_time, buffer_duration_ticks);
	}
}
