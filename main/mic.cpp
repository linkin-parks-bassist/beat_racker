#include "driver/i2s.h"
#include "mic.h"

int i2s_install()
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
    
    return 0;
}

int read_from_mic(int32_t *buffer, size_t read_size)
{
	if (!buffer)
		return -1;
	
	int bytes_read;
	int error = i2s_read(I2S_NUM_0, buffer, read_size, &bytes_read, portMAX_DELAY);
	
	if (error == ESP_OK)
		return bytes_read;
	else
		return -2;
}
