#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <stdio.h>

#include "beat_detection.h"
#include "led_control.h"
#include "buffers.h"
#include "mic.h"

int init()
{
	int error;
	
	if ((error = i2s_install()))
		return error;
	
	if ((error = init_leds()))
		return error;
	
	if ((error = init_beat_detection()))
		return error;
	
	init_buffer_queue();
	
	return 0;
}

void app_main(void)
{
	int error;
	if ((error = init()))
	{
		printf("Error %d\n", error);
	}
	else
	{
		printf("Initialised\n");
		
		xTaskCreatePinnedToCore(beat_detection_task, "beat_detector", 16384, NULL, 5, NULL, 0);
		xTaskCreatePinnedToCore(mic_read_task, 		    "mic_reader",  4096, NULL, 5, NULL, 1);
	}
}
