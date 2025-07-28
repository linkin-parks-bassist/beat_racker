#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <stdio.h>

#include "beat_detection.h"
#include "led_control.h"
#include "buffers.h"
#include "mic.h"

// Run  all initialisation routines and report any errors
int init()
{
	int error;
	
	if ((error = init_mic()))
		return error;
	
	if ((error = init_leds()))
		return error;
	
	if ((error = init_buffer_queue()))
		return error;
	
	return 0;
}

void app_main(void)
{
	int error;
	if ((error = init()))
	{
		// If there is an error, report it and die
		printf("Error %d\n", error);
	}
	else
	{
		// If there is no error, report succesful initialisation
		printf("Initialised\n");
		
		// Spin off tasks to paralelly get data from mic and run beat detection
		xTaskCreatePinnedToCore(beat_detection_task, "beat_detector", 16384, NULL, 5, NULL, 0);
		xTaskCreatePinnedToCore(mic_read_task, 		    "mic_reader",  4096, NULL, 5, NULL, 1);
	}
}
