idf_component_register(SRCS "main.c"
							"mic.c"
							"beat_detection.c"
							"led_control.c"
							"buffers.c"
                    INCLUDE_DIRS "." "../components/kiss_fft130")
