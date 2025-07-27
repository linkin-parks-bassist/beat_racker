#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#define NORM_FACTOR (1.0 / 8388608.0)

#include "kiss_fft.h"

#include "led_control.h"
#include "buffers.h"
#include "mic.h"

#define N_SPECTRA 2
#define PI 3.14159265

#define ACTIVATION_THRESHOLD 100.0

#define UNBEAT_THRESHOLD 0.9

#define RUNNING_AVG_ADAPT_RATE (1 - BUFFER_DURATION_SEC)
#define RUNNING_AVG_WAIT_ADAPT_RATE   0.6
#define RUNNING_AVG_FFT_ADAPT_RATE    0.5
#define BEAT_THRESHOLD_ADAPT_RATE     0.7
#define BEAT_THRESHOLD_SENSITISE_RATE 0.9

#define MIN_BEAT_TIME   0 //60.0/210.0

const double inv_erf_1 = 1.0/erf(1.0);

double beat_threshold = 1.3;

double buffer_energy_running_avg = 0;
double running_avg_wait = 1;

double seconds_since_last_beat = 0;

int beat_detected(double *buffer);

double frame[BUFFER_SIZE];

double spectra[N_SPECTRA][BUFFER_SIZE];
double freq_weights[BUFFER_SIZE / 2];
double ra_spectrum[BUFFER_SIZE / 2];
double ra_spectrum_magnitude = 0;
double weighted_spectral_magnitudes[N_SPECTRA];
int current_spectrum_index = 0;
int prev_spectrum_index = N_SPECTRA - 1;

QueueHandle_t data_queue;

kiss_fft_cfg kiss_fft_config;
kiss_fft_cpx* fft_in;
kiss_fft_cpx* fft_out;

double window[BUFFER_SIZE];

#define sqr(x) (x * x)

void init_hamming_window()
{
	double N;
	
	N = (double) (BUFFER_SIZE - 1);
	
	for (int n = 0; n < BUFFER_SIZE; n++)
		window[n] = 0.54 - (0.46 * cos (2 * PI * ((double)n / N)));
}

void init_freq_weights()
{
	double frequency;
	double norm = 0;
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
	{
		frequency = (double)(i * SAMPLE_RATE) / (double)BUFFER_SIZE;
		freq_weights[i] = 2.0 * exp(-sqr(fabs(frequency - 80) /  30)) + exp(-sqr(fabs(frequency - (double)SAMPLE_RATE / 2.0) /  2000));
		norm += sqr(freq_weights[i]);
	}
	
	norm = 1.0 / sqrt(norm);
	
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
		freq_weights[i] *= norm;
}

int init_beat_detection()
{
	data_queue = xQueueCreate(10, sizeof(sample_t*));
	
	if (!data_queue)
		return 5;
	
	fft_in  = malloc(sizeof(kiss_fft_cpx) * BUFFER_SIZE);
    fft_out = malloc(sizeof(kiss_fft_cpx) * BUFFER_SIZE);
    kiss_fft_config = kiss_fft_alloc(BUFFER_SIZE, 0, 0, 0);
    
    init_hamming_window();
    init_freq_weights();
    
    for (int j = 0; j < N_SPECTRA; j++)
		weighted_spectral_magnitudes[j] = 0.0;
    
    for (int i = 0; i < BUFFER_SIZE / 2; i++)
    {
		ra_spectrum[i] = 0.0;
		
		for (int j = 0; j < N_SPECTRA; j++)
			spectra[j][i] = 0.0;
	}
	
	return 0;
}

int copy_and_convert_buffer(int32_t *buffer)
{
	if (!buffer)
		return 1;
	
	for (int i = 0; i < BUFFER_SIZE; i++)
		frame[i] = (double)buffer[i] * NORM_FACTOR;
	
	return 0;
}

void beat_detection_task(void *params)
{
	printf("Beat detection task begin\n");
	
	printf("BTrack object created\n");
	
	sample_t *buffer;
	int64_t beat_detection_duration;
	int64_t queue_duration;
	int64_t loop_start;
	
	printf("Entering beat detection loop\n");
	while (true)
	{
		loop_start = esp_timer_get_time();
		xQueueReceive(data_queue, &buffer, portMAX_DELAY);
		queue_duration = esp_timer_get_time() - loop_start;
		
		copy_and_convert_buffer(buffer);
		
		return_buffer(buffer);
		
		/*btrack_start = esp_timer_get_time();
		b.processAudioFrame(frame);
		btrack_duration = esp_timer_get_time() - btrack_start;*/
		
		beat_detection_duration = esp_timer_get_time();
		if (beat_detected(frame))
		{
			beat_detection_duration = esp_timer_get_time();
			//printf("Beat detected !\n");
			swap_leds();
		}
		else
		{
			beat_detection_duration = esp_timer_get_time() - beat_detection_duration;
		}
		
		//printf("Beat detection loop finished. xQueueReceive duration: %lld. beat detection: %lld. Total duration: %lld\n", queue_duration, beat_detection_duration, esp_timer_get_time() - loop_start);
	}
}

double bin_frequency(int bin)
{
	return ((double)bin * SAMPLE_RATE) / (double)BUFFER_SIZE;
}

void perform_fft(double *buffer)
{
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
    {
        fft_in[i].r = buffer[i + BUFFER_SIZE / 2] * window[i + BUFFER_SIZE / 2];
        fft_in[i].i = 0.0;
        fft_in[i + BUFFER_SIZE / 2].r = buffer[i] * window[i];
        fft_in[i + BUFFER_SIZE / 2].i = 0.0;
    }
    
	kiss_fft(kiss_fft_config, fft_in, fft_out);
	
	current_spectrum_index = (current_spectrum_index + 1) % N_SPECTRA;
	prev_spectrum_index    = (prev_spectrum_index    + 1) % N_SPECTRA;
	
	for (int i = 0; i < BUFFER_SIZE; i++)
		spectra[current_spectrum_index][i] = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
}

double buffer_energy(double *buffer)
{
    double energy = 0, weighted_inner_product;
    double amp_weight, freq_weight, frequency;
    double weighted_spectral_angle = 0;
    
    ra_spectrum_magnitude = 0;
    weighted_spectral_magnitudes[current_spectrum_index] = 0;
    
    for (int i = 0; i < BUFFER_SIZE/2; i++)
    {
        frequency = ((double)i * SAMPLE_RATE) / (double)BUFFER_SIZE;
        amp_weight  = 1.5 * exp(-(fabs(frequency -   65) /  50) * (fabs(frequency -   65) /  50));
        amp_weight += 0.3 * exp(-(fabs(frequency -  200) /  80) * (fabs(frequency -  200) /  80));
        amp_weight += 0.1 * exp(-(fabs(frequency - 1200) / 600) * (fabs(frequency - 1200) / 600));
        
        energy += amp_weight * spectra[current_spectrum_index][i];
        
        weighted_spectral_magnitudes[current_spectrum_index] += freq_weights[i] * spectra[current_spectrum_index][i] * spectra[current_spectrum_index][i];
        ra_spectrum_magnitude  += freq_weights[i] * sqr(ra_spectrum[i]);
	}
	
	weighted_spectral_magnitudes[current_spectrum_index] = sqrt(weighted_spectral_magnitudes[current_spectrum_index]);
	ra_spectrum_magnitude = sqrt(ra_spectrum_magnitude);
	
	weighted_inner_product = 0;
	
	if (weighted_spectral_magnitudes[current_spectrum_index] > 0.01 && ra_spectrum_magnitude > 0.01)
	{
		for (int i = 0; i < BUFFER_SIZE/2; i++)
		{
			weighted_inner_product += freq_weights[i] * spectra[current_spectrum_index][i] * ra_spectrum[i];
		}
		
		weighted_spectral_angle = acos(weighted_inner_product / (weighted_spectral_magnitudes[current_spectrum_index] * ra_spectrum_magnitude));
		
		while (weighted_spectral_angle < 0)
			weighted_spectral_angle += PI/2;
	
		while (weighted_spectral_angle > PI/2)
			weighted_spectral_angle -= PI/2;
	}
	else
	{
		weighted_spectral_angle = 0.0;
	}
	
	/*printf("acos(<current, prev> / ||current||||prev||) = acos(%f / %f * %f) = acos(%f) = %f. WSA = %f ",
		weighted_inner_product, weighted_spectral_magnitudes[current_spectrum_index], weighted_spectral_magnitudes[prev_spectrum_index],
		weighted_inner_product / (weighted_spectral_magnitudes[current_spectrum_index] * weighted_spectral_magnitudes[prev_spectrum_index]),
		acos(weighted_inner_product / (weighted_spectral_magnitudes[current_spectrum_index] * weighted_spectral_magnitudes[prev_spectrum_index])),
		weighted_spectral_angle);*/
	
	printf("%f \t%f \t", energy, 100.0 * weighted_spectral_angle);

    return energy * (1 + weighted_spectral_angle / (PI / 8.0));
}

double new_running_avg_wait(double beat_wait)
{
	double ratio = beat_wait / running_avg_wait;
	double s = sin(2 * PI / ratio);
	
	return running_avg_wait * ((s / 10 * PI) + 1.0 + (0.5 / PI) * (ratio - 1));
}

void update_running_avg_spectrum()
{
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
	{
		ra_spectrum[i] = ra_spectrum[i] * RUNNING_AVG_FFT_ADAPT_RATE + spectra[current_spectrum_index][i] * (1 - RUNNING_AVG_FFT_ADAPT_RATE);
	}
}

int beat_detected(double *buffer)
{
    int result;
    
    perform_fft(buffer);
    
	seconds_since_last_beat += BUFFER_DURATION_SEC;

	double energy = buffer_energy(buffer);
	double time_weighting = inv_erf_1 * erf(seconds_since_last_beat / running_avg_wait);
	double factor;
	
	time_weighting *= time_weighting * time_weighting;
	printf("%f ", energy * time_weighting / beat_threshold);
	printf("\n");
	
	if (buffer_energy_running_avg == 0)
	{
		buffer_energy_running_avg = energy;
	}
	else if (buffer_energy_running_avg > ACTIVATION_THRESHOLD || energy > ACTIVATION_THRESHOLD)
	{
		active = 1;
	}
	else
	{
		active = 0;
	}

	buffer_energy_running_avg = buffer_energy_running_avg * RUNNING_AVG_ADAPT_RATE + energy * (1 - RUNNING_AVG_ADAPT_RATE);
    
    result = (time_weighting * energy) / buffer_energy_running_avg > beat_threshold;
    
    if (result)
    {
		//running_avg_wait = new_running_avg_wait(seconds_since_last_beat);
		running_avg_wait = running_avg_wait * RUNNING_AVG_WAIT_ADAPT_RATE + (0.025 * erf(2.0 * (seconds_since_last_beat/running_avg_wait - 1.0)) + running_avg_wait) * (1 - RUNNING_AVG_WAIT_ADAPT_RATE);
		
		seconds_since_last_beat = 0;
		beat_threshold = beat_threshold * BEAT_THRESHOLD_ADAPT_RATE + (1 - BEAT_THRESHOLD_ADAPT_RATE) * (energy / buffer_energy_running_avg);
	}
	else if (seconds_since_last_beat > MIN_BEAT_TIME)
	{
		beat_threshold = ((beat_threshold - 1) * BEAT_THRESHOLD_SENSITISE_RATE) + 1;
	}
	
	update_running_avg_spectrum();
	
	return result;
}
