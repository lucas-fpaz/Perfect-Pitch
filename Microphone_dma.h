#ifndef MICROPHONE_DMA_H
#define MICROPHONE_DMA_H

#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"

#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)
#define SAMPLE_RATE 2048
#define SAMPLES 4096

// Declaração das funções
void compute_fft(uint16_t adc_buffer[SAMPLES], kiss_fft_scalar in[SAMPLES], kiss_fft_cpx out[SAMPLES / 2 + 1]);
float find_peak_frequency(kiss_fft_cpx out[SAMPLES / 2 + 1]);
dma_channel_config setup_dma(int dma_channel);
void sample_mic(dma_channel_config cfg, uint16_t adc_buffer[SAMPLES], int dma_channel);

#endif
