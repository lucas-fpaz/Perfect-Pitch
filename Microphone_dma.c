#include "Microphone_dma.h"


void compute_fft(uint16_t adc_buffer[SAMPLES], kiss_fft_scalar in[SAMPLES], kiss_fft_cpx out[SAMPLES / 2 + 1]) {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    for (int i = 0; i < SAMPLES; i++) {
        in[i] = (adc_buffer[i] - 2048) / 2048.0;
    }
    kiss_fftr(cfg, in, out);
    free(cfg);
}

float find_peak_frequency(kiss_fft_cpx out[SAMPLES / 2 + 1]) {
    float max_magnitude = 0;
    int peak_index = 0;
    for (int i = 1; i < SAMPLES / 2; i++) {
        float magnitude = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            peak_index = i;
        }
    }
    return (peak_index * SAMPLE_RATE) / SAMPLES;
}

dma_channel_config setup_dma(int dma_channel) {
    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);

    adc_fifo_setup(
        true,  // Habilita o FIFO do ADC
        true,  // Habilita a solicitação de dados para o DMA
        1,     // Número de amostras antes do FIFO acionar o DMA
        false, // Não ativa alertas de erro
        false   // Armazena os valores como 16 bits
    );

    adc_set_clkdiv(48000000 / SAMPLE_RATE);

    dma_channel = dma_claim_unused_channel(true); 
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    return cfg;
}

void sample_mic(dma_channel_config cfg, uint16_t adc_buffer[SAMPLES], int dma_channel) {
    adc_fifo_drain();
    adc_run(false);

    dma_channel_configure(
        dma_channel,  
        &cfg,
        adc_buffer,
        &adc_hw->fifo,
        SAMPLES,
        true
    );

    sleep_us(10);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    adc_run(false);
    //for (int i = 0; i < SAMPLES; i++) {
    //    printf("ADC[%d] = %d\n", i, adc_buffer[i]);
    //}
}

