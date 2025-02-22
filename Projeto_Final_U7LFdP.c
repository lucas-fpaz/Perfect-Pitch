#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/regs/adc.h"

#include <math.h>
#include "kiss_fftr.h"
#include "kiss_fft.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// Pino e canal do microfone no ADC.
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)

#define SAMPLE_RATE 2048  // Taxa de amostragem em Hz (ajustável)
#define SAMPLES 1024   // Tamanho do buffer de captura, Quantidade de amostras lidas

uint16_t adc_buffer[SAMPLES];  // Buffer para armazenar os valores do ADC
int dma_channel;

kiss_fft_scalar in[SAMPLES];
kiss_fft_cpx out[SAMPLES / 2 + 1];

void sample_mic();

void compute_fft() {
    kiss_fftr_cfg cfg = kiss_fftr_alloc(SAMPLES, 0, NULL, NULL);
    for (int i = 0; i < SAMPLES; i++) {
        in[i] = (adc_buffer[i] - 2048) / 2048.0; // Normaliza os valores do ADC
    }
    kiss_fftr(cfg, in, out);
    free(cfg);
}

float find_peak_frequency() {
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

dma_channel_config setup_dma(uint dma_channel) {
    
    adc_gpio_init(MIC_PIN);  // Configura GPIO26 (ADC0) para entrada analógica
    adc_init();  // Inicializa o ADC
    adc_select_input(MIC_CHANNEL);  // Seleciona canal ADC 0 (GPIO26)

    adc_fifo_setup(
        true,  // Habilita o FIFO do ADC
        true,  // Habilita a solicitação de dados para o DMA
        1,     // Número de amostras antes do FIFO acionar o DMA
        false, // Não ativa alertas de erro
        false   // Armazena os valores como 16 bits
    );

    adc_set_clkdiv(48000000 / SAMPLE_RATE);  // Define a taxa de amostragem do ADC

    // Captura um canal livre do DMA
    dma_channel = dma_claim_unused_channel(true);

    // Configura o DMA
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel);
    
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16); // Transferência de 16 bits
    channel_config_set_read_increment(&cfg, false); // Mantém o mesmo endereço de leitura (registrador ADC)
    channel_config_set_write_increment(&cfg, true); // Escreve sequencialmente no buffer
    channel_config_set_dreq(&cfg, DREQ_ADC); // Sincroniza com o ADC

    return cfg;
}

int main()
{
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
    //setup_dma();
    
    // inicialização I2C, com frequencia 400Khz.
    i2c_init(I2C_PORT, 400*1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    stdio_init_all();

    dma_channel_config cfg = setup_dma(dma_channel);

    while (true) {
        sample_mic(cfg);
        for(int i = 0; i < SAMPLES; i++){
            printf("%d\n", adc_buffer[i]);
        }
        compute_fft();
        float peak_freq = find_peak_frequency();
        printf("Frequência de pico: %.2f Hz\n", peak_freq);
        sleep_ms(5000);
    }
}

/**
 * Realiza as leituras do ADC e armazena os valores no buffer.
 */
void sample_mic(dma_channel_config cfg) {
    adc_fifo_drain(); // Limpa o FIFO do ADC.
    adc_run(false); // Desliga o ADC (se estiver ligado) para configurar o DMA.

        dma_channel_configure(
        dma_channel,       // Canal do DMA
        &cfg,              // Configuração do canal
        adc_buffer,        // Destino (buffer na RAM)
        &adc_hw->fifo,     // Fonte (registrador do ADC)
        SAMPLES,       // Quantidade de amostras por captura
        true               // Inicia imediatamente
    );

    // Liga o ADC e espera acabar a leitura.
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    
    // Acabou a leitura, desliga o ADC de novo.
    adc_run(false);
  }