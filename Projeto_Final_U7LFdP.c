#include "hardware/i2c.h"

#include <stdio.h>
#include "Microphone_dma.h"  // Isso já inclui os outros headers indiretamente

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define LCD_ADDR 0x3C 

#define BTN_A 5
#define BTN_B 6

#define JOYSTICK_Y 26
#define ADC_JOYSTICK 0

#define BUZZER_A 10
#define BUZZER_B 21

#define DURATION 1000

// Tempo de debounce em microssegundos (10ms)
#define DEBOUNCE_DELAY 10000
volatile uint64_t last_press_time_a = 0;
volatile uint64_t last_press_time_b = 0;


uint16_t adc_buffer[SAMPLES];  // Definição real das variáveis
int dma_channel; // Definição real da variável global

kiss_fft_scalar in[SAMPLES];
kiss_fft_cpx out[SAMPLES / 2 + 1];

// Estrutura para armazenar as notas
typedef struct {
    int oitava;       // Número da oitava
    char *nota_br;    // Nome da nota (Brasil)
    char *nota_us;    // Nome da nota (EUA)
    int freq;         // Frequência arredondada (inteira)
} Nota;

// Array com as notas principais para cantores (E2 a A5)
const Nota notas[] = {
    {2, "Mi",  "E2",  82}, 
    {2, "Fa",  "F2",  87}, 
    {2, "Sol", "G2",  98}, 
    {2, "La",  "A2",  110}, 
    {2, "Si",  "B2",  123}, 

    {3, "Do",  "C3",  131}, 
    {3, "Re",  "D3",  147}, 
    {3, "Mi",  "E3",  165}, 
    {3, "Fa",  "F3",  175}, 
    {3, "Sol", "G3",  196}, 
    {3, "La",  "A3",  220}, 
    {3, "Si",  "B3",  247}, 

    {4, "Do",  "C4",  262}, 
    {4, "Re",  "D4",  294}, 
    {4, "Mi",  "E4",  330}, 
    {4, "Fa",  "F4",  349}, 
    {4, "Sol", "G4",  392}, 
    {4, "La",  "A4",  440}, 
    {4, "Si",  "B4",  494}, 

    {5, "Do",  "C5",  523}, 
    {5, "Re",  "D5",  587}, 
    {5, "Mi",  "E5",  659}, 
    {5, "Fa",  "F5",  698}, 
    {5, "Sol", "G5",  784}, 
    {5, "La",  "A5",  880}  
};

const int num_notas = sizeof(notas) / sizeof(Nota);

enum estados {
    MENU,
    NOTA,
    VOZ
}estado_atual;

// Função de debounce para botões
bool debounce(uint gpio, volatile uint64_t *last_press_time) {
    uint64_t now = time_us_64();
    if (now - *last_press_time > DEBOUNCE_DELAY) {
        *last_press_time = now;
        return true;
    }
    return false;
}

// Interrupção do botão
void button_handler(uint gpio, uint32_t events) {
    if (gpio == BTN_A){
        if (((estado_atual == VOZ) || (estado_atual == MENU)) && debounce(gpio, &last_press_time_a)){
            estado_atual = NOTA;
        }

    } else if (gpio == BTN_B){

        if (((estado_atual == VOZ) || (estado_atual == NOTA)) && debounce(gpio, &last_press_time_b)){
            estado_atual = MENU;
        }
    }
}

void lcd_print_nota(int index) {
    static int last_index = -1; // Variável persistente para lembrar a última nota desenhada

    if (index >= 0 && index < num_notas && index != last_index) { // Só atualiza se a nota mudar
        char buffer[32];
        uint8_t ssd1306_buffer[ssd1306_buffer_length];

        memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer)); // Limpa buffer apenas se necessário
        struct render_area area = {0, 127, 0, 7}; 
        calculate_render_area_buffer_length(&area);

        // Exibe a oitava
        sprintf(buffer, "%dº oitava", notas[index].oitava);
        ssd1306_draw_string(ssd1306_buffer, 0, 0, buffer);

        // Exibe a nota em ambas as notações
        sprintf(buffer, "Nota: %s (%s)", notas[index].nota_br, notas[index].nota_us);
        ssd1306_draw_string(ssd1306_buffer, 0, 20, buffer);

        // Exibe a frequência
        sprintf(buffer, "Frequencia: %d Hz", notas[index].freq);
        ssd1306_draw_string(ssd1306_buffer, 0, 40, buffer);

        render_on_display(ssd1306_buffer, &area); // Renderiza no display

        last_index = index; // Atualiza a última nota desenhada
    }
}

void lcd_feedback(int index, uint peak_freq) {
    static int last_index = -1; // Variável persistente para lembrar a última nota desenhada

    if (peak_freq != last_index) { // Só atualiza se a nota mudar
        char buffer[32];
        uint8_t ssd1306_buffer[ssd1306_buffer_length];

        memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer)); // Limpa buffer apenas se necessário
        struct render_area area = {0, 127, 0, 7}; 
        calculate_render_area_buffer_length(&area);

        // Exibe a nota em ambas as notações
        sprintf(buffer, "Nota: %s (%s)", notas[index].nota_br, notas[index].nota_us);
        ssd1306_draw_string(ssd1306_buffer, 0, 0, buffer);

        // Exibe a frequência
        sprintf(buffer, "Frequencia: %d Hz", notas[index].freq);
        ssd1306_draw_string(ssd1306_buffer, 0, 20, buffer);

        // Exibe a frequência detectada
        sprintf(buffer, "Sua Voz: %d Hz", peak_freq);
        ssd1306_draw_string(ssd1306_buffer, 0, 40, buffer);

        render_on_display(ssd1306_buffer, &area); // Renderiza no display

        last_index = peak_freq; // Atualiza a última nota desenhada
    }
}

void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta divisor de clock
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0); // Desliga o PWM inicialmente
}

void play_tone(uint pinA, uint pinB, uint frequency) {

    uint slice_numA = pwm_gpio_to_slice_num(pinA);
    uint slice_numB = pwm_gpio_to_slice_num(pinB);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t top = clock_freq / frequency - 1;

    pwm_set_wrap(slice_numA, top);
    pwm_set_gpio_level(pinA, top / 2); // 50% de duty cycle
    pwm_set_wrap(slice_numB, top);
    pwm_set_gpio_level(pinB, top / 2); // 50% de duty cycle

    sleep_ms(DURATION);

    pwm_set_gpio_level(pinA, 0); // Desliga o som após a duração
    pwm_set_gpio_level(pinB, 0); // Desliga o som após a duração
    sleep_ms(50); // Pausa entre notas
}
/**
 * @brief Função para controlar o LED RGB com PWM
 * @param pinA Pino do LED
 * @param duty_cycle Ciclo de trabalho (0 a 100)
 */
void led_pwm(uint pinA, float duty_cycle) {

    uint slice_numA = pwm_gpio_to_slice_num(pinA);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t top = clock_freq / 1 - 1;

    pwm_set_wrap(slice_numA, top);
    pwm_set_gpio_level(pinA, top * (duty_cycle/100)); //duty cycle
}

uint read_Joystick(int note) {
    static bool joystick_moved = false; // Guarda se já detectamos um movimento

    const uint16_t CENTER = 2048;  // Valor médio do ADC (joystick no centro)
    const uint16_t DEADZONE = 500; // Margem para ignorar pequenas variações

    // Pausa temporariamente a coleta de áudio via DMA
    dma_channel_abort(MIC_CHANNEL);

    // Faz a leitura do ADC do joystick
    adc_select_input(ADC_JOYSTICK);

    uint16_t adc_value = adc_read();  // Lê o valor do ADC (0 a 4095)

    // Se o joystick for para cima e ainda não detectamos movimento
    if (adc_value > CENTER + DEADZONE && !joystick_moved) {
        note++;
        if (note > (num_notas) - 1) note = 0; // Se passar do limite, volta ao início
        joystick_moved = true; // Marca que já movimentamos
    } 
    // Se o joystick for para baixo e ainda não detectamos movimento
    else if (adc_value < CENTER - DEADZONE && !joystick_moved) {
        note--;
        if (note < 0) note = (num_notas) - 1; // Se passar do início, volta ao fim
        joystick_moved = true; // Marca que já movimentamos
    } 
    // Se o joystick voltou ao centro, resetamos o estado para permitir novo movimento
    else if (adc_value >= CENTER - DEADZONE && adc_value <= CENTER + DEADZONE) {
        joystick_moved = false;
    }
    //printf("Joystick ADC: %d | Nota: %d\n", adc_value, note);
    sleep_ms(10);
    return note;
}

void note_feedback(float freq_detectada, int nota_alvo) {
    // Definição dos pinos do LED RGB
    #define LED_R 13
    #define LED_G 11
    #define LED_B 12

    // Obtém a frequência esperada para a nota alvo
    int freq_certa = notas[nota_alvo].freq;

    // Calcula a diferença absoluta entre a frequência detectada e a nota esperada
    float diferenca = fabs(freq_detectada - freq_certa);
    float porcentagem_diferenca = (diferenca / freq_certa) * 100;

    // Lógica para determinar a cor do LED
    if (diferenca <= 2) {  
        // 💚 Verde: Está na frequência correta (±2Hz)
        gpio_put(LED_R, 0);
        gpio_put(LED_G, 1);
        gpio_put(LED_B, 0);
    } 
    else if (diferenca <= 10) {  
        // 💛 Amarelo: Diferença menor que 10Hz
        gpio_put(LED_R, 1);
        gpio_put(LED_G, 1);
        gpio_put(LED_B, 0);
    } 
    else if (diferenca <= (notas[nota_alvo + 1].freq - freq_certa) || diferenca <= (freq_certa - notas[nota_alvo - 1].freq)) {  
        // 🟠 Laranja: Está na nota vizinha (acima ou abaixo)
        gpio_put(LED_R, 1);
        gpio_put(LED_G, 1);
        //led_pwm(LED_G, 10);
        gpio_put(LED_B, 0);
    } 
    else {  
        // 🔴 Vermelho: Muito fora da nota
        gpio_put(LED_R, 1);
        gpio_put(LED_G, 0);
        gpio_put(LED_B, 0);
    }
}

void setup(){

    estado_atual = MENU;

    stdio_init_all();

    pwm_init_buzzer(BUZZER_A);
    pwm_init_buzzer(BUZZER_B);

    // Inicialização I2C, com frequência 400Khz.
    i2c_init(I2C_PORT, 400*1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL); 

    // Configura os pinos como saída
    gpio_init(LED_R);
    gpio_init(LED_G);
    gpio_init(LED_B);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_set_dir(LED_B, GPIO_OUT);

    // Reseta o LED para apagado
    gpio_put(LED_R, 0);
    gpio_put(LED_G, 0);
    gpio_put(LED_B, 0);

    sleep_ms(2000); // Aguarda a USB conectar


    adc_init();
    adc_gpio_init(JOYSTICK_Y);  // Configura o pino do ADC

    // Configuração dos botões como entradas pull-up
    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &button_handler);

    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);
    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, &button_handler);



        // Processo de inicialização completo do OLED SSD1306
        ssd1306_init();

        // Preparar área de renderização para o display (ssd1306_width pixels por ssd1306_n_pages páginas)
        struct render_area frame_area = {
            start_column : 0,
            end_column : ssd1306_width - 1,
            start_page : 0,
            end_page : ssd1306_n_pages - 1
        };
    
        calculate_render_area_buffer_length(&frame_area);
    
        // zera o display inteiro
        uint8_t ssd[ssd1306_buffer_length];
        memset(ssd, 0, ssd1306_buffer_length);
        render_on_display(ssd, &frame_area);
    
    restart:
    
        ssd1306_draw_line(ssd, 10, 10, 100, 50, true);
        render_on_display(ssd, &frame_area);
        lcd_print_nota(0);
}

int main()
{ 
    setup();
    dma_channel_config cfg = setup_dma(dma_channel);
    uint nota_atual = 0;
    estado_atual = MENU;
    while (true) {

        switch(estado_atual){
            case MENU:
                //printf("Menu\n");
                gpio_put(LED_R, 0);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);
                lcd_print_nota(nota_atual);
                nota_atual = read_Joystick(nota_atual);
                break;
            case NOTA:
                //printf("Nota\n");
                play_tone(BUZZER_A, BUZZER_B,notas[nota_atual].freq);
                estado_atual = VOZ;
                break;
            case VOZ:
                // Pausa temporariamente a coleta de áudio via DMA
                dma_channel_abort(ADC_JOYSTICK);
                // Faz a leitura do ADC do joystick
                adc_select_input(MIC_CHANNEL);
                sample_mic(cfg, adc_buffer, dma_channel);
                compute_fft(adc_buffer, in, out);
                float peak_freq = find_peak_frequency(out);
                //printf("Frequência de pico: %.2f Hz\n", peak_freq);
                note_feedback(peak_freq, nota_atual);
                lcd_feedback(nota_atual, (uint)peak_freq);
                break;
        }   
    }
}
