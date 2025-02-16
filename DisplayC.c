#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "pico/bootrom.h"

#include "inc/ssd1306.h"    // Header do display
#include "inc/font.h"       // Fontes (usado pela lib)
#include "build/pio_matrix.pio.h" // Arquivo .pio compilado (WS2812)

//
// DEFINIÇÕES DE PINOS
//
#define I2C_PORT        i2c1
#define I2C_SDA         14
#define I2C_SCL         15
#define OLED_ADDR       0x3C
#define WIDTH           128
#define HEIGHT          64

// LEDs e botões
#define LED_G           11  // LED Verde
#define LED_B           12  // LED Azul  (usaremos PWM)
#define LED_R           13  // LED Vermelho (usaremos PWM)
#define BTN_A           5
#define BTN_B           6   // Se quiser usar para algo
#define JOYSTICK_BTN    22  // Botão do joystick

// Joystick (ADC)
#define JOYSTICK_X_ADC  26  // ADC0
#define JOYSTICK_Y_ADC  27  // ADC1
#define ADC_MAX         4095
#define ADC_CENTER      2048

// Debounce
static volatile uint32_t ultimo_tempo_a  = 0;
static volatile uint32_t ultimo_tempo_js = 0;
const  uint32_t debounce_delay_us        = 200000; // 200 ms

// Variáveis globais
static volatile bool  ledG_state      = false;  // LED Verde ON/OFF
static volatile bool  g_pwm_active    = true;   // se PWM (R e B) está ativo
static volatile int   g_border_style  = 0;       // Estilo da borda (caso queira usar)

// Display
static ssd1306_t ssd;

// -------------------------------------------------
// Funções Auxiliares
// -------------------------------------------------

/**
 * Converte R,G,B (0..1) em formato 24 bits (para WS2812).
 */
static uint32_t matrix_rgb(double r, double g, double b) {
    double brilho = 0.03;  // Ajuste de brilho (3%)
    unsigned char R_ = (unsigned char)(r * 255 * brilho);
    unsigned char G_ = (unsigned char)(g * 255 * brilho);
    unsigned char B_ = (unsigned char)(b * 255 * brilho);

    return ((uint32_t)G_ << 24)
         | ((uint32_t)R_ << 16)
         | ((uint32_t)B_ <<  8);
}

// Estilos de borda
#define BORDER_NONE    0  // Sem borda
#define BORDER_SOLID   1  // Borda sólida
#define BORDER_DOTTED  2  // Borda pontilhada
#define BORDER_DOUBLE  3  // Borda dupla

static void draw_border(int style) {
    switch (style) {
        case BORDER_SOLID:
            // Borda sólida
            ssd1306_rect(&ssd, 0, 1, WIDTH, 1, true, true);           // Topo
            ssd1306_rect(&ssd, 63, 1, WIDTH, 1, true, true);          // Base
            ssd1306_rect(&ssd, 0, WIDTH-2, 1, HEIGHT, true, true);    // Esquerda
            ssd1306_rect(&ssd, WIDTH, 0, 1, HEIGHT, true, true);      // Direita
            break;
        case BORDER_DOTTED:
            // Borda pontilhada
            for (int i = 0; i < WIDTH; i += 2) {
                ssd1306_rect(&ssd, i, 0, 1, 1, true, true);               // Topo
                ssd1306_rect(&ssd, i, HEIGHT*2 - 1, 1, 1, true, true);    // Base
            }
            for (int i = 0; i < HEIGHT*2; i += 2) {
                ssd1306_rect(&ssd, 0, i, 1, 1, true, true);               // Esquerda
                ssd1306_rect(&ssd, WIDTH - 1, i, 1, 1, true, true);       // Direita
            }
            break;
        case BORDER_DOUBLE:
            // Borda dupla
            ssd1306_rect(&ssd, 0, 0, WIDTH, 1, true, true);               // Topo (externa)
            ssd1306_rect(&ssd, 63, 0, WIDTH, 1, true, true);      // Base (externa)
            ssd1306_rect(&ssd, 0, 0, 1, HEIGHT, true, true);              // Esquerda (externa)
            ssd1306_rect(&ssd, 0, 127, 1, HEIGHT, true, true);      // Direita (externa)

            ssd1306_rect(&ssd, 2, 2, WIDTH - 4, 1, true, true);           // Topo (interna)
            ssd1306_rect(&ssd, 61, 0, WIDTH - 4, 1, true, true);  // Base (interna)
            ssd1306_rect(&ssd, 2, 2, 1, HEIGHT - 4, true, true);          // Esquerda (interna)
            ssd1306_rect(&ssd, 2, 125, 1, HEIGHT - 4, true, true);  // Direita (interna)
            break;
        default:
            // Sem borda (BORDER_NONE)
            break;
    }
}

/**
 * Mapeia 'val' de [in_min..in_max] para [out_min..out_max].
 */
static int map_value(int val, int in_min, int in_max, int out_min, int out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Inicializa PWM para 16 bits (wrap=65535) em um pino.
 */
static uint pwm_init_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 65535); // 16 bits
    pwm_init(slice_num, &cfg, true);

    // Duty inicial
    uint channel = pwm_gpio_to_channel(gpio);
    pwm_set_chan_level(slice_num, channel, 0);

    return slice_num;
}

/**
 * Define duty (0..65535) no pino, respeitando se PWM está ativo.
 */
static void pwm_set_duty(uint gpio, uint16_t duty) {
    if (!g_pwm_active && (gpio == LED_R || gpio == LED_B)) {
        duty = 0; // Se PWM inativo, força 0
    }
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint channel   = pwm_gpio_to_channel(gpio);
    pwm_set_chan_level(slice_num, channel, duty);
}

// -------------------------------------------------
// DISPLAY
// -------------------------------------------------

/**
 * Desenha um quadrado 8x8 na posição (sx, sy) - preenchido (BRANCO).
 */
static void draw_square(int sx, int sy) {
    // ssd1306_rect(&ssd, x1, y1, x2, y2, outline=false, color=true)
    // false = não é só contorno => preenchido
    // true  = branco
    ssd1306_rect(&ssd, sx, sy, 8, 8, true, true); 
}


// Função para atualizar o display com o quadrado e a borda
static void update_display(int sx, int sy, int border_style) {
    ssd1306_fill(&ssd, false);   // Limpa tudo (preto)
    draw_square(sx, sy);         // Desenha o quadrado branco
    draw_border(border_style);   // Desenha a borda
    ssd1306_send_data(&ssd);     // Envia ao display
}

// -------------------------------------------------
// INTERRUPÇÕES (Botões)
// -------------------------------------------------

// Função de callback para o botão do joystick
static void gpio_callback(uint gpio, uint32_t events) {
    uint32_t agora = time_us_32();

    // Botão A => Toggle PWM
    if (gpio == BTN_A) {
        if ((agora - ultimo_tempo_a) > debounce_delay_us) {
            ultimo_tempo_a = agora;
            g_pwm_active = !g_pwm_active;
            printf("[BTN_A] PWM agora = %s\n", g_pwm_active ? "ON" : "OFF");
        }
    }
    // Botão do Joystick => Toggle LED Verde e alternar estilo da borda
    else if (gpio == JOYSTICK_BTN) {
        if ((agora - ultimo_tempo_js) > debounce_delay_us) {
            ultimo_tempo_js = agora;

            // Alterna o estado do LED Verde
            ledG_state = !ledG_state;
            gpio_put(LED_G, ledG_state);

            // Alterna o estilo da borda
            g_border_style = (g_border_style + 1) % 4; // Alterna entre 0, 1, 2, 3
            printf("[JS_BTN] LED_G=%d, Borda=%d\n", ledG_state, g_border_style);
        }
    }
}

// -------------------------------------------------
// MAIN
// -------------------------------------------------

int main() {
    stdio_init_all();

    // 1) Inicializa I2C e SSD1306
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, OLED_ADDR, I2C_PORT);
    ssd1306_config(&ssd);
    // Limpa o display no início
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // 2) Matriz WS2812 (caso queira exibir números digitados)
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pio_matrix_program);

    // 3) LEDs (G -> digital, R/B -> PWM)
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_put(LED_G, ledG_state);

    pwm_init_pin(LED_R);
    pwm_init_pin(LED_B);

    // 4) Botões
    //  Botão A (GPIO5)
    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    //  Botão Joystick (GPIO22)
    gpio_init(JOYSTICK_BTN);
    gpio_set_dir(JOYSTICK_BTN, GPIO_IN);
    gpio_pull_up(JOYSTICK_BTN);
    gpio_set_irq_enabled(JOYSTICK_BTN, GPIO_IRQ_EDGE_FALL, true);

    // 5) Joystick (ADC) - X=26(ADC0), Y=27(ADC1)
    adc_init();
    adc_gpio_init(JOYSTICK_X_ADC);
    adc_gpio_init(JOYSTICK_Y_ADC);

    // Posição inicial do quadrado (centralizado)
    int square_x = (HEIGHT); // (128 - 8)/2 = 60
    int square_y = (WIDTH); // (64 - 8)/2 = 28

    // Loop principal
    while (true) {
        // 1) Lê joystick (X e Y)
        adc_select_input(0); // X
        uint16_t x_adc = adc_read();
        adc_select_input(1); // Y
        uint16_t y_adc = adc_read();

        // 2) Ajusta PWM R/B conforme distância do centro
        int diff_x = (x_adc > ADC_CENTER) ? (x_adc - ADC_CENTER) : (ADC_CENTER - x_adc);
        int diff_y = (y_adc > ADC_CENTER) ? (y_adc - ADC_CENTER) : (ADC_CENTER - y_adc);
        uint32_t duty_r = diff_x * 32;  // 2048*32=65536
        uint32_t duty_b = diff_y * 32;  
        if (duty_r > 65535) duty_r = 65535;
        if (duty_b > 65535) duty_b = 65535;
        pwm_set_duty(LED_R, (uint16_t)duty_r);
        pwm_set_duty(LED_B, (uint16_t)duty_b);

        // 3) Move quadrado
        square_x = map_value(x_adc, ADC_MAX, 0, 0, HEIGHT - 8);   // Eixo X controla horizontal (0 a 120)
        square_y = map_value(y_adc, ADC_MAX, 0, WIDTH - 8, 0); 

        // 4) Atualiza Display (desenha o quadrado e a borda)
        update_display(square_x, square_y, g_border_style);

        // Pequena pausa
        sleep_ms(40);
    }

    return 0;
}
