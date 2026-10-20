#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"             // relógio do chip (para medir o tempo sem travar o programa)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"           // pinos digitais (botão)
#include "driver/ledc.h"           // PWM (LED)
#include "esp_adc/adc_oneshot.h"   // ADC (potenciômetro)

// ---------------- Pinos (conforme o circuito da atividade) ----------------
#define PINO_LED    13             // LED (com resistor de 220 ohms)
#define PINO_BOTAO  9              // Botão (com pull-up externo de 10k)
#define CANAL_POT   ADC_CHANNEL_4  // GPIO5 no ESP32-S3 = ADC1, canal 4 (potenciômetro)

// ---------------- Parâmetros ----------------
#define ADC_MAX        4095        // 12 bits: 0 a 4095
#define ADC_MIN_CAL    2700           // leitura bruta com o pot no mínimo (resistor para o GND impede de chegar a 0)
#define ADC_MAX_CAL    4095        // leitura bruta com o pot no máximo (ajuste se não chegar a 4095)
#define PWM_MAX        1023        // 10 bits: 0 a 1023
#define TENSAO_MAX_MV  3300        // 3,3 V em milivolts
#define FREQ_PWM_HZ    5000        // 5 kHz
#define DEBOUNCE_MS    30          // botão precisa ficar estável por 30 ms
#define INTERVALO_PRINT_MS 500     // telemetria a cada 500 ms

static const char *TAG = "ATIVIDADE9";

void app_main(void)
{
    // =============== 1) Configura o ADC ===============
    adc_oneshot_unit_handle_t adc;                       // "controle remoto" do ADC
    adc_oneshot_unit_init_cfg_t cfg_unidade = {
        .unit_id = ADC_UNIT_1,                           // usamos a unidade 1 do ADC
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg_unidade, &adc));

    adc_oneshot_chan_cfg_t cfg_canal = {
        .atten = ADC_ATTEN_DB_12,                        // atenuação 12 dB: lê até ~3,3 V
        .bitwidth = ADC_BITWIDTH_12,                     // resolução de 12 bits (0 a 4095)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, CANAL_POT, &cfg_canal));

    // =============== 2) Configura o PWM (LEDC) ===============
    // O "timer" define a velocidade (frequência) e a precisão (resolução) do PWM
    ledc_timer_config_t cfg_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,               // o ESP32-S3 só tem modo "low speed"
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,            // 10 bits (0 a 1023)
        .freq_hz = FREQ_PWM_HZ,                          // 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&cfg_timer));

    // O "canal" liga o timer a um pino (o do LED)
    ledc_channel_config_t cfg_pwm = {
        .gpio_num = PINO_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,                                       // começa apagado
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg_pwm));

    // =============== 3) Configura o botão ===============
    gpio_config_t cfg_botao = {
        .pin_bit_mask = 1ULL << PINO_BOTAO,
        .mode = GPIO_MODE_INPUT,                         // o pino só lê
        .pull_up_en = GPIO_PULLUP_DISABLE,               // o pull-up já existe no circuito (10k)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg_botao));

    // =============== Variáveis do programa ===============
    bool modo_hold = false;          // false = LIVE, true = HOLD
    int ultima_leitura = 1;          // última leitura crua do botão (1 = solto)
    int estado_estavel = 1;          // estado do botão já "limpo" do bounce
    int64_t t_mudanca = 0;           // quando o botão mudou de valor pela última vez (ms)
    int64_t t_ultimo_print = 0;      // quando imprimimos pela última vez (ms)

    ESP_LOGI(TAG, "Iniciado em modo LIVE");

    while (1) {
        int64_t agora = esp_timer_get_time() / 1000;    // tempo atual em milissegundos

        // botao com debounce
        int leitura = gpio_get_level(PINO_BOTAO);       // 1 = solto, 0 = apertado
        if (leitura != ultima_leitura) {                // o botão mexeu (pode ser bounce)
            ultima_leitura = leitura;
            t_mudanca = agora;                          // reinicia a contagem de estabilidade
        }
        // Só aceita a mudança se o valor ficou parado por DEBOUNCE_MS
        if ((agora - t_mudanca) >= DEBOUNCE_MS && leitura != estado_estavel) {
            estado_estavel = leitura;
            if (estado_estavel == 0) {                  // 0 = acabou de ser apertado
                modo_hold = !modo_hold;                 // alterna LIVE <-> HOLD
            }
        }

        // ler adc
        int bruto = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, CANAL_POT, &bruto));   // valor de 0 a 4095

        // compensa o resistor: remapeia [ADC_MIN_CAL, ADC_MAX_CAL] -> [0, ADC_MAX]
        if (bruto < ADC_MIN_CAL) bruto = ADC_MIN_CAL;
        if (bruto > ADC_MAX_CAL) bruto = ADC_MAX_CAL;
        bruto = (bruto - ADC_MIN_CAL) * ADC_MAX / (ADC_MAX_CAL - ADC_MIN_CAL);

        // atualiza o PWM do LED (só se não estiver em modo fixo)
        if (!modo_hold) {
            int duty = bruto * PWM_MAX / ADC_MAX;       // regra de três: 0..4095 -> 0..1023
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);   // guarda o novo brilho
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);      // aplica no LED
        }
        // No modo fixo ele fica no último brilho

        if ((agora - t_ultimo_print) >= INTERVALO_PRINT_MS) {
            t_ultimo_print = agora;
            int tensao_mv = bruto * TENSAO_MAX_MV / ADC_MAX;   // VmV = Vraw * 3300 / 4095
            ESP_LOGI(TAG, "ADC bruto: %d | Tensao: %d mV | Estado: %s",
                     bruto, tensao_mv, modo_hold ? "HOLD" : "LIVE");
        }

        // para nao disparar watchdog
        vTaskDelay(1);
    }
}
