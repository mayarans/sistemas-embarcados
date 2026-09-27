#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#define BOTAO_GPIO GPIO_NUM_5
#define LED_GPIO GPIO_NUM_16
#define TEMPO_DEBOUNCE_MS 50
#define TEMPO_CONTAGEM_MS 30000
#define TEMPO_PRESSAO_LONGA_MS 2000

// botao com pull-up externo: pressionado = 0
#define NIVEL_PRESSIONADO 0

const char *TAG = "botao";

typedef enum {
    EVENTO_NENHUM,
    EVENTO_PRESSIONOU,
    EVENTO_SOLTOU,
} evento_botao_t;

inline int64_t agora_ms(void) {
    return esp_timer_get_time() / 1000;
}

void configura_gpio(void) {
    gpio_reset_pin(BOTAO_GPIO);
    gpio_set_direction(BOTAO_GPIO, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(BOTAO_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
}

// Debounce por polling, sem atraso: a leitura só é aceita depois de ficar
// estável por TEMPO_DEBOUNCE_MS. Retorna um evento a cada borda confirmada.
evento_botao_t le_botao(void) {
    static int ultima_leitura = !NIVEL_PRESSIONADO;
    static int estado_estavel = !NIVEL_PRESSIONADO;
    static int64_t instante_mudanca_ms = 0;

    int leitura = gpio_get_level(BOTAO_GPIO);
    int64_t agora = agora_ms();

    if (leitura != ultima_leitura) {
        ultima_leitura = leitura;
        instante_mudanca_ms = agora;   // reinicia a janela de debounce
    }

    if (leitura != estado_estavel && (agora - instante_mudanca_ms) >= TEMPO_DEBOUNCE_MS) {
        estado_estavel = leitura;
        return (estado_estavel == NIVEL_PRESSIONADO) ? EVENTO_PRESSIONOU : EVENTO_SOLTOU;
    }
    return EVENTO_NENHUM;
}

void app_main(void)
{
    configura_gpio();

    // O laço abaixo nunca cede a CPU (sem vTaskDelay), então a task IDLE da CPU0
    // não roda; monitora só a IDLE da CPU1 para evitar o aviso do watchdog.
    esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = (1 << 1),
        .trigger_panic = false,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_cfg));

    bool led_aceso = false;
    int64_t inicio_contagem_ms = 0;

    bool botao_segurado = false;     // botão está pressionado (já com debounce)
    bool pressao_tratada = false;    // o pressionamento atual já foi consumido (acendeu o LED ou desligou)
    int64_t inicio_pressao_ms = 0;

    while (1) {
        evento_botao_t evento = le_botao();
        int64_t agora = agora_ms();

        if (evento == EVENTO_PRESSIONOU) {
            botao_segurado = true;
            inicio_pressao_ms = agora;

            // Condição 1: LED apagado + botão pressionado -> acende o LED e inicia a contagem de 30 s.
            if (!led_aceso) {
                gpio_set_level(LED_GPIO, 1);
                led_aceso = true;
                inicio_contagem_ms = agora;
                pressao_tratada = true;   // este pressionamento não deve virar reinício nem desligamento
                ESP_LOGI(TAG, "Condição 1: LED aceso, contagem de %d s iniciada", TEMPO_CONTAGEM_MS / 1000);
            } else {
                pressao_tratada = false;  // decide ao soltar (toque curto) ou ao passar de 2 s (longo)
            }
        }

        // Condição 3: pressão longa com o LED aceso -> desligamento imediato.
        if (botao_segurado && !pressao_tratada && led_aceso &&
            (agora - inicio_pressao_ms) >= TEMPO_PRESSAO_LONGA_MS) {
            gpio_set_level(LED_GPIO, 0);
            led_aceso = false;
            pressao_tratada = true;
            ESP_LOGI(TAG, "Condição 3: pressão longa, LED apagado");
        }

        if (evento == EVENTO_SOLTOU) {
            botao_segurado = false;

            // Condição 2: toque curto com o LED aceso -> reinicia os 30 s sem apagar o LED.
            if (!pressao_tratada && led_aceso) {
                inicio_contagem_ms = agora;
                ESP_LOGI(TAG, "Condição 2: contagem reiniciada para %d s", TEMPO_CONTAGEM_MS / 1000);
            }
            pressao_tratada = false;
        }

        // Fim da contagem: apaga o LED.
        if (led_aceso && (agora - inicio_contagem_ms) >= TEMPO_CONTAGEM_MS) {
            gpio_set_level(LED_GPIO, 0);
            led_aceso = false;
            ESP_LOGI(TAG, "Contagem de %d s finalizada: LED apagado", TEMPO_CONTAGEM_MS / 1000);
        }
    }
}
