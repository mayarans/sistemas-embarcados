#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define BOTAO_GPIO              GPIO_NUM_5
#define LED_GPIO                GPIO_NUM_16
#define TEMPO_DEBOUNCE_MS       50
#define TEMPO_CONTAGEM_US       10000000
#define TEMPO_PRESSAO_LONGA_US   2000000

#define NIVEL_PRESSIONADO 0

static const char *TAG = "botao";

typedef enum {
    EVENTO_NENHUM,
    EVENTO_PRESSIONOU,
    EVENTO_SOLTOU,
} evento_botao_t;

static esp_timer_handle_t timer_contagem;
static esp_timer_handle_t timer_pressao_longa;

static volatile bool    borda_detectada       = false;
static volatile int64_t tempo_ultima_borda_ms = 0;

static volatile bool contagem_expirou       = false;
static volatile bool pressao_longa_disparou = false;

static void IRAM_ATTR isr_botao(void *arg) {
    borda_detectada       = true;
    tempo_ultima_borda_ms = esp_timer_get_time() / 1000;
}

evento_botao_t le_botao(void) {
    static int estado_anterior = !NIVEL_PRESSIONADO;

    if (!borda_detectada) return EVENTO_NENHUM;

    int64_t agora = esp_timer_get_time() / 1000;
    if ((agora - tempo_ultima_borda_ms) < TEMPO_DEBOUNCE_MS) return EVENTO_NENHUM;

    borda_detectada = false;
    int estado_atual = gpio_get_level(BOTAO_GPIO);

    if (estado_atual == estado_anterior) return EVENTO_NENHUM;
    estado_anterior = estado_atual;

    return (estado_atual == NIVEL_PRESSIONADO) ? EVENTO_PRESSIONOU : EVENTO_SOLTOU;
}

static void cb_contagem(void *arg) {
    gpio_set_level(LED_GPIO, 0);
    contagem_expirou = true;
}

static void cb_pressao_longa(void *arg) {
    esp_timer_stop(timer_contagem);
    gpio_set_level(LED_GPIO, 0);
    pressao_longa_disparou = true;
}

void configura_gpio(void) {
    gpio_reset_pin(BOTAO_GPIO);
    gpio_set_direction(BOTAO_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(BOTAO_GPIO, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_GPIO, isr_botao, NULL);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
}

void configura_timers(void) {
    esp_timer_create_args_t args = {.callback = cb_contagem, .name = "contagem"};
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer_contagem));

    args.callback = cb_pressao_longa;
    args.name     = "pressao_longa";
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer_pressao_longa));
}

void app_main(void) {
    configura_gpio();
    configura_timers();

    bool led_aceso             = false;
    bool primeiro_acionamento  = false;
    bool pressao_longa_ocorreu = false;

    while (1) {
        if (pressao_longa_disparou) {
            pressao_longa_disparou = false;
            led_aceso              = false;
            pressao_longa_ocorreu  = true;
            ESP_LOGI(TAG, "Condição 3: pressão longa, LED apagado");
        }

        if (contagem_expirou) {
            contagem_expirou = false;
            led_aceso        = false;
            ESP_LOGI(TAG, "Contagem de 10 s finalizada: LED apagado");
        }

        evento_botao_t evento = le_botao();

        if (evento == EVENTO_PRESSIONOU) {
            pressao_longa_ocorreu = false;
            if (!led_aceso) {
                gpio_set_level(LED_GPIO, 1);
                led_aceso            = true;
                esp_timer_stop(timer_contagem);
                esp_timer_start_once(timer_contagem, TEMPO_CONTAGEM_US);
                primeiro_acionamento = true;
                ESP_LOGI(TAG, "Condição 1: LED aceso, contagem de 10 s iniciada");
            } else {
                primeiro_acionamento = false;
                esp_timer_stop(timer_pressao_longa);
                esp_timer_start_once(timer_pressao_longa, TEMPO_PRESSAO_LONGA_US);
            }
        }

        if (evento == EVENTO_SOLTOU) {
            esp_timer_stop(timer_pressao_longa);
            if (led_aceso && !pressao_longa_ocorreu && !primeiro_acionamento) {
                esp_timer_stop(timer_contagem);
                esp_timer_start_once(timer_contagem, TEMPO_CONTAGEM_US);
                ESP_LOGI(TAG, "Condição 2: contagem reiniciada para 10 s");
            }
            primeiro_acionamento = false;
        }

        vTaskDelay(1);
    }
}