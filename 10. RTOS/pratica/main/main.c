// =====================================================================
// Atividade 10 - Sistemas Multitarefa com RTOS e Sensores I2C
// ESP32-S3 | ESP-IDF 6.0.2 | FreeRTOS
//
// Ideia geral: em vez de um unico "while(1)" fazendo tudo, o programa e
// dividido em 5 tarefas (tasks), como funcionarios de uma loja, cada um
// com sua funcao:
//   - tarefa_botao   : fica de olho no botao e avisa quando ele e apertado
//   - tarefa_adc     : le o potenciometro e manda o valor para o LED
//   - tarefa_led     : recebe o valor e ajusta o brilho (respeita LIVE/HOLD)
//   - tarefa_imu     : le a aceleracao do MPU-6050 pelo I2C
//   - tarefa_console : imprime o resumo no terminal a cada 500 ms
//
// Os "bilhetes" que os funcionarios trocam entre si (IPC):
//   - Fila (queue)            : adc  -> led      (valor do brilho)
//   - Semaforo binario        : botao -> led     (aviso "apertou!")
//   - Mutex                   : protege a "lousa" (struct) compartilhada
// =====================================================================

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"        // fila
#include "freertos/semphr.h"       // semaforo binario e mutex
#include "driver/gpio.h"           // botao
#include "driver/ledc.h"           // PWM do LED
#include "driver/i2c_master.h"     // I2C (comunicacao com o MPU-6050)
#include "esp_adc/adc_oneshot.h"   // ADC (potenciometro)

// ---------------- Pinos (tabela do Word, recomendacao para ESP32-S3) ----------------
#define PINO_POT        GPIO_NUM_4         // potenciometro (entrada analogica)
#define CANAL_POT       ADC_CHANNEL_3      // GPIO4 no ESP32-S3 = ADC1, canal 3
#define PINO_LED        GPIO_NUM_18        // LED (PWM)
#define PINO_BOTAO      GPIO_NUM_5         // botao (com resistor de 10k)
#define PINO_SDA        GPIO_NUM_8         // I2C: linha de dados
#define PINO_SCL        GPIO_NUM_9         // I2C: linha de clock

// ---------------- Parametros pedidos na atividade ----------------
#define ADC_MAX             4095           // ADC de 12 bits: 0 a 4095
#define PWM_MAX             8191           // PWM de 13 bits: 0 a 8191
#define FREQ_PWM_HZ         5000           // 5 kHz
#define TENSAO_MAX_MV       3300           // 3,3 V em milivolts
#define I2C_FREQ_HZ         100000         // 100 kHz (Standard Mode)
#define IMU_ENDERECO        0x68           // endereco I2C do MPU-6050
#define IMU_SENSIBILIDADE   16384.0f       // LSB por g na escala +-2g
#define INTERVALO_PRINT_MS  500            // telemetria a cada 500 ms

// ---------------- Registradores do MPU-6050 (gavetas internas do sensor) ----------------
#define IMU_REG_WHO_AM_I      0x75         // "documento de identidade" do sensor (deve valer 0x68)
#define IMU_REG_PWR_MGMT_1    0x6B         // gerenciamento de energia (precisa "acordar" o sensor)
#define IMU_REG_ACCEL_CONFIG  0x1C         // escolhe a escala do acelerometro
#define IMU_REG_ACCEL_XOUT_H  0x3B         // inicio dos 6 bytes de aceleracao (X, Y, Z)

// ---------------- Prioridades (maior numero = mais importante) ----------------
#define PRIO_BOTAO      4                  // reage a uma pessoa: nao pode demorar
#define PRIO_LED        3
#define PRIO_ADC        3
#define PRIO_IMU        2
#define PRIO_CONSOLE    1                  // so imprime: pode esperar

#define TAMANHO_PILHA   4096               // memoria reservada para cada tarefa (bytes)
#define NIVEL_APERTADO  0                  // botao com pull-up: apertado = nivel 0

static const char *TAG = "ATIVIDADE10";

// ---------------- Estrutura compartilhada (a "lousa" do grupo) ----------------
typedef struct {
    int   pot_bruto;      // leitura do potenciometro, 0 a 4095
    int   pot_mv;         // mesma leitura em milivolts
    int   led_percent;    // brilho atual do LED, 0 a 100 %
    bool  modo_hold;      // false = LIVE, true = HOLD
    float acel_x;         // aceleracao em g
    float acel_y;
    float acel_z;
} dados_t;

// ---------------- Objetos de comunicacao (IPC) ----------------
// Sao criados no app_main ANTES das tarefas e depois so sao usados
// pelas funcoes do FreeRTOS (que ja sao seguras entre tarefas).
static QueueHandle_t     fila_brilho;      // adc -> led
static SemaphoreHandle_t sem_botao;        // botao -> led
static SemaphoreHandle_t mutex_dados;      // protege dados_compartilhados

// ATENCAO: esta variavel so pode ser tocada dentro das funcoes dados_*()
// abaixo, que sempre pegam o mutex antes. Ninguem mais acessa ela direto.
static dados_t dados_compartilhados;

// =====================================================================
// Funcoes de acesso a "lousa". Todas seguem o mesmo roteiro:
//   1) pega a chave (Take)   2) le/escreve   3) devolve a chave (Give)
// Enquanto uma tarefa esta com a chave, as outras esperam na porta.
// =====================================================================

static void dados_escreve_pot(int bruto, int mv)
{
    // tenta pegar a chave; espera no maximo 100 ms
    if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
        dados_compartilhados.pot_bruto = bruto;     // escreve na lousa
        dados_compartilhados.pot_mv = mv;
        xSemaphoreGive(mutex_dados);                // devolve a chave
    }
}

static void dados_escreve_led(int percent, bool hold)
{
    if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
        dados_compartilhados.led_percent = percent;
        dados_compartilhados.modo_hold = hold;
        xSemaphoreGive(mutex_dados);
    }
}

static void dados_escreve_imu(float x, float y, float z)
{
    if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
        dados_compartilhados.acel_x = x;
        dados_compartilhados.acel_y = y;
        dados_compartilhados.acel_z = z;
        xSemaphoreGive(mutex_dados);
    }
}

// Copia a lousa inteira para uma variavel local de quem chamou.
// Assim a tarefa devolve a chave rapido e imprime com calma depois.
static bool dados_le(dados_t *copia)
{
    if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
        *copia = dados_compartilhados;              // copia tudo de uma vez
        xSemaphoreGive(mutex_dados);
        return true;
    }
    return false;                                   // nao conseguiu a chave
}

// =====================================================================
// TAREFA 1: BOTAO
// Le o botao a cada 10 ms. So considera um aperto "valido" se o botao
// ficar na mesma posicao por 30 ms seguidos (debounce: ignora o
// "tremor" dos contatos mecanicos). Quando valida, da um Give no
// semaforo para avisar a tarefa do LED.
// =====================================================================
static void tarefa_botao(void *arg)
{
    // configura o pino do botao como entrada
    gpio_config_t cfg_botao = {
        .pin_bit_mask = 1ULL << PINO_BOTAO,
        .mode = GPIO_MODE_INPUT,                    // so le o pino
        .pull_up_en = GPIO_PULLUP_DISABLE,          // o pull-up ja existe no circuito (10k)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,             // sem interrupcao: vamos "olhar" de tempos em tempos (polling)
    };
    ESP_ERROR_CHECK(gpio_config(&cfg_botao));

    int estado_estavel = !NIVEL_APERTADO;           // estado "oficial" do botao (comeca solto)
    int contador = 0;                               // quantas leituras seguidas diferentes do oficial

    while (1) {
        int leitura = gpio_get_level(PINO_BOTAO);   // olha o botao agora

        if (leitura != estado_estavel) {            // o botao esta diferente do oficial...
            contador++;                             // ...conta mais uma leitura diferente
            if (contador >= 3) {                    // 3 leituras x 10 ms = 30 ms estavel
                estado_estavel = leitura;           // aceita a mudanca
                contador = 0;
                if (estado_estavel == NIVEL_APERTADO) {
                    xSemaphoreGive(sem_botao);      // AVISA: o botao foi apertado!
                }
            }
        } else {
            contador = 0;                           // voltou ao normal: era so tremor
        }

        vTaskDelay(pdMS_TO_TICKS(10));              // descansa 10 ms (cede a CPU)
    }
}

// =====================================================================
// TAREFA 2: ADC (potenciometro)
// A cada 20 ms le o potenciometro, converte de 12 bits (0-4095) para
// 13 bits (0-8191) e coloca na fila para a tarefa do LED.
// =====================================================================
static void tarefa_adc(void *arg)
{
    // --- configura o ADC (igual a atividade 9) ---
    adc_oneshot_unit_handle_t adc;                  // "controle remoto" do ADC
    adc_oneshot_unit_init_cfg_t cfg_unidade = {
        .unit_id = ADC_UNIT_1,                      // unidade 1 do ADC
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg_unidade, &adc));

    adc_oneshot_chan_cfg_t cfg_canal = {
        .atten = ADC_ATTEN_DB_12,                   // le de 0 a ~3,3 V
        .bitwidth = ADC_BITWIDTH_12,                // 12 bits: 0 a 4095
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, CANAL_POT, &cfg_canal));

    TickType_t ultimo_acordar = xTaskGetTickCount();   // referencia para o ritmo fixo

    while (1) {
        int bruto = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, CANAL_POT, &bruto));   // 0 a 4095

        int mv = bruto * TENSAO_MAX_MV / ADC_MAX;   // converte para milivolts
        uint32_t valor_13bits = (uint32_t)bruto * PWM_MAX / ADC_MAX; // regra de tres: 0..4095 -> 0..8191

        dados_escreve_pot(bruto, mv);               // guarda na lousa (para a telemetria)

        // coloca o valor na fila. Tempo de espera 0: se a fila estiver cheia,
        // descarta (o proximo valor chega em 20 ms de qualquer forma)
        xQueueSend(fila_brilho, &valor_13bits, 0);

        // acorda exatamente 20 ms depois da ultima vez (ritmo constante)
        vTaskDelayUntil(&ultimo_acordar, pdMS_TO_TICKS(20));
    }.
}

// =====================================================================
// TAREFA 3: LED
// - Configura o PWM de 13 bits / 5 kHz.
// - Se recebeu o aviso do botao (semaforo), alterna LIVE <-> HOLD.
// - Se recebeu um valor na fila e esta em LIVE, muda o brilho.
//   Em HOLD o valor e lido da fila mas jogado fora: o LED congela.
// =====================================================================
static void tarefa_led(void *arg)
{
    // O "timer" define a frequencia e a resolucao do PWM
    ledc_timer_config_t cfg_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,          // o ESP32-S3 so tem o modo "low speed"
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,       // 13 bits: 0 a 8191
        .freq_hz = FREQ_PWM_HZ,                     // 5 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&cfg_timer));

    // O "canal" liga o timer ao pino do LED
    ledc_channel_config_t cfg_canal = {
        .gpio_num = PINO_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,                                  // comeca apagado
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg_canal));

    bool modo_hold = false;                         // comeca em LIVE
    uint32_t valor = 0;                             // onde a fila entrega o valor recebido

    while (1) {
        // 1) O botao foi apertado? (timeout 0 = olha e segue, sem esperar)
        if (xSemaphoreTake(sem_botao, 0) == pdTRUE) {
            modo_hold = !modo_hold;                 // troca LIVE <-> HOLD
            ESP_LOGI(TAG, "Modo: %s", modo_hold ? "HOLD" : "LIVE");
            // atualiza o status na lousa (mantem o ultimo brilho)
            dados_t atual;
            if (dados_le(&atual)) {
                dados_escreve_led(atual.led_percent, modo_hold);
            }
        }

        // 2) Chegou valor novo na fila? (timeout 0 = olha e segue)
        if (xQueueReceive(fila_brilho, &valor, 0) == pdTRUE) {
            if (!modo_hold) {                       // so aplica se estiver em LIVE
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, valor);   // guarda o novo brilho
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);       // aplica no LED
                dados_escreve_led((int)(valor * 100 / PWM_MAX), modo_hold);  // brilho em % na lousa
            }
            // em HOLD: o valor e descartado e o LED fica como estava
        }

        vTaskDelay(pdMS_TO_TICKS(10));              // descansa 10 ms (cede a CPU)
    }
}

// =====================================================================
// Funcao auxiliar do IMU: escreve 1 byte em um registrador do sensor.
// No I2C, enviamos primeiro o "numero da gaveta" e depois o valor.
// =====================================================================
static esp_err_t imu_escreve(i2c_master_dev_handle_t imu, uint8_t registrador, uint8_t valor)
{
    uint8_t pacote[2] = { registrador, valor };
    return i2c_master_transmit(imu, pacote, 2, 100);   // timeout de 100 ms
}

// =====================================================================
// TAREFA 4: IMU (MPU-6050)
// A cada 50 ms le a aceleracao nos 3 eixos pelo I2C, converte para g
// e guarda na lousa compartilhada.
// =====================================================================
static void tarefa_imu(void *arg)
{
    // --- cria o barramento I2C (a "rua" por onde passam os dados) ---
    i2c_master_bus_config_t cfg_barramento = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PINO_SDA,
        .scl_io_num = PINO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,                     // filtra ruidos curtos na linha
        .flags.enable_internal_pullup = true,       // pull-up interno de reforco
    };
    i2c_master_bus_handle_t barramento;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg_barramento, &barramento));

    // --- registra o MPU-6050 como "morador" dessa rua ---
    i2c_device_config_t cfg_imu = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ENDERECO,             // endereco 0x68
        .scl_speed_hz = I2C_FREQ_HZ,                // 100 kHz
    };
    i2c_master_dev_handle_t imu;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(barramento, &cfg_imu, &imu));

    // --- confere se o sensor responde (le o WHO_AM_I) ---
    uint8_t reg_id = IMU_REG_WHO_AM_I;
    uint8_t id = 0;
    if (i2c_master_transmit_receive(imu, &reg_id, 1, &id, 1, 100) != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6050 nao respondeu! Confira SDA/SCL, 3V3 e GND.");
        vTaskDelete(NULL);                          // encerra so esta tarefa
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (esperado 0x68)", id);

    // --- "acorda" o sensor (ele liga dormindo) e escolhe a escala +-2g ---
    ESP_ERROR_CHECK(imu_escreve(imu, IMU_REG_PWR_MGMT_1, 0x00));     // 0 = acordado
    ESP_ERROR_CHECK(imu_escreve(imu, IMU_REG_ACCEL_CONFIG, 0x00));   // 0 = escala +-2g

    TickType_t ultimo_acordar = xTaskGetTickCount();

    while (1) {
        uint8_t reg = IMU_REG_ACCEL_XOUT_H;         // "comecar a ler a partir desta gaveta"
        uint8_t b[6];                               // 6 bytes: X_alto, X_baixo, Y_alto, Y_baixo, Z_alto, Z_baixo

        if (i2c_master_transmit_receive(imu, &reg, 1, b, 6, 100) == ESP_OK) {
            // junta o byte alto com o baixo para formar um numero de 16 bits com sinal
            int16_t bruto_x = (int16_t)((b[0] << 8) | b[1]);
            int16_t bruto_y = (int16_t)((b[2] << 8) | b[3]);
            int16_t bruto_z = (int16_t)((b[4] << 8) | b[5]);

            // converte para g: valor bruto / 16384 (sensibilidade da escala +-2g)
            dados_escreve_imu(bruto_x / IMU_SENSIBILIDADE,
                              bruto_y / IMU_SENSIBILIDADE,
                              bruto_z / IMU_SENSIBILIDADE);
        } else {
            ESP_LOGW(TAG, "Falha ao ler o MPU-6050");
        }

        vTaskDelayUntil(&ultimo_acordar, pdMS_TO_TICKS(50));   // ritmo fixo de 50 ms
    }
}

// =====================================================================
// TAREFA 5: CONSOLE
// A cada 500 ms copia a lousa (com o mutex) e imprime no terminal.
// =====================================================================
static void tarefa_console(void *arg)
{
    TickType_t ultimo_acordar = xTaskGetTickCount();

    while (1) {
        dados_t d;                                  // copia local: so desta tarefa
        if (dados_le(&d)) {                         // pega a chave, copia e devolve
            printf("=====================================================\n");
            printf("STATUS: [%s] | POT: %d (%d mV) | LED: %d%%\n",
                   d.modo_hold ? "HOLD" : "LIVE", d.pot_bruto, d.pot_mv, d.led_percent);
            printf("IMU ACCEL (g): X: %.2f | Y: %.2f | Z: %.2f\n",
                   d.acel_x, d.acel_y, d.acel_z);
            printf("=====================================================\n");
        }

        vTaskDelayUntil(&ultimo_acordar, pdMS_TO_TICKS(INTERVALO_PRINT_MS));   // 500 ms
    }
}

// =====================================================================
// app_main: cria os objetos de comunicacao e depois as tarefas.
// Depois disso o FreeRTOS assume o comando e alterna entre elas.
// =====================================================================
void app_main(void)
{
    // fila com espaco para 5 valores do tipo uint32_t
    fila_brilho = xQueueCreate(5, sizeof(uint32_t));
    // semaforo binario: nasce "vazio" (ninguem apertou ainda)
    sem_botao = xSemaphoreCreateBinary();
    // mutex: a chave da lousa compartilhada
    mutex_dados = xSemaphoreCreateMutex();

    // se faltou memoria para criar algum, nao adianta continuar
    if (fila_brilho == NULL || sem_botao == NULL || mutex_dados == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila/semaforo/mutex");
        return;
    }

    // xTaskCreate(funcao, nome, pilha, parametro, prioridade, handle)
    xTaskCreate(tarefa_botao,   "Botao",   TAMANHO_PILHA, NULL, PRIO_BOTAO,   NULL);
    xTaskCreate(tarefa_adc,     "ADC",     TAMANHO_PILHA, NULL, PRIO_ADC,     NULL);
    xTaskCreate(tarefa_led,     "LED",     TAMANHO_PILHA, NULL, PRIO_LED,     NULL);
    xTaskCreate(tarefa_imu,     "IMU",     TAMANHO_PILHA, NULL, PRIO_IMU,     NULL);
    xTaskCreate(tarefa_console, "Console", TAMANHO_PILHA, NULL, PRIO_CONSOLE, NULL);

    ESP_LOGI(TAG, "Sistema iniciado em modo LIVE");
}
