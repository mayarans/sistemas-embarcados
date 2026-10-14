#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define TX_PIN GPIO_NUM_17       // Pino de transmissão da UART2
#define RX_PIN GPIO_NUM_16        // Pino de recepção da UART2
#define TEMPO_ESPERA 2000 // Tempo de espera em milissegundos
#define LED_PIN GPIO_NUM_5

#include "esp_log.h"


// configurar e inicializar periférico UART2 do ESP32 via drivers nativos
// uart_param_config e uart_driver_install são funções da API do driver UART do ESP32, que permitem configurar os parâmetros da UART e instalar o driver para uso.
void configure_uart2(void)
{
    // Configuração dos parâmetros da UART2
    uart_config_t uart_config = {
        .baud_rate = 115200,                  // Taxa de transmissão em bauds
        .data_bits = UART_DATA_8_BITS,        // Número de bits de dados
        .parity = UART_PARITY_DISABLE,        // Sem paridade
        .stop_bits = UART_STOP_BITS_1,        // Um bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE // Sem controle de fluxo
    };

    // Configura a UART2 com os parâmetros definidos
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Instala o driver da UART2 com um buffer de 1024 bytes para RX e TX
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0));

}

void configure_led(void)
{
    // Configura o pino do LED como saída
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

// função para enviar comandos via UART2 para acender ou apagar o LED
void send_command(int command)
{
    if (command == 1)
    {
        uart_write_bytes(UART_NUM_2, "LIGAR\n", strlen("LIGAR\n"));
    }
    else if (command == 0)
    {
        uart_write_bytes(UART_NUM_2, "DESLIGAR\n", strlen("DESLIGAR\n"));
    }
    else
    {
        printf("Comando inválido. Use 1 para ligar o LED e 0 para desligar.\n");
    }
}

void app_main(void)
{
    configure_uart2();
    configure_led();
    char buffer[1024];
    int ligar_led = 1; // Variável para controlar o estado do LED (1 = aceso, 0 = apagado)
    while (1)
    {
        send_command(ligar_led); // Envia o comando para acender/apagar o LED
        ESP_LOGI("UART", "Comando enviado: %s", ligar_led ? "LIGAR" : "DESLIGAR");
        // Para a leitura da UART2 (uart_read_bytes), deve-se utilizar o tempo de estouro (timeout) oferecido pela própria API do driver em milissegundos para controlar o fluxo de execução.
        int len = uart_read_bytes(UART_NUM_2, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(TEMPO_ESPERA)); // Lê os bytes recebidos na UART2 com timeout
        if (len > 0)
        {
            // Remove possíveis terminadores de linha (\r ou \n) recebidos
            while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r'))
            {
                len--;
            }
            // Adiciona a terminação nula à string lida
            buffer[len] = '\0';
            ESP_LOGI("UART", "Recebido: %.*s", len, buffer); // Exibe os bytes recebidos na UART2

            // Verifica se a string lida é "LIGAR"
            if (strcmp(buffer, "LIGAR") == 0)
            {
                // Acende o LED
                ESP_LOGI("LED", "Acendendo o LED");
                gpio_set_level(LED_PIN, 1);
            }
            // Verifica se a string lida é "DESLIGAR"
            else if (strcmp(buffer, "DESLIGAR") == 0)
            {
                // Apaga o LED
                ESP_LOGI("LED", "Apagando o LED");
                gpio_set_level(LED_PIN, 0);
            }
        }
        ligar_led = !ligar_led; // Alterna o estado do LED para a próxima iteração
    }
}