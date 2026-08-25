#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    printf("Nome: Mayara\n");
    printf("Matricula: 202221250014\n");

    int contador = 0;

    while (1)
    {
        printf("Tempo decorrido: %d segundos\n", contador);

        contador += 2;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}