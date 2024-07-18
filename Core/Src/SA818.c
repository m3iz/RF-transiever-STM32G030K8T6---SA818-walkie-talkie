/*
    SA818.c - SA818U/SA818V Communication Library for STM32 using HAL.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    For a full copy of the GNU General Public License,
    see <http://www.gnu.org/licenses/>.
*/

#include "SA818.h"

void SA818_Init(SA818_HandleTypeDef *sa818, UART_HandleTypeDef *huart)
{
    sa818->huart = huart;


    HAL_GPIO_WritePin(sa818->GPIOx_PTT, sa818->GPIO_Pin_PTT, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sa818->GPIOx_PD, sa818->GPIO_Pin_PD, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sa818->GPIOx_HL, sa818->GPIO_Pin_HL, GPIO_PIN_RESET);
    if (sa818->GPIOx_AMP) {
        HAL_GPIO_WritePin(sa818->GPIOx_AMP, sa818->GPIO_Pin_AMP, GPIO_PIN_RESET);
    }
}

uint8_t SA818_ReadSerialTimeout(SA818_HandleTypeDef *sa818)
{
    uint8_t readed = 0;
    uint32_t timeout = HAL_GetTick();
    uint8_t rx_buffer[1];

    while (HAL_GetTick() - timeout < 500) {
        HAL_Delay(100);
        while (HAL_UART_Receive(sa818->huart, rx_buffer, 1, 10) == HAL_OK) {
            readed = 1;
        }
        if (readed) break;
    }
    return readed;
}

uint8_t SA818_Begin(SA818_HandleTypeDef *sa818)
{
    char cmd[] = "AT\r\n";
    HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, sizeof(cmd) - 1, HAL_MAX_DELAY);
    HAL_Delay(500);

    for (uint8_t r = 0; r < 5; r++) {
        char cmd_connect[] = "AT+DMOCONNECT\r\n";
        HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd_connect, sizeof(cmd_connect) - 1, HAL_MAX_DELAY);
        if (SA818_ReadSerialTimeout(sa818)) {
            return 1;
        }
    }
    return 0;
}

uint8_t SA818_SetConfig(SA818_HandleTypeDef *sa818, uint8_t bw, float tx_f, float rx_f, char* tx_ctcss, char* rx_ctcss, uint8_t squelch)
{
    for (uint8_t r = 0; r < 5; r++) {
        char cmd[50];
        snprintf(cmd, sizeof(cmd), "AT+DMOSETGROUP=%d,%d,%d,%s,%d,%s\r\n", bw, tx_f, rx_f, tx_ctcss, squelch, rx_ctcss);
        HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
        if (SA818_ReadSerialTimeout(sa818)) {
            return 1;
        }
    }
    return 0;
}

uint8_t SA818_SetVolume(SA818_HandleTypeDef *sa818, uint8_t volume)
{
    if (volume > 8) volume = 8;

    char cmd[20];
    snprintf(cmd, sizeof(cmd), "AT+DMOSETVOLUME=%d\r\n", volume);
    HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);

    return SA818_ReadSerialTimeout(sa818);
}

uint8_t SA818_SetFilters(SA818_HandleTypeDef *sa818, uint8_t preemph, uint8_t highpass, uint8_t lowpass)
{

    char cmd[20];
    snprintf(cmd, sizeof(cmd), "AT+SETFILTER=%d,%d,%d\r\n", preemph, highpass, lowpass);
    HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);

    return SA818_ReadSerialTimeout(sa818);
}
