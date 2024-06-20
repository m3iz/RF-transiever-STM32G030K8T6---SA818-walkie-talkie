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

void SA818_Init(SA818_HandleTypeDef *sa818, UART_HandleTypeDef *huart, GPIO_TypeDef* GPIOx_PTT, uint16_t GPIO_Pin_PTT,
                GPIO_TypeDef* GPIOx_PD, uint16_t GPIO_Pin_PD,
                GPIO_TypeDef* GPIOx_HL, uint16_t GPIO_Pin_HL,
                GPIO_TypeDef* GPIOx_AMP, uint16_t GPIO_Pin_AMP)
{
    sa818->huart = huart;
    sa818->GPIOx_PTT = GPIOx_PTT;
    sa818->GPIO_Pin_PTT = GPIO_Pin_PTT;
    sa818->GPIOx_PD = GPIOx_PD;
    sa818->GPIO_Pin_PD = GPIO_Pin_PD;
    sa818->GPIOx_HL = GPIOx_HL;
    sa818->GPIO_Pin_HL = GPIO_Pin_HL;
    sa818->GPIOx_AMP = GPIOx_AMP;
    sa818->GPIO_Pin_AMP = GPIO_Pin_AMP;

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

uint8_t SA818_SetPower(SA818_HandleTypeDef *sa818, uint8_t is_high)
{
    if (is_high) {
        HAL_GPIO_WritePin(sa818->GPIOx_HL, sa818->GPIO_Pin_HL, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(sa818->GPIOx_HL, sa818->GPIO_Pin_HL, GPIO_PIN_RESET);
    }
    return 1;
}

uint8_t SA818_SetConfig(SA818_HandleTypeDef *sa818, uint8_t bw, char* tx_f, char* rx_f, char* tx_ctcss, char* rx_ctcss, uint8_t squelch)
{
    HAL_GPIO_WritePin(sa818->GPIOx_PTT, sa818->GPIO_Pin_PTT, GPIO_PIN_SET);
    HAL_Delay(100);

    for (uint8_t r = 0; r < 5; r++) {
        char cmd[50];
        snprintf(cmd, sizeof(cmd), "AT+DMOSETGROUP=%d,%s,%s,%s,%d,%s\r\n", bw, tx_f, rx_f, tx_ctcss, squelch, rx_ctcss);
        HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
        if (SA818_ReadSerialTimeout(sa818)) {
            return 1;
        }
    }
    return 0;
}

uint8_t SA818_SetVolume(SA818_HandleTypeDef *sa818, uint8_t volume)
{
    HAL_GPIO_WritePin(sa818->GPIOx_PTT, sa818->GPIO_Pin_PTT, GPIO_PIN_SET);
    HAL_Delay(100);

    if (volume > 8) volume = 8;

    char cmd[20];
    snprintf(cmd, sizeof(cmd), "AT+DMOSETVOLUME=%d\r\n", volume);
    HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);

    return SA818_ReadSerialTimeout(sa818);
}

uint8_t SA818_SetFilters(SA818_HandleTypeDef *sa818, uint8_t preemph, uint8_t highpass, uint8_t lowpass)
{
    HAL_GPIO_WritePin(sa818->GPIOx_PTT, sa818->GPIO_Pin_PTT, GPIO_PIN_SET);
    HAL_Delay(100);

    char cmd[20];
    snprintf(cmd, sizeof(cmd), "AT+SETFILTER=%d,%d,%d\r\n", preemph, highpass, lowpass);
    HAL_UART_Transmit(sa818->huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);

    return SA818_ReadSerialTimeout(sa818);
}

void SA818_PowerDown(SA818_HandleTypeDef *sa818, uint8_t powerdown)
{
    HAL_GPIO_WritePin(sa818->GPIOx_PTT, sa818->GPIO_Pin_PTT, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sa818->GPIOx_PD, sa818->GPIO_Pin_PD, powerdown ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SA818_ChangeMode(SA818_HandleTypeDef *sa818, uint8_t mode)
{
    HAL_GPIO_WritePin(sa818->GPIOx_PTT, sa818->GPIO_Pin_PTT, mode ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (sa818->GPIOx_AMP) {
        HAL_GPIO_WritePin(sa818->GPIOx_AMP, sa818->GPIO_Pin_AMP, mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void SA818_SetModeTX(SA818_HandleTypeDef *sa818)
{
    SA818_ChangeMode(sa818, 1);
}

void SA818_SetModeRX(SA818_HandleTypeDef *sa818)
{
    SA818_ChangeMode(sa818, 0);
}
