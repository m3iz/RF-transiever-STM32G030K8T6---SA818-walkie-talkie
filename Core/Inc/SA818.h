/*
    SA818.h - SA818U/SA818V Communication Library for STM32 using HAL.

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

#ifndef SA818_H
#define SA818_H

#include "stm32g0xx_hal.h"
#include <string.h>
#include <stdio.h>

#define SA_BANDWIDTH_12_5KHZ    0
#define SA_BANDWIDTH_25KHZ      1

#define SA_CTCSS_OFF    "0000"
#define SA_SQUELCH_OFF  0

#define SA_POWER_LOW    0
#define SA_POWER_HIGH   1

#define SA_VOLUME_MIN   0
#define SA_VOLUME_DEFAULT 4
#define SA_VOLUME_MAX   8

#define SA_FILTER_OFF   0
#define SA_FILTER_ON    1

#define SA_POWER_OFF    0
#define SA_POWER_ON     1

#define SA_MODE_TX      0
#define SA_MODE_RX      1

// Define SA818 structure
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef* GPIOx_PTT;
    uint16_t GPIO_Pin_PTT;
    GPIO_TypeDef* GPIOx_PD;
    uint16_t GPIO_Pin_PD;
    GPIO_TypeDef* GPIOx_HL;
    uint16_t GPIO_Pin_HL;
    GPIO_TypeDef* GPIOx_AMP;
    uint16_t GPIO_Pin_AMP;
} SA818_HandleTypeDef;

void SA818_Init(SA818_HandleTypeDef *sa818, UART_HandleTypeDef *huart);

uint8_t SA818_ReadSerialTimeout(SA818_HandleTypeDef *sa818);
uint8_t SA818_Begin(SA818_HandleTypeDef *sa818);
uint8_t SA818_SetPower(SA818_HandleTypeDef *sa818, uint8_t is_high);
uint8_t SA818_SetConfig(SA818_HandleTypeDef *sa818, uint8_t bw, float tx_f, float rx_f, char* tx_ctcss, char* rx_ctcss, uint8_t squelch);
uint8_t SA818_SetVolume(SA818_HandleTypeDef *sa818, uint8_t volume);
uint8_t SA818_SetFilters(SA818_HandleTypeDef *sa818, uint8_t preemph, uint8_t highpass, uint8_t lowpass);
void SA818_PowerDown(SA818_HandleTypeDef *sa818, uint8_t powerdown);
void SA818_ChangeMode(SA818_HandleTypeDef *sa818, uint8_t mode);
void SA818_SetModeTX(SA818_HandleTypeDef *sa818);
void SA818_SetModeRX(SA818_HandleTypeDef *sa818);

#endif // SA818_H
