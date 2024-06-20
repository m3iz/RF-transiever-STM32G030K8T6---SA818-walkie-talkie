/**
  ******************************************************************************
  * @file           : ssd1306.h
  * @brief          : SSD1306 driver header
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */

#ifndef SSD1306_H
#define SSD1306_H



/* Includes ------------------------------------------------------------------*/

#include "stm32g0xx_hal.h"



/* Declarations and definitions ----------------------------------------------*/

#define SSD1306_X_SIZE                                                  128
#define SSD1306_Y_SIZE                                                  64

#define SSD1306_BUFFER_SIZE                                             (SSD1306_X_SIZE *  SSD1306_Y_SIZE) / 8



/* Functions -----------------------------------------------------------------*/

extern int SSD1306_Init();
extern void SSD1306_ClearScreen();
extern void SSD1306_UpdateScreen();
extern void SSD1306_DrawFilledRect(uint8_t xStart, uint8_t xEnd, uint8_t yStart, uint8_t yEnd);
extern uint8_t SSD1306_IsReady();



#endif // #ifndef SSD1306_H
