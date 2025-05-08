/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#ifndef STM32_IF_H
#define STM32_IF_H

#include "L9963E_interface.h"

//typedef enum { L9963T_H = 0, L9963T_L = 1 } L9963T_PORT_T;

L9963E_IF_PinState L9963TH_GPIO_ReadPin(L9963E_IF_PINS pin);
L9963E_StatusTypeDef L9963TH_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state);
L9963E_StatusTypeDef L9963TH_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms);
L9963E_StatusTypeDef L9963TH_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms);

L9963E_IF_PinState L9963TL_GPIO_ReadPin(L9963E_IF_PINS pin);
L9963E_StatusTypeDef L9963TL_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state);
L9963E_StatusTypeDef L9963TL_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms);
L9963E_StatusTypeDef L9963TL_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms);

uint32_t GetTickMs(void);
void DelayMs(uint32_t delay);

#endif  //STM32_IF_H