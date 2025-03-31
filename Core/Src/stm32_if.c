/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#include "stm32_if.h"

#include "main.h"
#include "spi.h"

L9963E_IF_PinState L9963TH_GPIO_ReadPin(L9963E_IF_PINS pin) {
    L9963E_IF_PinState state = L9963E_IF_GPIO_PIN_RESET;
    switch (pin) {
        case L9963E_IF_CS:
            state = HAL_GPIO_ReadPin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin);
            break;
        case L9963E_IF_TXEN:
            state = HAL_GPIO_ReadPin(L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin);
            break;
        case L9963E_IF_BNE:
            state = HAL_GPIO_ReadPin(L9963TH_BNE_GPIO_IN_GPIO_Port, L9963TH_BNE_GPIO_IN_Pin);
            break;
        case L9963E_IF_ISOFREQ:
            state = HAL_GPIO_ReadPin(L9963TH_ISOFREQ_GPIO_OUT_GPIO_Port, L9963TH_ISOFREQ_GPIO_OUT_Pin);
            break;
        case L9963E_IF_DIS:
            state = HAL_GPIO_ReadPin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin);
            break;
    }

    return state == L9963E_IF_GPIO_PIN_RESET ? GPIO_PIN_RESET : GPIO_PIN_SET;  //convert lib state to stm state
}

L9963E_StatusTypeDef L9963TH_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state) {
    GPIO_PinState stm_state = state == L9963E_IF_GPIO_PIN_RESET ? GPIO_PIN_RESET
                                                                : GPIO_PIN_SET;  //convert lib state to stm state
    switch (pin) {
        case L9963E_IF_CS:
            HAL_GPIO_WritePin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin, stm_state);
            break;
        case L9963E_IF_TXEN:
            HAL_GPIO_WritePin(L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin, stm_state);
            break;
        case L9963E_IF_BNE:
            HAL_GPIO_WritePin(L9963TH_BNE_GPIO_IN_GPIO_Port, L9963TH_BNE_GPIO_IN_Pin, stm_state);
            break;
        case L9963E_IF_ISOFREQ:
            HAL_GPIO_WritePin(L9963TH_ISOFREQ_GPIO_OUT_GPIO_Port, L9963TH_ISOFREQ_GPIO_OUT_Pin, stm_state);
            break;
        case L9963E_IF_DIS:
            HAL_GPIO_WritePin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin, stm_state);
            break;
        default:
            return L9963E_ERROR;
    }
    return L9963E_OK;
}
L9963E_StatusTypeDef L9963TH_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    HAL_StatusTypeDef errorcode;

    errorcode = HAL_SPI_Receive(&hspi3, data, size, timeout_ms);

    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}
L9963E_StatusTypeDef L9963TH_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    HAL_StatusTypeDef errorcode;

    errorcode = HAL_SPI_Transmit(&hspi3, data, size, timeout_ms);

    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}

L9963E_IF_PinState L9963TL_GPIO_ReadPin(L9963E_IF_PINS pin) {
    L9963E_IF_PinState state = L9963E_IF_GPIO_PIN_RESET;
    switch (pin) {
        case L9963E_IF_CS:
            state = HAL_GPIO_ReadPin(L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin);
            break;
        case L9963E_IF_TXEN:
            state = HAL_GPIO_ReadPin(L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin);
            break;
        case L9963E_IF_BNE:
            state = HAL_GPIO_ReadPin(L9963TL_BNE_GPIO_IN_GPIO_Port, L9963TL_BNE_GPIO_IN_Pin);
            break;
        case L9963E_IF_ISOFREQ:
            state = HAL_GPIO_ReadPin(L9963TL_ISOFREQ_GPIO_OUT_GPIO_Port, L9963TL_ISOFREQ_GPIO_OUT_Pin);
            break;
        case L9963E_IF_DIS:
            state = HAL_GPIO_ReadPin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin);
            break;
    }
    return state == L9963E_IF_GPIO_PIN_RESET ? GPIO_PIN_RESET : GPIO_PIN_SET;  //convert lib state to stm state
}

L9963E_StatusTypeDef L9963TL_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state) {
    GPIO_PinState stm_state = state == L9963E_IF_GPIO_PIN_RESET ? GPIO_PIN_RESET
                                                                : GPIO_PIN_SET;  //convert lib state to stm state
    switch (pin) {
        case L9963E_IF_CS:
            HAL_GPIO_WritePin(L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin, stm_state);
            break;
        case L9963E_IF_TXEN:
            HAL_GPIO_WritePin(L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin, stm_state);
            break;
        case L9963E_IF_BNE:
            HAL_GPIO_WritePin(L9963TL_BNE_GPIO_IN_GPIO_Port, L9963TL_BNE_GPIO_IN_Pin, stm_state);
            break;
        case L9963E_IF_ISOFREQ:
            HAL_GPIO_WritePin(L9963TL_ISOFREQ_GPIO_OUT_GPIO_Port, L9963TL_ISOFREQ_GPIO_OUT_Pin, stm_state);
            break;
        case L9963E_IF_DIS:
            HAL_GPIO_WritePin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin, stm_state);
            break;
        default:
            return L9963E_ERROR;
    }
    return L9963E_OK;
}

L9963E_StatusTypeDef L9963TL_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    HAL_StatusTypeDef errorcode;

    errorcode = HAL_SPI_Receive(&hspi2, data, size, timeout_ms);

    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}

L9963E_StatusTypeDef L9963TL_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    HAL_StatusTypeDef errorcode;

    errorcode = HAL_SPI_Transmit(&hspi2, data, size, timeout_ms);

    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}

uint32_t GetTickMs(void) {
    return HAL_GetTick();
}
void DelayMs(uint32_t delay) {
    HAL_Delay(delay);
}