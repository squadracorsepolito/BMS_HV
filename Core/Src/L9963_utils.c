#include "L9963_utils.h"
#include "L9963E.h"
#include "stm32_if.h"

L9963E_HandleTypeDef hl9963e;

const L9963E_IfTypeDef interface_H = {
    .L9963E_IF_DelayMs = DelayMs,
    .L9963E_IF_GetTickMs = GetTickMs,
    .L9963E_IF_GPIO_ReadPin = L9963TH_GPIO_ReadPin,
    .L9963E_IF_GPIO_WritePin = L9963TH_GPIO_WritePin,
    .L9963E_IF_SPI_Receive = L9963TH_SPI_Receive,
    .L9963E_IF_SPI_Transmit = L9963TH_SPI_Transmit
};

const L9963E_IfTypeDef interface_L = {
    .L9963E_IF_DelayMs = DelayMs,
    .L9963E_IF_GetTickMs = GetTickMs,
    .L9963E_IF_GPIO_ReadPin = L9963TL_GPIO_ReadPin,
    .L9963E_IF_GPIO_WritePin = L9963TL_GPIO_WritePin,
    .L9963E_IF_SPI_Receive = L9963TL_SPI_Receive,
    .L9963E_IF_SPI_Transmit = L9963TL_SPI_Transmit
};

// TRYING TO FIGURE OUT HOW TO INITIALIZE THE DRIVER HANDLE
void L9963E_utils_init(void) {
    L9963E_init(&hl9963e , interface_H, N_SLAVES);


}