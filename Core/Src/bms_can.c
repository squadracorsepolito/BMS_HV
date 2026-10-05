#include "bms_can.h"
#include "can.h"
#include "bms_hv_fsm.h"

static BMS_CanCommands commands;
static uint8_t started;
static uint32_t last_polled_at;

static void stop_requests(void) {
    commands = (BMS_CanCommands){0};
    BMS_HV_set_commands(0, 0, 0);
}

int BMS_CAN_init(void) {
    CAN_FilterTypeDef filter = {0};
    started = 0;
    stop_requests();
    /* Timing configured for 16 MHz APB1, 16 time quanta, prescaler 1. */
    if (HAL_RCC_GetPCLK1Freq() != 16000000U) return -1;
    filter.FilterBank = 0;
    filter.SlaveStartFilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = BMS_VCU_COMMAND_ID << 5;
    filter.FilterMaskIdHigh = 0xFFE0U;
    filter.FilterMaskIdLow = 0x6U; /* Require standard identifier and data frame. */
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK) return -1;
    last_polled_at = HAL_GetTick();
    started = 1;
    return 0;
}

void BMS_CAN_routine(void) {
    if (!started) {
        stop_requests();
        return;
    }
    if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) ||
        __HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0)) {
        /* Lost ordering cannot be repaired by replaying queued activation frames.
         * Stop reception until an explicit reinitialization/power cycle. */
        started = 0;
        stop_requests();
        return;
    }
    uint32_t now = HAL_GetTick();
    uint8_t discard_queued = now - last_polled_at >= BMS_VCU_COMMAND_MAX_AGE_MS;
    if (discard_queued) stop_requests();
    /* Hardware FIFO holds three frames. Bound work even under sustained traffic. */
    for (unsigned count = 0; count < 3 &&
         HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0); ++count) {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) ||
            __HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0)) {
            started = 0;
            stop_requests();
            return;
        }
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            started = 0;
            stop_requests();
            return;
        }
        if (!discard_queued) {
            /* Polling has no trusted arrival timestamp. Use the previous poll
             * as the earliest possible arrival; dequeue time must not renew an
             * old activation request. Discard backlog after a watchdog-length gap. */
            BMS_CAN_decode_command(&commands,
                header.IDE == CAN_ID_STD ? header.StdId : header.ExtId,
                header.IDE != CAN_ID_STD, header.RTR != CAN_RTR_DATA,
                header.DLC, data, last_polled_at);
        }
    }
    last_polled_at = now;
    if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) ||
        __HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0)) {
        started = 0;
        stop_requests();
        return;
    }
    BMS_CAN_expire_commands(&commands, HAL_GetTick());
    BMS_HV_set_commands(0, commands.valid && commands.drive,
                       commands.valid && commands.balance);
    /* clear_error never resets the latched fault. The reset policy is pending.
     * all_measurements is decoded but reporting is not implemented here. */
}
