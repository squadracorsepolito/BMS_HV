#include "bms_can_protocol.h"
#include <stddef.h>

int BMS_CAN_decode_command(BMS_CanCommands *commands, uint32_t id,
                          uint8_t extended, uint8_t remote, uint8_t length,
                          const uint8_t *data, uint32_t now) {
    if (commands == NULL) return -1;
    if (id != BMS_VCU_COMMAND_ID || extended) return 0;
    if (remote || length != 2 || data == NULL ||
        ((data[0] & 1U) && (data[1] & 1U))) {
        *commands = (BMS_CanCommands){0};
        return -1;
    }
    /* DBC Motorola single bits: 0, 6, 8 and 14. */
    *commands = (BMS_CanCommands){
        .drive = data[0] & 1U,
        .balance = data[1] & 1U,
        .clear_error = (data[0] >> 6) & 1U,
        .all_measurements = (data[1] >> 6) & 1U,
        .valid = 1,
        .received_at = now
    };
    return 1;
}

void BMS_CAN_expire_commands(BMS_CanCommands *commands, uint32_t now) {
    if (commands != NULL && commands->valid &&
        now - commands->received_at >= BMS_VCU_COMMAND_MAX_AGE_MS)
        *commands = (BMS_CanCommands){0};
}
