#ifndef BMS_CAN_PROTOCOL_H
#define BMS_CAN_PROTOCOL_H
#include <stdint.h>

#define BMS_VCU_COMMAND_ID 0x150U
/* Three nominal 100 ms periods; validate allowed bus-loss latency on bench. */
#define BMS_VCU_COMMAND_MAX_AGE_MS 300U

typedef struct {
    uint8_t drive, balance, all_measurements, clear_error;
    uint8_t valid;
    uint32_t received_at;
} BMS_CanCommands;

/* Return 0 for unrelated traffic, 1 for accepted command, -1 for bad command. */
int BMS_CAN_decode_command(BMS_CanCommands *commands, uint32_t id,
                          uint8_t extended, uint8_t remote, uint8_t length,
                          const uint8_t *data, uint32_t now);
void BMS_CAN_expire_commands(BMS_CanCommands *commands, uint32_t now);
#endif
