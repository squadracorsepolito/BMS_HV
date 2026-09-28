#ifndef SIM_H
#define SIM_H
#include <stdint.h>
enum { INIT_SLEEP = 0, INIT_AWAKE_FAST_LOCKED, INIT_AWAKE_INIT_SLOW, INIT_SLEEP_WITH_CHIPID };
typedef struct { uint32_t seed; int n_slaves; int dual_ring; uint32_t slave_wake_us; int slave_initial; int verbose;
                 uint32_t run_seconds; uint32_t inject_noise_at_ms; } SimParams;
typedef struct { uint32_t setup_violations, cmd_dropped_txen_low, garbage_tx, collisions, rate_mismatch_drops,
                 slave_crc_err, answers, bcast_rx, th_frames_tx, tl_frames_tx, frames_to_sleeping_trx, sw_resets,
                 slave_sleeps_after_init; int phase_running; int expect_tx; double addressed_at_ms; int noise_done; } SimStats;
extern SimParams P; extern SimStats S; extern uint64_t now_ns;
void sim_setup(void);
int sim_check_slave(int k, int dual);
uint16_t sim_cell_raw(int k, int c);
uint16_t sim_gpio_raw(int k, int g);
void sim_advance_ms(uint32_t ms);
#endif
