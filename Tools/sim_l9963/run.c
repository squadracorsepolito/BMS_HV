#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"
#include "L9963_utils.h"
#include "data_reading_timebase.h"
#include "ntc.h"
#include "stm32_if.h"

extern volatile uint16_t vcells[N_SLAVES][N_CELLS_PER_SLAVE];
extern volatile uint16_t vgpio[N_SLAVES][N_GPIOS_PER_SLAVE];
extern L9963E_HandleTypeDef hl9963e;

static const char *names[] = {"slave in SLEEP (avvio a freddo)", "MCU resettato con slave SVEGLIA in ISO veloce + lock",
                              "slave sveglia in INIT (lenta, senza indirizzo)", "slave in SLEEP con chip_ID gia' assegnato"};

int main(int argc, char **argv) {
    memset(&P, 0, sizeof(P));
    P.seed = argc > 1 ? atoi(argv[1]) : 1;
    P.slave_initial = argc > 2 ? atoi(argv[2]) : 0;
    P.slave_wake_us = argc > 3 ? atoi(argv[3]) : 2000;
    P.run_seconds   = argc > 4 ? atoi(argv[4]) : 12;
    P.inject_noise_at_ms = argc > 5 ? atoi(argv[5]) : 0;
    P.verbose       = argc > 6 ? atoi(argv[6]) : 1;
#ifdef DUAL
    P.dual_ring = 1;
#endif
    P.n_slaves = N_SLAVES;
    printf("### SCENARIO seed=%u | %s | risveglio slave %u us | durata %u s%s | %s\n", P.seed, names[P.slave_initial],
           P.slave_wake_us, P.run_seconds, P.inject_noise_at_ms ? " | rumore SPI iniettato" : "", P.dual_ring ? "DOPPIO ANELLO" : "ANELLO SINGOLO");
    sim_setup();

    /* === main.c === */
    DelayMs(50);
    S.expect_tx = 1;
    L9963E_utils_init();
    data_reading_timebase_init();
    ntc_init();
    S.phase_running = 1;
    uint32_t t_init_end = GetTickMs();

    uint32_t next_read = GetTickMs() + 100, next_print = GetTickMs() + 2000;
    uint32_t end = GetTickMs() + P.run_seconds * 1000;
    while (GetTickMs() < end) {
        if (GetTickMs() >= next_read) { next_read += 100; data_reading_l9963e_cb(); if (GetTickMs() > next_read) next_read = GetTickMs(); }
        if (GetTickMs() >= next_print) { next_print += 2000; if (P.verbose) L9963E_utils_debug_print(); }
        DelayMs(0);
    }

    /* === verifiche === */
    int ok = 1;
    printf("\n=== VERIFICHE (init terminata a t=%u ms) ===\n", t_init_end);
    #define V(c, msg) do { int _c = (c); printf("  [%s] %s\n", _c ? " OK " : "FAIL", msg); ok &= _c; } while (0)
    V(l9963_addressing_ok == 1, "wakeup + addressing riusciti");
    for (int k = 0; k < N_SLAVES; ++k) V(sim_check_slave(k, P.dual_ring), "registri slave: chip_ID, ISO veloce, lock, CommTimeout 2048, celle, GPIO, Farthest/isotx");
    int cells_ok = 1, gpio_ok = 1;
    static const int cidx[11] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 13};
    static const int gidx[6] = {0, 1, 2, 3, 5, 6};
    for (int k = 0; k < N_SLAVES; ++k) {
        for (int c = 0; c < 11; ++c) if (vcells[k][c] != sim_cell_raw(k, cidx[c])) cells_ok = 0;
        for (int g = 0; g < 6; ++g) if (vgpio[k][g] != sim_gpio_raw(k, gidx[g])) gpio_ok = 0;
    }
    V(cells_ok, "11 tensioni cella lette = valori reali della slave");
    V(gpio_ok && l9963_gpio_valid, "6 NTC (GPIO3,4,5,6,8,9) lette = valori reali della slave");
    V(S.slave_sleeps_after_init == 0, "la slave NON si e' mai addormentata dopo l'init");
    V(S.setup_violations == 0, "nessuna violazione setup TXEN/ISOFREQ prima di NCS (1.4 us)");
    V(S.cmd_dropped_txen_low == 0, "nessun comando scartato dal TH per TXEN campionato LOW");
    V(S.garbage_tx == 0, "nessun frame spazzatura trasmesso sull'ISO");
    V(S.collisions == 0, "nessuna collisione sull'ISO (risposta mentre TH trasmette)");
    V(P.inject_noise_at_ms ? l9963_comm_errors <= 2 : l9963_comm_errors == 0, "errori di comunicazione (letture fallite)");
    printf("  statistiche: cicli lettura=%u errori=%u frame TH->ISO=%u risposte slave=%u broadcast ricevuti=%u sw_reset=%u freq-mismatch scartati=%u\n",
           l9963_read_cycles, l9963_comm_errors, S.th_frames_tx, S.answers, S.bcast_rx, S.sw_resets, S.rate_mismatch_drops);
    printf("RISULTATO: %s\n\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
