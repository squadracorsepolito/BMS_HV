/* ============================================================================
 * Simulatore host: L9963T (TH, TL) + catena di L9963E, modellati sul datasheet.
 * Esegue il firmware REALE (L9963E_drv.c, L9963E.c, L9963_utils.c,
 * data_reading_timebase.c, ntc.c) sostituendo solo stm32_if.c (GPIO/SPI/tick).
 * Tempo virtuale in ns.
 * ==========================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "stm32_if.h"
#include "L9963E_drv.h"
#include "L9963_utils.h"
#include "sim.h"
#include "adc.h"
#include "tim.h"

SimParams P;
SimStats  S;
uint64_t now_ns = 0;

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim6;
uint8_t ams_error = 0;

/* ------------------------------ tempi DS ------------------------------ */
#define T_GPIO_NS        40
#define T_SETUP_LATCH_NS 1388   /* TRC_DELAY 700 + T_DEGLITCH 687.5 (DS L9963T Tab.11) */
#define BIT_SLOW_NS      3000   /* TBIT_LENGTH_SLOW 3 us */
#define BIT_FAST_NS      375    /* TBIT_LENGTH_FAST 375 ns */
#define TH_TWAKEUP_NS    1060000ULL
#define SPI_BYTE_NS      1422   /* 8 bit @ 5.625 MHz */

static uint64_t rnd_state = 1;
static uint32_t rnd(void) { rnd_state ^= rnd_state << 13; rnd_state ^= rnd_state >> 7; rnd_state ^= rnd_state << 17; return (uint32_t)rnd_state; }

/* ------------------------------ eventi ISO ----------------------------- */
enum { DST_SLAVE_L, DST_SLAVE_H, DST_TH, DST_TL };
typedef struct { uint64_t t; int dst; int idx; uint64_t frame; int bits; int fast; int used; } Ev;
#define MAXEV 4096
static Ev ev[MAXEV];

static int ev_count = 0; static uint64_t ev_min_t = UINT64_MAX;
static void ev_push(uint64_t t, int dst, int idx, uint64_t f, int bits, int fast) {
    if (t < ev_min_t) ev_min_t = t;
    for (int i = 0; i < MAXEV; ++i) if (!ev[i].used) { ev[i] = (Ev){t, dst, idx, f, bits, fast, 1}; ev_count++; return; }
    fprintf(stderr, "event overflow\n"); exit(2);
}

/* ------------------------------ transceiver ---------------------------- */
typedef struct {
    const char *name;
    int is_awake; uint64_t ready_at; int pending_wake;
    int cs, txen, isofreq, dis;
    uint64_t txen_t, isofreq_t; int txen_prev, isofreq_prev;
    int lat_txen, rx_fast, tx_fast, tx_fast_next;
    uint8_t mosi[16]; int mosi_n;
    uint64_t rxq[20]; int rxq_n;       /* MAX_RX_QUEUE 20 */
    int miso_active; int miso_pos; uint64_t miso_frame;
    uint64_t iso_busy_until, tx_end; int is_cmd;
} Trx;
static Trx TH, TL;

/* ------------------------------ slave ---------------------------------- */
enum { SL_SLEEP, SL_WAKING, SL_INIT, SL_NORMAL };
typedef struct {
    int state; uint64_t ready_at; uint64_t last_valid; uint64_t init_since;
    uint32_t reg[128];
    int fast;
    uint64_t conv_done_at; int conv_gpio;
    uint16_t cell_raw[14]; uint16_t gpio_raw[7];
} Slave;
static Slave SL[32]; /* fino a 31 slave indirizzabili (DS L9963E Tab. 16) */

static uint64_t fr_get(uint64_t v, int off, int w) { return (v >> off) & ((1ULL << w) - 1); }
#define F_CRC(v)   fr_get(v, 0, 6)
#define F_DATA(v)  fr_get(v, 6, 18)
#define F_ADDR(v)  fr_get(v, 26, 7)
#define F_DEVID(v) fr_get(v, 33, 5)
#define F_RW(v)    fr_get(v, 38, 1)
#define F_PA(v)    fr_get(v, 39, 1)

/* registri: bit di interesse */
#define R_DEVGEN 0x01
#define R_FASTCH 0x02
#define R_BAL1   0x03
#define R_BAL3   0x05
#define R_FSM    0x12
#define R_GPIOCF 0x14
#define R_VCELLEN 0x1C
#define R_ADCV   0x13   /* placeholder, riassegnato sotto con indirizzo vero */
static int ADDR_ADCV, ADDR_VCELL[15], ADDR_GPIOMEAS[10], ADDR_VBATTDIV, ADDR_VSUMBATT;

static uint32_t dev_chip(Slave *s)  { return (s->reg[R_DEVGEN] >> 13) & 0x1F; }
static uint32_t dev_isotx(Slave *s) { return (s->reg[R_DEVGEN] >> 12) & 1; }
static uint32_t dev_freq(Slave *s)  { return (s->reg[R_DEVGEN] >> 8) & 3; }
static uint32_t lock_bit(Slave *s)  { return (s->reg[R_BAL3] >> 15) & 1; }
static uint32_t commto(Slave *s)    { return (s->reg[R_FASTCH] >> 16) & 3; }

static void slave_por_main_reset(Slave *s) {
    /* reset source B (POR main): iso_freq_sel, Lock, HeartBeat_En, FaultL_force, ... */
    s->reg[R_DEVGEN] &= ~((3u << 8) | (1u << 2) | 1u);
    s->reg[R_BAL3] &= ~((1u << 15) | (1u << 16));
    s->fast = 0;
}
static void slave_full_reset(Slave *s) {
    memset(s->reg, 0, sizeof(s->reg));
    s->reg[R_DEVGEN] = 0x4u << 4;            /* HeartBeatCycle = 4 */
    s->reg[R_GPIOCF] = (2u << 14) | (2u << 12);
    s->reg[0x04] = 1u << 16;                 /* Balmode 01 */
    s->fast = 0;
}
static void slave_go_sleep(int k, const char *why) {
    Slave *s = &SL[k];
    if (s->state != SL_SLEEP) {
        if (S.phase_running) S.slave_sleeps_after_init++;
        if (P.verbose) printf("        [SIM] t=%.3f ms slave%d -> SLEEP (%s)\n", now_ns / 1e6, k + 1, why);
    }
    s->state = SL_SLEEP;
    slave_por_main_reset(s);
}

static uint32_t commto_ms(Slave *s) { static const uint32_t t[4] = {32, 256, 1024, 2048}; return t[commto(s)]; }

static void deliver_answer(int k, uint64_t t_end_rx, uint64_t ans) {
    /* risposta generata dopo TANSWER, trasmessa in entrambe le direzioni */
    Slave *s = &SL[k];
    int bit = s->fast ? BIT_FAST_NS : BIT_SLOW_NS;
    uint64_t t0 = t_end_rx + (s->fast ? 4500 : 9000);
    uint64_t t1 = t0 + 40ULL * bit;
    /* verso il basso (ISO_L): verso slave k-1 (porta H) oppure TH */
    if (k == 0) ev_push(t1, DST_TH, 0, ans, 40, s->fast);
    else        ev_push(t1 + bit, DST_SLAVE_H, k - 1, ans, 40, s->fast);
    /* verso l'alto (ISO_H) se abilitata */
    if (dev_isotx(s)) {
        if (k + 1 < P.n_slaves) ev_push(t1 + bit, DST_SLAVE_L, k + 1, ans, 40, s->fast);
        else if (P.dual_ring)   ev_push(t1 + bit, DST_TL, 0, ans, 40, s->fast);
    }
    /* collisione sul link TH<->slave1: TH sta trasmettendo mentre arriva la risposta? */
    if (k == 0 && TH.tx_end > t0 && !P.dual_ring) S.collisions++;
}

static uint64_t mk_answer(uint32_t devid, uint32_t addr, uint32_t data) {
    uint64_t v = 0;
    v |= (uint64_t)(data & 0x3FFFF) << 6;
    v |= (uint64_t)(addr & 0x7F) << 26;
    v |= (uint64_t)(devid & 0x1F) << 33;
    v |= 0ULL << 38; v |= 0ULL << 39;
    v |= L9963E_DRV_crc_calc(v);
    return v;
}

static void slave_update_conv(Slave *s) {
    if (s->conv_done_at && now_ns >= s->conv_done_at) {
        s->conv_done_at = 0;
        L9963E_RegisterUnionTypeDef r;
        r.generic = s->reg[ADDR_ADCV]; r.ADCV_CONV.SOC = 0; s->reg[ADDR_ADCV] = r.generic;
        for (int c = 1; c <= 14; ++c) { r.generic = 0; r.Vcell1.VCell1 = s->cell_raw[c - 1]; r.Vcell1.d_rdy_Vcell1 = 1; s->reg[ADDR_VCELL[c]] = r.generic & 0x3FFFF; }
        r.generic = 0; r.VBATTDIV.VBATT_DIV = 38000 & 0xFFFF; s->reg[ADDR_VBATTDIV] = r.generic & 0x3FFFF;
        s->reg[ADDR_VSUMBATT] = 0x100;
        if (s->conv_gpio)
            for (int g = 3; g <= 9; ++g) { r.generic = 0; r.GPIO3_MEAS.GPIO3_MEAS = s->gpio_raw[g - 3]; r.GPIO3_MEAS.d_rdy_gpio3 = 1; s->reg[ADDR_GPIOMEAS[g]] = r.generic & 0x3FFFF; }
    }
}

static void slave_write(Slave *s, uint32_t addr, uint32_t data, int bcast) {
    if (addr == R_DEVGEN) {
        uint32_t old = s->reg[R_DEVGEN];
        uint32_t nv  = data & ~(1u << 7);
        if (dev_chip(s) != 0) nv = (nv & ~(0x1Fu << 13)) | (old & (0x1Fu << 13));   /* chip_ID lock */
        if (lock_bit(s)) nv = (nv & ~((1u << 12) | (3u << 8))) | (old & ((1u << 12) | (3u << 8)));
        s->reg[R_DEVGEN] = nv;
        return;
    }
    if (addr == R_FSM) {
        uint32_t swrst = (data >> 14) & 3, go2slp = (data >> 12) & 3;
        if (swrst == 2) { uint32_t ct = s->reg[R_FASTCH] & (3u << 16); slave_full_reset(s); s->reg[R_FASTCH] |= ct; S.sw_resets++; }
        if (go2slp == 2) s->state = -1; /* marcato: va in sleep dopo il frame */
        return;
    }
    if (addr == ADDR_ADCV) {
        L9963E_RegisterUnionTypeDef r = {.generic = data};
        s->reg[addr] = data;
        if (r.ADCV_CONV.SOC) {
            s->conv_gpio   = r.ADCV_CONV.GPIO_CONV;
            s->conv_done_at = now_ns + (s->conv_gpio ? 3000000ULL : 1500000ULL);
            for (int c = 1; c <= 14; ++c) { L9963E_RegisterUnionTypeDef v = {.generic = s->reg[ADDR_VCELL[c]]}; v.Vcell1.d_rdy_Vcell1 = 0; s->reg[ADDR_VCELL[c]] = v.generic; }
            for (int g = 3; g <= 9; ++g) { L9963E_RegisterUnionTypeDef v = {.generic = s->reg[ADDR_GPIOMEAS[g]]}; v.GPIO3_MEAS.d_rdy_gpio3 = 0; s->reg[ADDR_GPIOMEAS[g]] = v.generic; }
        }
        return;
    }
    (void)bcast;
    s->reg[addr & 0x7F] = data & 0x3FFFF;
}

static void slave_rx(int k, uint64_t f, int fast, int from_below) {
    Slave *s = &SL[k];
    slave_update_conv(s);
    int bit = fast ? BIT_FAST_NS : BIT_SLOW_NS;
    if (s->state == SL_SLEEP) {
        /* >=8 impulsi entro 282 us: qualsiasi frame da 40 bit sveglia */
        s->state = SL_WAKING; s->ready_at = now_ns + (uint64_t)P.slave_wake_us * 1000ULL;
        if (P.verbose) printf("        [SIM] t=%.3f ms slave%d wakeup rilevato (pronta fra %u us)\n", now_ns / 1e6, k + 1, P.slave_wake_us);
        return;
    }
    if (s->state == SL_WAKING) return;
    if (fast != s->fast) { S.rate_mismatch_drops++; return; }
    int crc_ok = (F_CRC(f) == L9963E_DRV_crc_calc(f));
    /* ripetizione verso l'alto (comandi) se porta H abilitata */
    if (from_below && dev_isotx(s)) {
        if (k + 1 < P.n_slaves) ev_push(now_ns + bit, DST_SLAVE_L, k + 1, f, 40, fast);
        else if (P.dual_ring)   ev_push(now_ns + bit, DST_TL, 0, f, 40, fast);
    }
    if (!from_below) { /* risposta da slave superiore: ripeti verso il basso */
        if (k == 0) ev_push(now_ns + bit, DST_TH, 0, f, 40, fast);
        else        ev_push(now_ns + bit, DST_SLAVE_H, k - 1, f, 40, fast);
        return;
    }
    if (!crc_ok) { S.slave_crc_err++; return; }
    if (F_PA(f) != 1) return;
    uint32_t devid = F_DEVID(f), addr = F_ADDR(f), data = F_DATA(f), rw = F_RW(f);
    s->last_valid = now_ns;
    if (s->state == SL_INIT) {
        if (rw && addr == R_DEVGEN && (devid == 0 || devid == dev_chip(s))) {
            uint32_t chip = (data >> 13) & 0x1F;
            uint32_t keep = (1u << 12) | (3u << 8) | (0x1Fu << 13);
            s->reg[R_DEVGEN] = (s->reg[R_DEVGEN] & ~keep) | (data & keep);
            if (chip) { s->state = SL_NORMAL; S.addressed_at_ms = now_ns / 1e6;
                if (P.verbose) printf("        [SIM] t=%.3f ms slave%d INIT -> NORMAL, chip_ID=%u\n", now_ns / 1e6, k + 1, chip); }
        }
        s->fast = (dev_freq(s) == 3);
        return;
    }
    /* NORMAL */
    if (devid == 0) {
        if (rw) { slave_write(s, addr, data, 1); S.bcast_rx++; }
    } else if (devid == dev_chip(s)) {
        if (rw) slave_write(s, addr, data, 0);
        slave_update_conv(s);
        deliver_answer(k, now_ns, mk_answer(devid, addr, s->reg[addr & 0x7F]));
        S.answers++;
        if (!rw) { /* d_rdy: "0 = dato già letto una volta" → si azzera alla lettura */
            for (int c = 1; c <= 14; ++c) if ((int)addr == ADDR_VCELL[c]) { L9963E_RegisterUnionTypeDef v = {.generic = s->reg[addr]}; v.Vcell1.d_rdy_Vcell1 = 0; s->reg[addr] = v.generic; }
            for (int g = 3; g <= 9; ++g) if ((int)addr == ADDR_GPIOMEAS[g]) { L9963E_RegisterUnionTypeDef v = {.generic = s->reg[addr]}; v.GPIO3_MEAS.d_rdy_gpio3 = 0; s->reg[addr] = v.generic; }
        }
    }
    s->fast = (dev_freq(s) == 3);
    if (s->state == -1) { s->state = SL_NORMAL; slave_go_sleep(k, "GO2SLP"); }
}

static void trx_rx(Trx *t, uint64_t f, int fast) {
    if (!t->is_awake || now_ns < t->ready_at) return;
    if (fast != t->rx_fast) { S.rate_mismatch_drops++; return; }
    if (t->rxq_n < 20) t->rxq[t->rxq_n++] = f; else t->rxq[19] = f;
}

static void sim_process(void) {
    /* transceiver wake per DIS */
    Trx *ts[2] = {&TH, &TL};
    for (int i = 0; i < 2; ++i) {
        Trx *t = ts[i];
        if (!t->is_awake && t->dis == 0 && t->pending_wake && now_ns >= t->pending_wake) {
            t->is_awake = 1; t->ready_at = now_ns + TH_TWAKEUP_NS; t->pending_wake = 0;
            t->rx_fast = t->tx_fast = t->tx_fast_next = t->isofreq; /* latched al wakeup */
        }
    }
    while (ev_count > 0 && ev_min_t <= now_ns) {
        int best = -1;
        for (int i = 0; i < MAXEV; ++i) if (ev[i].used && ev[i].t <= now_ns && (best < 0 || ev[i].t < ev[best].t)) best = i;
        if (best < 0) { ev_min_t = UINT64_MAX; for (int i = 0; i < MAXEV; ++i) if (ev[i].used && ev[i].t < ev_min_t) ev_min_t = ev[i].t; break; }
        Ev e = ev[best]; ev[best].used = 0; ev_count--;
        ev_min_t = UINT64_MAX; for (int i = 0; i < MAXEV; ++i) if (ev[i].used && ev[i].t < ev_min_t) ev_min_t = ev[i].t;
        uint64_t save = now_ns; now_ns = e.t;
        if (e.dst == DST_SLAVE_L) slave_rx(e.idx, e.frame, e.fast, 1);
        else if (e.dst == DST_SLAVE_H) slave_rx(e.idx, e.frame, e.fast, 0);
        else if (e.dst == DST_TH) trx_rx(&TH, e.frame, e.fast);
        else trx_rx(&TL, e.frame, e.fast);
        now_ns = save;
    }
    for (int k = 0; k < P.n_slaves; ++k) {
        Slave *s = &SL[k];
        slave_update_conv(s);
        if (s->state == SL_WAKING && now_ns >= s->ready_at) {
            s->state = dev_chip(s) ? SL_NORMAL : SL_INIT; s->last_valid = now_ns; s->init_since = now_ns;
        }
        if (s->state == SL_NORMAL && now_ns - s->last_valid > (uint64_t)commto_ms(s) * 1000000ULL)
            slave_go_sleep(k, "CommTimeout scaduto");
        if (s->state == SL_INIT && now_ns - s->init_since > 60000000000ULL) slave_go_sleep(k, "t_SHUT");
    }
}
static void advance(uint64_t ns) { now_ns += ns; sim_process(); }

/* ------------------------------ pin model ------------------------------ */
static int latched(int pin, int prev, uint64_t tchg) {
    if (now_ns - tchg >= T_SETUP_LATCH_NS) return pin;
    S.setup_violations++;
    if (S.setup_violations <= 5) printf("        [SIM-DBG] violazione setup a t=%.6f ms: dt=%llu ns pin=%d prev=%d\n", now_ns/1e6, (unsigned long long)(now_ns - tchg), pin, prev);
    return prev;   /* caso peggiore: il deglitch vede ancora il valore vecchio */
}
static void trx_ncs_fall(Trx *t) {
    t->mosi_n = 0; t->is_cmd = 0;
    if (!t->is_awake || now_ns < t->ready_at) { t->miso_active = 0; t->lat_txen = 0; return; }
    t->lat_txen = latched(t->txen, t->txen_prev, t->txen_t);
    int f = latched(t->isofreq, t->isofreq_prev, t->isofreq_t);
    t->rx_fast = f; t->tx_fast_next = f;
    if (t->rxq_n) { t->miso_active = 1; t->miso_frame = t->rxq[0]; t->miso_pos = 0; } else t->miso_active = 0;
}
static void trx_ncs_rise(Trx *t, int is_th) {
    if (!t->is_awake || now_ns < t->ready_at) { if (t->mosi_n) S.frames_to_sleeping_trx++; return; }
    if (t->miso_active && t->miso_pos >= 5) { memmove(t->rxq, t->rxq + 1, (t->rxq_n - 1) * sizeof(uint64_t)); t->rxq_n--; }
    t->miso_active = 0;
    int bits = t->mosi_n * 8;
    if (bits >= 8 && bits <= 64) {
        if (t->lat_txen) {
            uint64_t f = 0; for (int i = 0; i < t->mosi_n && i < 5; ++i) f = (f << 8) | t->mosi[i];
            int fast = t->tx_fast; int bit = fast ? BIT_FAST_NS : BIT_SLOW_NS;
            uint64_t start = now_ns > t->iso_busy_until ? now_ns : t->iso_busy_until;
            uint64_t end = start + (uint64_t)bits * bit;
            t->iso_busy_until = end + 4ULL * bit; t->tx_end = end;
            t->tx_fast = t->tx_fast_next;
            if (is_th) {
                ev_push(end, DST_SLAVE_L, 0, f, bits, fast); S.th_frames_tx++;
                if (F_CRC(f) != L9963E_DRV_crc_calc(f) && !(t->mosi[0] == 0x55 && t->mosi[1] == 0x55)) { S.garbage_tx++;
                    if (S.garbage_tx <= 5) printf("        [SIM-DBG] garbage TX t=%.6f ms frame=%010llx txen_pin=%d\n", now_ns/1e6, (unsigned long long)f, t->txen); }
            } else S.tl_frames_tx++;
        } else {
            t->tx_fast = t->tx_fast_next;
            if (t->is_cmd && is_th && S.expect_tx) S.cmd_dropped_txen_low++;
        }
    }
}

static Trx *trx_of(int is_th) { return is_th ? &TH : &TL; }
static void pin_write(int is_th, L9963E_IF_PINS pin, L9963E_IF_PinState v) {
    Trx *t = trx_of(is_th); advance(T_GPIO_NS);
    int val = v == L9963E_IF_GPIO_PIN_SET;
    switch (pin) {
        case L9963E_IF_CS:
            if (t->cs && !val) trx_ncs_fall(t);
            if (!t->cs && val) trx_ncs_rise(t, is_th);
            t->cs = val; break;
        case L9963E_IF_TXEN: if (t->txen != val) { t->txen_prev = t->txen; t->txen = val; t->txen_t = now_ns; } break;
        case L9963E_IF_ISOFREQ: if (t->isofreq != val) { t->isofreq_prev = t->isofreq; t->isofreq = val; t->isofreq_t = now_ns; } break;
        case L9963E_IF_DIS:
            t->dis = val;
            if (!val && !t->is_awake) t->pending_wake = now_ns + 1400;
            if (val && t->is_awake) { t->is_awake = 0; t->rxq_n = 0; }
            break;
        default: break;
    }
}
static L9963E_IF_PinState pin_read(int is_th, L9963E_IF_PINS pin) {
    Trx *t = trx_of(is_th); advance(T_GPIO_NS);
    int v = 0;
    switch (pin) {
        case L9963E_IF_CS: v = t->cs; break;
        case L9963E_IF_TXEN: v = t->txen; break;
        case L9963E_IF_ISOFREQ: v = t->isofreq; break;
        case L9963E_IF_DIS: v = t->dis; break;
        case L9963E_IF_BNE: v = t->is_awake && now_ns >= t->ready_at && t->rxq_n > 0; break;
    }
    return v ? L9963E_IF_GPIO_PIN_SET : L9963E_IF_GPIO_PIN_RESET;
}
static L9963E_StatusTypeDef spi_tx(int is_th, uint8_t *d, uint8_t n) {
    Trx *t = trx_of(is_th);
    advance(1000 + (uint64_t)n * SPI_BYTE_NS);
    if (t->cs) return L9963E_OK; /* CS alto: niente */
    t->is_cmd = 1;
    for (int i = 0; i < n; ++i) if (t->mosi_n < 16) t->mosi[t->mosi_n++] = d[i];
    if (t->miso_active) t->miso_pos += n;
    return L9963E_OK;
}
static L9963E_StatusTypeDef spi_rx(int is_th, uint8_t *d, uint8_t n) {
    Trx *t = trx_of(is_th);
    /* HAL_SPI_Receive in full-duplex manda il buffer stesso su MOSI */
    uint8_t mosi[16]; memcpy(mosi, d, n);
    advance(1000 + (uint64_t)n * SPI_BYTE_NS);
    for (int i = 0; i < n; ++i) {
        if (t->miso_active) d[i] = (uint8_t)(t->miso_frame >> (8 * (4 - i)));
        else d[i] = 0xFF;
        if (t->mosi_n < 16) t->mosi[t->mosi_n++] = mosi[i];
    }
    if (t->miso_active) t->miso_pos += n;
    if (P.inject_noise_at_ms && now_ns / 1000000ULL >= P.inject_noise_at_ms && !S.noise_done && t->miso_active) { d[2] ^= 0x10; S.noise_done = 1; printf("        [SIM] t=%.3f ms RUMORE: frame di risposta corrotto sulla SPI\n", now_ns / 1e6); }
    return L9963E_OK;
}

/* ------------------------------ stm32_if -------------------------------- */
L9963E_IF_PinState L9963TH_GPIO_ReadPin(L9963E_IF_PINS p) { return pin_read(1, p); }
L9963E_StatusTypeDef L9963TH_GPIO_WritePin(L9963E_IF_PINS p, L9963E_IF_PinState s) { pin_write(1, p, s); return L9963E_OK; }
L9963E_StatusTypeDef L9963TH_SPI_Receive(uint8_t *d, uint8_t n, uint8_t to) { (void)to; return spi_rx(1, d, n); }
L9963E_StatusTypeDef L9963TH_SPI_Transmit(uint8_t *d, uint8_t n, uint8_t to) { (void)to; return spi_tx(1, d, n); }
L9963E_IF_PinState L9963TL_GPIO_ReadPin(L9963E_IF_PINS p) { return pin_read(0, p); }
L9963E_StatusTypeDef L9963TL_GPIO_WritePin(L9963E_IF_PINS p, L9963E_IF_PinState s) { pin_write(0, p, s); return L9963E_OK; }
L9963E_StatusTypeDef L9963TL_SPI_Receive(uint8_t *d, uint8_t n, uint8_t to) { (void)to; return spi_rx(0, d, n); }
L9963E_StatusTypeDef L9963TL_SPI_Transmit(uint8_t *d, uint8_t n, uint8_t to) { (void)to; return spi_tx(0, d, n); }
uint32_t GetTickMs(void) { advance(20); return (uint32_t)(now_ns / 1000000ULL); }
void DelayMs(uint32_t d) {   /* semantica HAL_Delay: attende d+1 tick */
    uint32_t start = GetTickMs(); uint32_t wait = d + 1;
    uint64_t target = (uint64_t)(start + wait) * 1000000ULL;
    while (now_ns < target) { uint64_t step = target - now_ns; if (step > 50000) step = 50000; advance(step); }
}

/* --------------------------- setup scenario ------------------------------ */
void sim_setup(void) {
    memset(ev, 0, sizeof(ev)); ev_count = 0; ev_min_t = UINT64_MAX; memset(&TH, 0, sizeof(TH)); memset(&TL, 0, sizeof(TL)); memset(SL, 0, sizeof(SL));
    memset(&S, 0, sizeof(S));
    rnd_state = P.seed * 2654435761ULL + 7;
    now_ns = (uint64_t)(rnd() % 1000) * 1000ULL;   /* fase casuale del tick */
    /* indirizzi registri reali dalla libreria */
    ADDR_ADCV = L9963E_ADCV_CONV_ADDR; ADDR_VBATTDIV = L9963E_VBATTDIV_ADDR; ADDR_VSUMBATT = L9963E_VSUMBATT_ADDR;
    L9963E_RegistersAddrTypeDef vc[15] = {0, L9963E_Vcell1_ADDR, L9963E_Vcell2_ADDR, L9963E_Vcell3_ADDR, L9963E_Vcell4_ADDR,
        L9963E_Vcell5_ADDR, L9963E_Vcell6_ADDR, L9963E_Vcell7_ADDR, L9963E_Vcell8_ADDR, L9963E_Vcell9_ADDR, L9963E_Vcell10_ADDR,
        L9963E_Vcell11_ADDR, L9963E_Vcell12_ADDR, L9963E_Vcell13_ADDR, L9963E_Vcell14_ADDR};
    for (int i = 1; i <= 14; ++i) ADDR_VCELL[i] = vc[i];
    L9963E_RegistersAddrTypeDef gm[10] = {0, 0, 0, L9963E_GPIO3_MEAS_ADDR, L9963E_GPIO4_MEAS_ADDR, L9963E_GPIO5_MEAS_ADDR,
        L9963E_GPIO6_MEAS_ADDR, L9963E_GPIO7_MEAS_ADDR, L9963E_GPIO8_MEAS_ADDR, L9963E_GPIO9_MEAS_ADDR};
    for (int i = 3; i <= 9; ++i) ADDR_GPIOMEAS[i] = gm[i];
    /* MX_GPIO_Init: DIS rilasciati (alto), TXEN basso, ISOFREQ basso, NCS basso */
    TH.dis = TL.dis = 1; TH.cs = TL.cs = 0; TH.name = "TH"; TL.name = "TL";
    for (int k = 0; k < P.n_slaves; ++k) {
        Slave *s = &SL[k]; slave_full_reset(s);
        for (int c = 0; c < 14; ++c) s->cell_raw[c] = (uint16_t)((3600 + 10 * c + k * 5) * 1000 / 89);
        for (int g = 0; g < 7; ++g) s->gpio_raw[g] = (uint16_t)((1800 + 50 * g) * 1000 / 89);
        s->state = SL_SLEEP;
        if (P.slave_initial == INIT_AWAKE_FAST_LOCKED) {
            s->state = SL_NORMAL; s->reg[R_DEVGEN] = (s->reg[R_DEVGEN] & ~(0x1Fu << 13)) | ((uint32_t)(k + 1) << 13) | (3u << 8);
            s->reg[R_BAL3] |= 1u << 15; s->reg[R_FASTCH] |= 3u << 16; s->fast = 1; s->last_valid = now_ns;
        } else if (P.slave_initial == INIT_AWAKE_INIT_SLOW) {
            s->state = SL_INIT; s->init_since = now_ns; s->last_valid = now_ns;
        } else if (P.slave_initial == INIT_SLEEP_WITH_CHIPID) {
            s->reg[R_DEVGEN] |= (uint32_t)(k + 1) << 13;
        }
    }
}

int sim_check_slave(int k, int dual) {
    Slave *s = &SL[k]; int ok = 1;
    #define CHK(c, msg) do { if (!(c)) { printf("   CHECK FALLITO: %s\n", msg); ok = 0; } } while (0)
    CHK(s->state == SL_NORMAL, "slave non in NORMAL");
    CHK(dev_chip(s) == (uint32_t)(k + 1), "chip_ID errato");
    /* Non si pretende piu' una frequenza specifica: in messa in servizio il firmware resta
     * legittimamente a 333 kbps. L'invariante vero e' che slave e transceiver del master siano
     * alla STESSA velocita', altrimenti non si capiscono (DS L9963T §4.4: "The new bit rate of
     * L9963T must be compatible with the one of all other units communicating on the bus").
     * Questo check trova il guasto reale in entrambi i versi, quello vecchio no. */
    CHK(s->fast == (dev_freq(s) == 3), "iso_freq_sel della slave incoerente col suo bit-rate");
    CHK(s->fast == TH.tx_fast, "slave e TH del master a bit-rate DIVERSI: non si capirebbero");
    (void)lock_bit; /* Lock_isoh_isofreq non piu' imposto: bloccarlo impedisce il recupero
                       dopo un reset del solo MCU (si azzera solo andando in low power). */
    CHK(commto(s) == 3, "CommTimeout != 2048 ms (broadcast perso?)");
    CHK((s->reg[R_VCELLEN] & 0x3FFF) == 0x38FF, "VCELLS_EN errato (broadcast perso?)");
    CHK(((s->reg[R_GPIOCF] >> 12) & 3) == 2 && ((s->reg[R_GPIOCF] >> 14) & 3) == 0, "GPIO7/GPIO8 config errata");
    if (k == P.n_slaves - 1) {
        CHK(dev_isotx(s) == (uint32_t)(dual ? 1 : 0), "isotx_en_h ultima slave errato");
        if (!dual) CHK(((s->reg[R_DEVGEN] >> 1) & 1) == 1, "Farthest_Unit non impostato");
    } else CHK(dev_isotx(s) == 1, "isotx_en_h slave intermedia != 1");
    return ok;
}
uint16_t sim_cell_raw(int k, int c) { return SL[k].cell_raw[c]; }
uint16_t sim_gpio_raw(int k, int g) { return SL[k].gpio_raw[g]; }
void sim_advance_ms(uint32_t ms) { DelayMs(ms ? ms - 1 : 0); }
