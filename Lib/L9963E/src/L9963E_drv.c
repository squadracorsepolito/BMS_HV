/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 * - Filippo Rossi [filippo.rossi.sc@gmail.com]
 */

#include "L9963E_drv.h"

#include <stddef.h>
#include <string.h>

#define WORD_LEN           40UL
#define CRC_LEN            6UL
#define CRC_POLY           (uint64_t)0x0000000000000059 /*L9963 CRC Poly:x^3+x^2+x+1*/
#define CRC_SEED           (uint64_t)0x0000000000000038
#define CRC_INIT_SEED_MASK (CRC_SEED << (WORD_LEN - CRC_LEN))
#define CRC_INIT_MASK      (CRC_POLY << (WORD_LEN - CRC_LEN - 1))
#define FIRST_BIT_MASK     ((uint64_t)1 << (WORD_LEN - 1))  // 0x80000000
#define CRC_LOWER_MASK     ((uint8_t)(1 << CRC_LEN) - 1)    //0b111

uint8_t crc6_lut[64] = {0x0,  0x19, 0x32, 0x2b, 0x3d, 0x24, 0xf,  0x16, 0x23, 0x3a, 0x11, 0x8,  0x1e, 0x7,  0x2c, 0x35,
                        0x1f, 0x6,  0x2d, 0x34, 0x22, 0x3b, 0x10, 0x9,  0x3c, 0x25, 0xe,  0x17, 0x1,  0x18, 0x33, 0x2a,
                        0x3e, 0x27, 0xc,  0x15, 0x3,  0x1a, 0x31, 0x28, 0x1d, 0x4,  0x2f, 0x36, 0x20, 0x39, 0x12, 0xb,
                        0x21, 0x38, 0x13, 0xa,  0x1c, 0x5,  0x2e, 0x37, 0x2,  0x1b, 0x30, 0x29, 0x3f, 0x26, 0xd,  0x14};

uint8_t L9963E_DRV_crc_calc(uint64_t InputWord) {
    uint64_t TestBitMask;
    uint64_t CRCMask;
    uint8_t BitCount;
    uint8_t crc = 0;

    InputWord = (InputWord & 0xFFFFFFFFC0) ^ CRC_INIT_SEED_MASK; /* Clear the CRC bit in the data frame*/

    // first 4 bit executed as standard crc (shift and xor)
    // in order to reach a multiple of CRC_LEN data length
    TestBitMask = FIRST_BIT_MASK;
    CRCMask     = CRC_INIT_MASK;  // 1111 <<
    BitCount    = WORD_LEN % CRC_LEN;
    while (0 != BitCount--) {
        if (0 != (InputWord & TestBitMask)) {
            InputWord ^= CRCMask;
        } /* endif */
        CRCMask >>= 1;
        TestBitMask >>= 1;
    } /* endwhile */

    // then proceed with a lut-based calculation
    for (int8_t i = WORD_LEN - (WORD_LEN % CRC_LEN) - CRC_LEN; i > 0; i -= CRC_LEN) {
        crc = crc6_lut[((InputWord >> i) & 0b111111) ^ crc];
    }

    return crc;
}

L9963E_StatusTypeDef L9963E_DRV_init(L9963E_DRV_HandleTypeDef *handle, L9963E_IfTypeDef interface) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (interface.L9963E_IF_GPIO_ReadPin == NULL) {
        return L9963E_ERROR;
    }

    if (interface.L9963E_IF_GPIO_WritePin == NULL) {
        return L9963E_ERROR;
    }

    if (interface.L9963E_IF_SPI_Receive == NULL) {
        return L9963E_ERROR;
    }

    if (interface.L9963E_IF_SPI_Transmit == NULL) {
        return L9963E_ERROR;
    }

    if (interface.L9963E_IF_GetTickMs == NULL) {
        return L9963E_ERROR;
    }

    if (interface.L9963E_IF_DelayMs == NULL) {
        return L9963E_ERROR;
    }
#endif

    handle->interface    = interface;
    handle->is_dual_ring = 0;  /* anello singolo per default */

    L9963E_DRV_CS_HIGH(handle);
    L9963E_DRV_TXEN_HIGH(handle);
    L9963E_DRV_ISOFREQ_LOW(handle);
    L9963E_DRV_DIS_LOW(handle);

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_DRV_init_dual_ring(L9963E_DRV_HandleTypeDef *handle,
                                               L9963E_IfTypeDef tx_interface,
                                               L9963E_IfTypeDef rx_interface) {
#if L9963E_DEBUG
    if (handle == NULL)                          return L9963E_ERROR;
    if (rx_interface.L9963E_IF_GPIO_ReadPin  == NULL) return L9963E_ERROR;
    if (rx_interface.L9963E_IF_GPIO_WritePin == NULL) return L9963E_ERROR;
    if (rx_interface.L9963E_IF_SPI_Receive   == NULL) return L9963E_ERROR;
    if (rx_interface.L9963E_IF_GetTickMs     == NULL) return L9963E_ERROR;
    if (rx_interface.L9963E_IF_DelayMs       == NULL) return L9963E_ERROR;
#endif

    /* Inizializza lato TX (TH) tramite L9963E_DRV_init → is_dual_ring=0 temporaneo */
    L9963E_StatusTypeDef ret = L9963E_DRV_init(handle, tx_interface);
    if (ret != L9963E_OK) return ret;

    handle->rx_interface = rx_interface;
    handle->is_dual_ring = 1;

    /* Inizializza TL (rx_interface): NCS deasserted, TXEN=LOW (sempre in ricezione),
     * ISOFREQ=LOW (slow mode, verrà aggiornato da addressing_procedure), DIS=LOW (attivo) */
    L9963E_DRV_RX_CS_HIGH(handle);
    L9963E_DRV_RX_TXEN_LOW(handle);      /* TL non trasmette mai su ISO_L */
    L9963E_DRV_RX_ISOFREQ_LOW(handle);
    L9963E_DRV_RX_WRITE_PIN(handle, L9963E_IF_DIS, L9963E_IF_GPIO_PIN_RESET); /* DIS=LOW: TL attivo */

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_DRV_wakeup(L9963E_DRV_HandleTypeDef *handle) {
    uint8_t dummy[5]               = {0x55, 0x55, 0x55, 0x55, 0x55};
    L9963E_IF_PinState isofreq     = L9963E_IF_GPIO_PIN_RESET;
    L9963E_StatusTypeDef errorcode = L9963E_OK;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    isofreq = L9963E_DRV_ISOFREQ_READ(handle);

    /* Il wakeup va inviato in bassa frequenza e con TX abilitato.
     * L9963T campiona TXEN e ISOFREQ sul fronte di discesa di NCS e li vuole stabili
     * per TRC_DELAY (700 ns) + T_DEGLITCH (687.5 ns) ≈ 1.4 us prima (DS L9963T Tab. 11):
     * 1 ms di attesa copre ampiamente questo vincolo. */
    L9963E_DRV_ISOFREQ_LOW(handle);
    L9963E_DRV_TXEN_HIGH(handle);
    L9963E_DRV_DELAY(handle, 1);

    L9963E_DRV_CS_LOW(handle);
    errorcode = L9963E_DRV_SPI_TRANSMIT(handle, (uint8_t *)dummy, sizeof(dummy), 10);
    L9963E_DRV_CS_HIGH(handle);

    /* Ripristina subito i pin (sono già stati campionati sul fronte di NCS↓) e poi attende:
     * così restano stabili >1.4 us prima del prossimo NCS↓. */
    L9963E_DRV_WRITE_PIN(handle, L9963E_IF_ISOFREQ, isofreq);
    if (!handle->is_dual_ring) {
        L9963E_DRV_TXEN_LOW(handle); /* anello singolo: TXEN a riposo LOW */
    }
    L9963E_DRV_DELAY(handle, 1);

    return errorcode;
}

void _L9963E_DRV_switch_endianness(uint8_t *in, uint8_t *out) {
    out[0] = in[4];
    out[1] = in[3];
    out[2] = in[2];
    out[3] = in[1];
    out[4] = in[0];

    return;
}

L9963E_StatusTypeDef _L9963E_DRV_build_frame(uint8_t *out,
                                             uint8_t pa,
                                             uint8_t rw_burst,
                                             uint8_t devid,
                                             uint8_t addr_command,
                                             uint32_t data) {
    union L9963E_DRV_FrameUnion frame;
    frame.cmd.pa       = pa;
    frame.cmd.rw_burst = rw_burst;
    frame.cmd.devid    = devid;
    frame.cmd.addr     = addr_command;
    frame.cmd.data     = data;
    frame.cmd.crc      = L9963E_DRV_crc_calc(frame.val);

    _L9963E_DRV_switch_endianness((uint8_t *)&frame.val, out);

    return L9963E_OK;
}

L9963E_StatusTypeDef _L9963E_DRV_spi_transmit(L9963E_DRV_HandleTypeDef *handle,
                                              uint8_t *data,
                                              uint8_t len,
                                              uint8_t timeout) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;

    /* Semantica TXEN (DS L9963T §3.2.1.6, NSLAVE=0):
     *  - TXEN viene CAMPIONATO sul fronte di DISCESA di NCS.
     *  - TXEN=1 → il frame SPI entra nella TX queue e viene trasmesso su ISO.
     *  - TXEN=0 → i dati su SDI sono scartati (si usa per leggere la RX queue).
     *  - TXEN (e ISOFREQ) devono essere stabili per ≈1.4 us PRIMA di NCS↓
     *    (TRC_DELAY 700 ns + TTXEN_DEGLITCH 687.5 ns), e TTXEN_HOLD = 300 ns dopo.
     * Anello singolo: TXEN a riposo è LOW → va alzato e bisogna ATTENDERE prima di NCS↓,
     * altrimenti il TH campiona TXEN=0 e il comando viene scartato in silenzio.
     * Il delay copre anche il setup di ISOFREQ quando addressing_procedure lo cambia. */
    if (!handle->is_dual_ring) {
        L9963E_DRV_TXEN_HIGH(handle);
    }
    L9963E_DRV_DELAY(handle, 1);

    L9963E_DRV_CS_LOW(handle);
    errorcode = L9963E_DRV_SPI_TRANSMIT(handle, data, len, timeout);
    L9963E_DRV_CS_HIGH(handle);

    if (!handle->is_dual_ring) {
        /* Anello singolo: TXEN torna LOW SUBITO (TTXEN_HOLD=300 ns dopo NCS↓ è già rispettato:
         * l'SPI è durato ~7 us). Così durante l'attesa qui sotto TXEN resta LOW stabile per 1 ms
         * e la successiva lettura della RX queue (NCS↓, byte dummy su MOSI) viene campionata con
         * TXEN=0 → i byte dummy sono scartati e NON escono sull'ISO. */
        L9963E_DRV_TXEN_LOW(handle);
    }
    /* Doppio anello: TH resta TXEN=HIGH (solo TX), le risposte si leggono dal TL. */

    /* SPI 5.6 MHz → 5 byte in ~7 us; ISO lento (333 kbps) → 40 bit in ~120 us.
     * 1 ms: il frame esce sull'ISO, la risposta ha tempo di arrivare e TXEN/ISOFREQ
     * sono stabili ben oltre gli 1.4 us richiesti prima del prossimo NCS↓. */
    L9963E_DRV_DELAY(handle, 1);

    return errorcode;
}

L9963E_StatusTypeDef _L9963E_DRV_wait_and_receive(union L9963E_DRV_FrameUnion *frame,
                                                  L9963E_DRV_HandleTypeDef *handle,
                                                  uint8_t device,
                                                  uint32_t current_tick,
                                                  uint8_t timeout) {
    uint8_t raw[5];
    L9963E_StatusTypeDef errorcode = L9963E_OK;

    frame->cmd.addr  = -1;
    frame->cmd.devid = -1;
    frame->cmd.data  = -1;

    if (!handle->is_dual_ring) {
        /* Anello singolo: TXEN deve essere LOW durante la lettura della RX queue del TH,
         * così i byte dummy su MOSI vengono scartati (già LOW da spi_transmit, qui per sicurezza). */
        L9963E_DRV_TXEN_LOW(handle);
    }
    /* Dual-ring: TH rimane TXEN=HIGH (solo TX). TL è fisso TXEN=LOW (solo RX):
     * la risposta slave arriva su ISO_H della slave → ISO_L del TL → SPI2. */

    /* Accetta solo frame di RISPOSTA (P.A.=0, DS L9963E Tab. 20) del device atteso.
     * In doppio anello il TL riceve anche i COMANDI (P.A.=1) ripetuti dalle slave verso l'alto:
     * senza il controllo su pa un comando di lettura (stesso devid) verrebbe preso per risposta. */
    frame->cmd.pa = 1;
    while (frame->cmd.devid != device || frame->cmd.pa != 0) {
        /* Poll BNE: dual-ring → TL BNE (rx_interface); singolo → TH BNE */
        while ((handle->is_dual_ring ? L9963E_DRV_RX_BNE_READ(handle)
                                     : L9963E_DRV_BNE_READ(handle)) == L9963E_IF_GPIO_PIN_RESET) {
            if (L9963E_DRV_GETTICK(handle) - current_tick >= timeout) {
                return L9963E_TIMEOUT; /* TXEN resta LOW (riposo anello singolo) */
            }
        }

        if (handle->is_dual_ring) {
            /* Leggi frame via TL (SPI2) */
            L9963E_DRV_RX_CS_LOW(handle);
            errorcode = L9963E_DRV_RX_SPI_RECEIVE(handle, raw, 5, 10);
            L9963E_DRV_RX_CS_HIGH(handle);
        } else {
            /* Leggi frame via TH (SPI3) */
            /* TXEN resta LOW anche fra frame successivi: se ci sono più frame in coda
             * (es. risposta vecchia), la lettura successiva non deve trasmettere nulla. */
            L9963E_DRV_CS_LOW(handle);
            errorcode = L9963E_DRV_SPI_RECEIVE(handle, raw, 5, 10);
            L9963E_DRV_CS_HIGH(handle);
        }

        if (errorcode != L9963E_OK) {
            return errorcode;
        }

        _L9963E_DRV_switch_endianness(raw, (uint8_t *)&frame->val);

        if (frame->cmd.crc != L9963E_DRV_crc_calc(frame->val)) {
            return L9963E_CRC_ERROR;
        }
    }

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_DRV_burst_cmd(L9963E_DRV_HandleTypeDef *handle,
                                          uint8_t device,
                                          L9963E_BurstCmdTypeDef command,
                                          L9963E_BurstUnionTypeDef *data,
                                          uint8_t expected_frames_n,
                                          uint8_t timeout) {
    union L9963E_DRV_FrameUnion frame;
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    uint32_t current_tick;
    uint8_t raw[5];

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (data == NULL) {
        return L9963E_ERROR;
    }
#endif

    _L9963E_DRV_build_frame(raw, 1, 0, device, command, 0);

    errorcode = _L9963E_DRV_spi_transmit(handle, raw, 5, 10);

    if (errorcode != L9963E_OK) {
        return errorcode;
    }

    current_tick = L9963E_DRV_GETTICK(handle);
    for (uint8_t i = 0; i < expected_frames_n; ++i) {
        errorcode = _L9963E_DRV_wait_and_receive(&frame, handle, device, current_tick, timeout);

        if (errorcode != L9963E_OK) {
            return errorcode;
        }

        if (frame.cmd.addr == command)
            frame.cmd.addr = 1;

        data->generics[(frame.cmd.addr & 0b11111) - 1] = frame.cmd.data;
    }

    return L9963E_OK;
}

L9963E_StatusTypeDef _L9963E_DRV_reg_cmd(L9963E_DRV_HandleTypeDef *handle,
                                         uint8_t is_write,
                                         uint8_t device,
                                         L9963E_RegistersAddrTypeDef address,
                                         L9963E_RegisterUnionTypeDef *data,
                                         uint8_t timeout) {
    union L9963E_DRV_FrameUnion frame;
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    uint8_t raw[5];

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (data == NULL) {
        return L9963E_ERROR;
    }
#endif

    _L9963E_DRV_build_frame(raw, 1, is_write ? 1 : 0, device, address, is_write ? data->generic : 0);

    errorcode = _L9963E_DRV_spi_transmit(handle, raw, 5, 10);

    if (errorcode != L9963E_OK) {
        return errorcode;
    }

    /* Per una write broadcast (device=0) lo slave risponde con il proprio chip_ID,
     * non con 0. La wait_and_receive cercherebbe devid==0 e andrebbe sempre in
     * timeout. Saltiamo il readback: i dati sono stati inviati in broadcast. */
    if (is_write && device == L9963E_DEVICE_BROADCAST) {
        return L9963E_OK;
    }

    errorcode = _L9963E_DRV_wait_and_receive(&frame, handle, device, L9963E_DRV_GETTICK(handle), timeout);

    if (errorcode != L9963E_OK) {
        return errorcode;
    }

    if (is_write && frame.cmd.data != data->generic) {
        return L9963E_READBACK_ERROR;
    }

    data->generic = frame.cmd.data;

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_DRV_reg_read(L9963E_DRV_HandleTypeDef *handle,
                                         uint8_t device,
                                         L9963E_RegistersAddrTypeDef address,
                                         L9963E_RegisterUnionTypeDef *data,
                                         uint8_t timeout) {
#if L9963E_DEBUG
    if (device == 0) {
        return L9963E_ERROR;
    }
#endif

    return _L9963E_DRV_reg_cmd(handle, 0, device, address, data, timeout);
}

L9963E_StatusTypeDef L9963E_DRV_reg_write(L9963E_DRV_HandleTypeDef *handle,
                                          uint8_t device,
                                          L9963E_RegistersAddrTypeDef address,
                                          L9963E_RegisterUnionTypeDef *data,
                                          uint8_t timeout) {
    return _L9963E_DRV_reg_cmd(handle, 1, device, address, data, timeout);
}

L9963E_StatusTypeDef L9963E_DRV_trans_sleep(L9963E_DRV_HandleTypeDef *handle) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif
    /* DIS=HIGH → L9963T enters low-power/standby mode (L9963T DS Table 1) */
    return L9963E_DRV_DIS_HIGH(handle);
}

L9963E_StatusTypeDef L9963E_DRV_trans_wakeup(L9963E_DRV_HandleTypeDef *handle) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif
    /* DIS=LOW → L9963T enters Normal mode / active (L9963T DS Table 1) */
    return L9963E_DRV_DIS_LOW(handle);
}

L9963E_IF_PinState L9963E_DRV_trans_is_sleeping(L9963E_DRV_HandleTypeDef *handle) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    return L9963E_DRV_DIS_READ(handle);
}