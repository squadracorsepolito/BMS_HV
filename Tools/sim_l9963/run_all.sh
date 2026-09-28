#!/bin/bash
# Simulatore host di L9963T (TH/TL) + L9963E, modellato sui datasheet.
# Compila il firmware REALE (driver, L9963E.c, L9963_utils.c, data_reading, ntc)
# sostituendo solo stm32_if.c e lancia 5 scenari. Uso: ./run_all.sh [dual]
set -e
D=$(cd "$(dirname "$0")" && pwd); R=$(cd "$D/../.." && pwd)
CI=$(mktemp -d); CS=$(mktemp -d)
for h in L9963_utils.h data_reading_timebase.h ntc.h stm32_if.h bms_hv_fsm.h; do cp $R/Core/Inc/$h $CI/; done
cp $R/Core/Src/L9963_utils.c $R/Core/Src/data_reading_timebase.c $R/Core/Src/ntc.c $CS/
EXTRA=""; [ "$1" = "dual" ] && EXTRA="-DDUAL"
gcc -O1 -I$D/inc -I$CI -I$R/Lib/L9963E/inc -I$R/Lib/stmlibs $EXTRA $D/model.c $D/run.c \
    $R/Lib/L9963E/src/L9963E.c $R/Lib/L9963E/src/L9963E_drv.c $CS/L9963_utils.c $CS/data_reading_timebase.c $CS/ntc.c -o /tmp/sim_bms -lm
# argomenti: seed  stato_iniziale_slave(0..3)  tempo_risveglio_us  durata_s  rumore_ms  verbose
for a in "11 0 2000 15 0" "22 0 8000 15 0" "33 1 2000 15 0" "44 2 2000 15 0" "55 3 3000 30 5000"; do
    /tmp/sim_bms $a 0 | grep -E "^###|RISULTATO"
done
