#!/bin/bash
# Chay ca 4 banh dao chieu lien tuc N giay roi tong ket nhieu: stress_test.sh 300
DUR=${1:-300}; L=$(ls -t /home/bon/datn/logs/uart_*.log | head -1); n=$(wc -l < $L); t0=$(date +%T)
end=$((SECONDS+DUR)); i=0; levels=(50 -60 80 -80 100 -100 30 -50)
while [ $SECONDS -lt $end ]; do printf 'm 0 %s\n' ${levels[$((i % 8))]} > /tmp/agv_cmd; i=$((i+1)); sleep 1.9; done
printf 's\n' > /tmp/agv_cmd; sleep 2; printf 'i\n' > /tmp/agv_cmd; sleep 1
F=$(mktemp); tail -n +$((n+1)) $L > $F
echo "Tu $t0 toi $(date +%T), $i lenh"
echo "khoi dong lai: $(grep -c '=== AGV' $F) | WATCHDOG: $(grep -c WATCHDOG $F) | lenh bi bo qua: $(grep -c 'bo qua' $F) | IMU loi: $(grep -c 'IMU loi' $F) | lenh co tra loi: $(grep -c 'chay M0' $F)/$i"
python3 -I - "$F" <<'PY'
import re,sys
L=open(sys.argv[1]).read().splitlines(); ok=bad=0; pins=[]; gaps=0; last=None
import datetime
for l in L:
    m=re.search(r'^(\d\d:\d\d:\d\d) .*ENC (-?\d+) (-?\d+) (-?\d+) (-?\d+) \| V .*PIN ([\d.]+) V',l)
    if m:
        ok+=1; pins.append(float(m.group(6)))
        t=datetime.datetime.strptime(m.group(1),'%H:%M:%S')
        if last and (t-last).total_seconds()>2: gaps+=1
        last=t
    elif ' ENC' in l: bad+=1
print("dong trang thai hong:",bad,"/",ok+bad,"| quang mat du lieu >2 s:",gaps,"| pin min %.2f max %.2f"%(min(pins),max(pins)) if pins else "khong co du lieu")
PY
rm -f $F
