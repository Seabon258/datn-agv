#!/bin/sh
# Chay bam line tren Pi, KHONG chay trung (Wi-Fi chap chon co the gui lenh 2 lan). Dung: touch ~/agv/STOP
cd ~/agv || exit 1
if ps -eo args | grep -q "^python3 -u line_follow.py"; then echo "DANG CHAY ROI"; exit 0; fi
rm -f STOP
nohup setsid python3 -u line_follow.py --max-s "${1:-60}" > follow.log 2>&1 < /dev/null &
echo DA_BAT_DAU
