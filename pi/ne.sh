#!/bin/sh
# Nguoi van hanh chon ben ne khi xe dang dung cho vat can:  ./ne.sh 9   (ne ve 9 gio = TRAI)   |   ./ne.sh 3   (3 gio = PHAI)
cd ~/agv || exit 1
case "$1" in 9) c=ne_9 ;; 3) c=ne_3 ;; *) echo "dung: ne.sh 9|3"; exit 1 ;; esac
python3 -c "from db import AgvDB; AgvDB('agv.db').push_command('$c'); print('da gui lenh $c')"
