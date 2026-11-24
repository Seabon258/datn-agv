#!/bin/bash
# Theo doi lien tuc: board vao DFU (0483:df11) la tu nap file .elf moi nhat. Ctrl+C de thoat.
ELF=${1:-/home/bon/datn/agv_f411/Debug/agv_f411.elf}
CLI=/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI
echo "$(date +%T) CHO DFU ..."
while true; do
  if lsusb | grep -q '0483:df11'; then
    echo "$(date +%T) THAY DFU, nap $(date -r "$ELF" +'%d/%m %H:%M') ..."
    sleep 1
    out=$(timeout 120 "$CLI" -c port=usb1 -d "$ELF" -v -g 0x08000000 2>&1 | sed 's/\x1b\[[0-9;]*m//g')
    if echo "$out" | grep -q 'Download verified successfully'; then
      echo "$(date +%T) NAP OK, code dang chay"
    else
      echo "$(date +%T) NAP LOI: $(echo "$out" | grep -i -E 'error|fail' | head -2 | tr '\n' ' ')"
    fi
    while lsusb | grep -q '0483:df11'; do sleep 1; done   # cho thoat DFU roi moi canh tiep
  fi
  sleep 1
done
