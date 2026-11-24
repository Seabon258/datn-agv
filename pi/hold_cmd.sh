#!/bin/bash
# Giu 1 lenh dong co lien tuc N giay (qua uart_logger FIFO): hold_cmd.sh "m 3 50" 120
end=$((SECONDS+$2)); while [ $SECONDS -lt $end ]; do printf '%s\n' "$1" > /tmp/agv_cmd; sleep 1.8; done
printf 's\n' > /tmp/agv_cmd; echo "da dung: $1"
