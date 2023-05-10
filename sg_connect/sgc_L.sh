#!/usr/bin/env sh
#0C:8B:95:70:2E:BE Right
#90:38:0C:A4:C1:8E

CTRL_DEVICE="2C:DB:07:6D:54:D8"
SG_DEVICE_L="90:38:0C:A4:C1:8E"
SG_RFCOMM_L="/dev/rfcomm1"

bluetoothctl pairable on
bluetoothctl discoverable on
bluetoothctl pair ${SG_DEVICE_L}
bluetoothctl trust ${SG_DEVICE_L}
bluetoothctl connect ${SG_DEVICE_L}
rfcomm connect ${SG_RFCOMM_L} ${SG_DEVICE_L} 1 &
