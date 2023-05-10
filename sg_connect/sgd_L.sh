#!/usr/bin/env sh

CTRL_DEVICE="2C:DB:07:6D:54:D8"
SG_DEVICE_L="90:38:0C:A4:C1:8E"
SG_RFCOMM_L="/dev/rfcomm1"

bluetoothctl disconnect ${SG_DEVICE_L}
rfcomm release ${SG_RFCOMM_L}
