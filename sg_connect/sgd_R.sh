#!/usr/bin/env sh

CTRL_DEVICE="2C:DB:07:6D:54:D8"
SG_DEVICE_R="0C:8B:95:70:2E:BE"
SG_RFCOMM_R="/dev/rfcomm0"


bluetoothctl disconnect ${SG_DEVICE_R}
rfcomm release ${SG_RFCOMM_R}
