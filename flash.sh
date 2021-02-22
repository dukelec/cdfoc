#!/bin/sh

st-flash --reset write build/mdrv_fw.bin 0x08006800

#/mnt/TF/company/dukelec/cdbus/fw/cdbus_bridge/sw/cdbus_tools/cdbus_iap.py --in-file build/mdrv_fw.bin --addr=0x08006800 --reboot

