#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2024, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <d@d-l.io>

"""CDFOC anticog tool

Args:
  --dev DEV           # specify serial port, default: /dev/ttyACM0
  --baud BAUD         # set baudrate, default: 115200
  --target-addr ADDR  # default: 80:00:fe
  --help    | -h      # this help message
  --verbose | -v      # debug level: verbose
  --debug   | -d      # debug level: debug
  --info    | -i      # debug level: info

Command example:

$ ./anticog.py

Notes:
  Please disable anticog_en first

"""

R_tp_pos = 0x01f8
R_state = 0x0240
R_tp_state = 0x0288
R_sen_i_sq_avg = 0x02b0

import sys, os
import struct
import _thread
import re
import json
from time import sleep
from argparse import ArgumentParser
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), './pycdnet'))

from cdnet.utils.log import *
from cdnet.utils.cd_args import CdArgs
from cdnet.dev.cdbus_serial import CDBusSerial
from cdnet.dispatch import *

args = CdArgs()
local_mac = int(args.get("--local-mac", dft="0x00"), 0)
dev_str = args.get("--dev", dft="ttyACM0")
baud = int(args.get("--baud", dft="10000000"), 0)
target_addr = args.get("--target-addr", dft="00:00:fe")


if args.get("--help", "-h") != None:
    print(__doc__)
    exit()


if args.get("--verbose", "-v") != None:
    logger_init(logging.VERBOSE)
elif args.get("--debug", "-d") != None:
    logger_init(logging.DEBUG)
elif args.get("--info", "-i") != None:
    logger_init(logging.INFO)


dev = CDBusSerial(dev_str, baud=baud)
CDNetIntf(dev, mac=local_mac)
sock = CDNetSocket(('', 0x40))
sock_dbg = CDNetSocket(('', 9))


def dbg_echo():
    while True:
        rx = sock_dbg.recvfrom()
        #print('\x1b[0;37m  ' + re.sub(br'[^\x20-\x7e]',br'.', rx[0][5:-1]).decode() + '\x1b[0m')
        print('\x1b[0;37m  ' + re.sub(br'[^\x20-\x7e]',br'.', rx[0]).decode() + '\x1b[0m')

_thread.start_new_thread(dbg_echo, ())


def csa_write(offset, dat):
    sock.sendto(b'\x20' + struct.pack("<H", offset) + dat, (target_addr, 5))
    ret, _ = sock.recvfrom(timeout=1)
    if ret == None or ret[0] != 0:
        print(f'csa_write error at: 0x{offset:x}: {dat.hex()}')
    return ret

def csa_read(offset, len_):
    sock.sendto(b'\x00' + struct.pack("<HB", offset, len_), (target_addr, 5))
    ret, _ = sock.recvfrom(timeout=1)
    if ret == None or ret[0] != 0:
        print(f'csa_write read at: 0x{offset:x}, len: {len_}')
    return ret


if csa_read(R_state, 1)[1] != 0:
    print('disable motor first...')
    csa_write(R_state, b'\x00')
    sleep(1)

print('start motor in position mode ...')
csa_write(R_state, b'\x05')
sleep(1)

print(f'goto 0 pos...')
csa_write(R_tp_pos, struct.pack("<i", 0))
while True:
    sleep(0.5)
    if csa_read(R_tp_state, 1)[1] == 0:
        print('goto 0 pos ready.')
        sleep(2)
        break


origin_val = []
for i in range(4096):
    pos = i << 4
    print(f'i: {i}, pos: {pos}')
    csa_write(R_tp_pos, struct.pack("<i", pos))
    sleep(0.5) # 0.2
    d = struct.unpack("<ff", csa_read(R_sen_i_sq_avg, 8)[1:])
    origin_val.append(d)


print('disable motor ...')
csa_write(R_state, b'\x00')

max_val = [0, 0]
for i in range(4096):
    max_val[0] = max(abs(origin_val[i][0]), max_val[0])
    max_val[1] = max(abs(origin_val[i][1]), max_val[1])
print('max_val origin:', max_val)
max_val[0] = round(max_val[0] * 128 / 127)
max_val[1] = round(max_val[1] * 128 / 127)

flash_iq = b''
flash_vq = b''
for i in range(4096):
    v0 = round(origin_val[i][0] * 128 / max_val[0])
    v1 = round(origin_val[i][1] * 128 / max_val[1])
    flash_iq += struct.pack("<b", v0)
    flash_vq += struct.pack("<b", v0)

flash_val = flash_iq + flash_vq

print(f'write anticog data to file: anticog_dat.bin, data len: {len(flash_val)} ...')
with open('anticog_dat.bin', 'wb') as f:
    f.write(flash_val)

print(f'write anticog data to file: anticog_dat.txt')
with open('anticog_dat.txt', 'w') as f:
    f.write(json.dumps(origin_val))

print(f'anticog_max_iq: {max_val[0]}, max_vq: {max_val[1]}')


print('done, please save to flash by yourself.')

# cdbus_gui/tools:
# ./cdg_iap.py --baud 10000000 --cfg ../configs/cdfoc-xxx.json --in-file anticog_dat.bin --addr=0x0801d800

