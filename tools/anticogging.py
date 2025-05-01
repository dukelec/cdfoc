#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2024, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <d@d-l.io>

"""CDFOC anticogging tool

Args:
  --dev DEV           # specify serial port, default: /dev/ttyACM0
  --baud BAUD         # set baudrate, default: 115200
  --target-addr ADDR  # default: 80:00:fe
  --help    | -h      # this help message
  --verbose | -v      # debug level: verbose
  --debug   | -d      # debug level: debug
  --info    | -i      # debug level: info

Command prompt example:

$ ./anticogging.py

"""

R_bias_encoder = 0x146
R_tc_pos = 0x01f8
R_anticogging_en = 0x021a
R_anticogging_max_val = 0x021c
R_state = 0x0240
R_nob_encoder = 0x264
R_tc_state = 0x0284
R_sen_i_sq_avg = 0x029c
R_cal_v_sq_avg = 0x02a0

ANTICOGGING_TBL = 0x0801d800 # 8k, 2bytes x 4096


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
baud = int(args.get("--baud", dft="115200"), 0)
target_addr = args.get("--target-addr", dft="80:00:fe")

out_file = args.get("--out-file")
reboot_flag = args.get("--reboot") != None

sub_size = 128

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
sock = CDNetSocket(('', 0xcdcd))
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
    if ret == None or ret[0] != 0x80:
        print(f'csa_write error at: 0x{offset:x}: {dat.hex()}')
    return ret

def csa_read(offset, len_):
    sock.sendto(b'\x00' + struct.pack("<HB", offset, len_), (target_addr, 5))
    ret, _ = sock.recvfrom(timeout=1)
    if ret == None or ret[0] != 0x80:
        print(f'csa_write read at: 0x{offset:x}, len: {len_}')
    return ret


def _read_flash(addr, _len):
    sock.sendto(b'\x00' + struct.pack("<IB", addr, _len), (target_addr, 8))
    ret, _ = sock.recvfrom()
    print(('  %08x: ' % addr) + ret.hex())
    if ret[0] != 0x80 or len(ret[1:]) != _len:
        print('read flash error')
        exit(-1)
    return ret[1:]

def _write_flash(addr, dat):    
    print(('  %08x: ' % addr) + dat.hex())
    sock.sendto(b'\x20' + struct.pack("<I", addr) + dat, (target_addr, 8))
    ret, _ = sock.recvfrom()
    print('  write ret: ' + ret.hex())
    if ret != b'\x80':
        print('write flash error')
        exit(-1)

def _erase_flash(addr, _len):
    sock.sendto(b'\x2f' + struct.pack("<II", addr, _len), (target_addr, 8))
    ret, _ = sock.recvfrom()
    print('  erase ret: ' + ret.hex())
    if ret != b'\x80':
        print('erase flash error')
        exit(-1)


def read_flash(addr, _len):
    cur = addr
    ret = b''
    while True:
        size = min(sub_size, _len-(cur-addr))
        if size == 0:
            break
        ret += _read_flash(cur, size)
        cur += size
    return ret

def write_flash(addr, dat):
    cur = addr
    ret = b''
    _erase_flash(addr, len(dat))
    while True:
        size = min(sub_size, len(dat)-(cur-addr))
        if size == 0:
            break
        wdat = dat[cur-addr:cur-addr+size]
        _write_flash(cur, wdat)
        rdat = _read_flash(cur, len(wdat))
        if rdat != wdat:
            print(f'rdat != wdat, @{cur:08x}')
            exit(-1)
        cur += size


if csa_read(R_state, 1)[1] != 0:
    print('disable motor first...')
    csa_write(R_state, b'\x00')
    sleep(1)

print('start motor in position mode ...')
csa_write(R_state, b'\x05')
print('disable anticogging_en first ...')
csa_write(R_anticogging_en, b'\x00')
sleep(1)

bias_encoder = struct.unpack("<H", csa_read(R_bias_encoder, 2)[1:])[0]

print(f'goto 0 pos... (bias_encoder: {bias_encoder:04x})')
csa_write(R_tc_pos, struct.pack("<i", -bias_encoder))
while True:
    sleep(0.5)
    if csa_read(R_tc_state, 1)[1] == 0:
        print('goto 0 pos ready.')
        sleep(2)
        break


origin_val = []
for i in range(4096):
    pos = i << 4
    print(f'i: {i}, pos: {pos}')
    csa_write(R_tc_pos, struct.pack("<i", pos - bias_encoder))
    while True:
        sleep(0.5) # 0.2
        d = struct.unpack("<ff", csa_read(R_sen_i_sq_avg, 8)[1:])
        nob_encoder = struct.unpack("<H", csa_read(R_nob_encoder, 2)[1:])[0]
        print(f'd: {d}, nob_encoder: {nob_encoder:04x}')
        origin_val.append(d)
        break # TODO: read the value twice and compare them


print('disable motor ...')
csa_write(R_state, b'\x00')

max_val = [0, 0]
for i in range(4096):
    max_val[0] = max(origin_val[i][0], max_val[0])
    max_val[1] = max(origin_val[i][1], max_val[1])
print('max_val:', max_val)

flash_val = b''
for i in range(4096):
    v0 = round(origin_val[i][0] * 100 / max_val[0])
    v1 = round(origin_val[i][1] * 100 / max_val[1])
    flash_val += struct.pack("<bb", v0, v1)

print(f'write anticogging data to file: anticogging_data.bin, data len: {len(flash_val)} ...')
with open('anticogging_data.bin', 'wb') as f:
    f.write(flash_val)

print(f'write anticogging data to file: anticogging_data.txt')
with open('anticogging_data.txt', 'w') as f:
    f.write(json.dumps(origin_val))


print(f'write anticogging data to flash ...')
write_flash(ANTICOGGING_TBL, flash_val)

print('write anticogging_max_val')
csa_write(R_anticogging_max_val, struct.pack("<ff", max_val[0], max_val[1]))

print('enable anticogging_en ...')
csa_write(R_anticogging_en, b'\x01')

print('done, please save to flash by yourself.')

