#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2024, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <d@d-l.io>

"""CDFOC anticog tool

Command example:

$ ./anticog_dat2bin.py anticog_dat.txt

"""

import sys, os
import struct
import json
import math

with open(sys.argv[1], 'r') as file:
    origin_val = json.load(file)

if len(origin_val) != 4096:
    print(f'wrong len')
    exit(-1)

def get_avg(idx):
    vlist = [
            origin_val[(idx-2) & 4095],
            origin_val[(idx-1) & 4095],
            origin_val[idx & 4095],
            origin_val[(idx+1) & 4095],
            origin_val[(idx+2) & 4095],
        ]
    avg0 = (vlist[0][0] + vlist[1][0] + vlist[2][0] + vlist[3][0] + vlist[4][0]) / 5
    avg1 = (vlist[0][1] + vlist[1][1] + vlist[2][1] + vlist[3][1] + vlist[4][1]) / 5
    return [avg0, avg1]


dat_filtered = []
for i in range(len(origin_val)):
    dat_filtered.append(get_avg(i))


sum0 = 0
sum1 = 0
for i in range(len(dat_filtered)):
    sum0 += dat_filtered[i][0]
    sum1 += dat_filtered[i][1]
mid0 = sum0 / len(dat_filtered)
mid1 = sum1 / len(dat_filtered)
print(f'mid val: {mid0}, {mid1}')


dat_ac = []
for i in range(len(dat_filtered)):
    dat_ac.append([dat_filtered[i][0] - mid0, dat_filtered[i][1] - mid1])


sum0 = 0
sum1 = 0
for i in range(len(dat_ac)):
    sum0 += abs(dat_ac[i][0])
    sum1 += abs(dat_ac[i][1])
ratio = sum1 / sum0
ratio_12 = round(ratio * 4096)
print(f'ratio_vq: {ratio_12} ({ratio})')


print(f'write dat_ac to file: anticog_ac.txt')
with open('anticog_ac.txt', 'w') as f:
    f.write(json.dumps(dat_ac))


max_val = [0, 0]
for i in range(4096):
    max_val[0] = max(abs(dat_ac[i][0]), max_val[0])
    max_val[1] = max(abs(dat_ac[i][1]), max_val[1])
print('max_val origin:', max_val)
max_val[0] = math.ceil(max_val[0] * 128 / 127)
max_val[1] = math.ceil(max_val[1] * 128 / 127)
print(f'anticog_max_iq: {max_val[0]} (max_vq: {max_val[1]})')


flash_val = b''
for i in range(4096):
    v0 = round(dat_ac[i][0] * 128 / max_val[0])
    flash_val += struct.pack("<b", v0)

print(f'write anticog data to file: anticog_dat.bin, data len: {len(flash_val)} ...')
with open('anticog_dat.bin', 'wb') as f:
    f.write(flash_val)

print('done, please save to flash by yourself.')

# cdbus_gui/tools:
# ./cdg_iap.py --baud 10000000 --cfg ../configs/cdfoc-xxx.json --in-file anticog_dat.bin --addr=0x0801e800

