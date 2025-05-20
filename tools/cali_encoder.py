#!/usr/bin/env python3

import sys
import time
import math
import struct
import umsgpack
import _thread
#import numpy as np
import matplotlib.pyplot as plt

# rotate motor by state = 1, cali_angle_step > 0 and bias_encoder = 0
# enable plot0 for a short period, then export cdbus_gui data to mpk file

if len(sys.argv) <= 1:
    print(f'usage: {sys.argv[0]} mpk_dat_file')
    exit(-1)

print('open data file: %s' % sys.argv[1])


with open(sys.argv[1], 'rb') as f:
    in_file = f.read()
    in_data = umsgpack.unpackb(in_file)

dat_x = in_data['plot'][0][0]
dat_encoder = in_data['plot'][0][6]

if dat_x[-1] - dat_x[0] + 1 != len(dat_x):
    print('dat_x not continued, exit')
    exit(-1)


def find_edge(i_start, is_first=True):
    if i_start + 1 >= len(dat_encoder):
        return None
    val_max = dat_encoder[i_start]
    val_cur = val_max
    i_cur = i_start
    while True:
        i_cur += 1
        if i_cur >= len(dat_encoder):
            return None
        val_cur = dat_encoder[i_cur]
        if val_cur > val_max:
            val_max = val_cur
        elif val_max - val_cur > 60000:
            if is_first:
                return i_cur
            else:
                if i_cur - i_start >= 20:
                    return i_cur
                else:
                    val_max = 0

i_s = find_edge(0)
i_e = find_edge(i_s, False)
i_len = i_e - i_s
print(f'start at: {i_s}, end at: {i_e}, len: {i_len}')

plt.figure()
plt.subplot(211)

t = list(range(0, len(dat_encoder)))
plt.setp(plt.plot(t, dat_encoder, 'g.-'), alpha=0.2) # green


def get_avg(idx, is_low=True):
    vlist = [
            dat_encoder[idx-1],
            dat_encoder[idx],
            dat_encoder[idx+1],
        ]
    if is_low:
        for i in range(len(vlist)):
            if vlist[i] > 0x10000*(3/4):
                vlist[i] -= 0x10000
    else:
        for i in range(len(vlist)):
            if vlist[i] < 0x10000*(1/4):
                vlist[i] += 0x10000
    avg = (vlist[0] + vlist[1] + vlist[2]) / 3
    return avg


dat_filtered = []
for i in range(i_len):
    is_low = i < (i_len / 2)
    dat_filtered.append(get_avg(i_s + i, is_low))


plt.setp(plt.plot(t[i_s:i_e], dat_filtered, 'b.-'), alpha=0.2)
plt.setp(plt.plot([i_s, i_e-1], [dat_encoder[i_s], dat_encoder[i_e-1]], 'r.-'), alpha=0.6) # red


plt.tight_layout()
plt.grid()

cali_t = []
cali_table = []
next_val = 0

def find_val(y_val):
    for i in range(len(dat_filtered)):
        if dat_filtered[i] >= y_val:
            return round(i / len(dat_filtered) * 65535)
    return 65535

for i in range(4096):
    cali_t.append(i)
    cali_table.append(find_val(i * 16))

print(f'cali len: {len(cali_table)}')

flash_val = b''
for i in range(4096):
    flash_val += struct.pack("<H", cali_table[i])

print(f'write cali_encoder data to file: cali_encoder_data.bin, data len: {len(flash_val)} ...')
with open('cali_encoder_data.bin', 'wb') as f:
    f.write(flash_val)


plt.subplot(212)
plt.setp(plt.plot(cali_t, cali_table, 'b.-'), alpha=0.2)

plt.setp(plt.plot([0, 4095], [0, 65535], 'r.-'), alpha=0.6) # red

plt.tight_layout()
plt.grid()

plt.show()

# cdbus_gui/tools:
# ./cdg_iap.py --baud 10000000 --cfg ../configs/cdfoc-xxx.json --in-file cali_encoder_data.bin --addr=0x0801b800

