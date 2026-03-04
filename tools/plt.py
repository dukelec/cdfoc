#!/usr/bin/env python3

import sys
import time
import math
import struct
import _thread
#import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) <= 1:
    print(f'usage: {sys.argv[0]} dat_file.txt')
    exit(-1)

print('open data file: %s' % sys.argv[1])


with open(sys.argv[1], 'r') as f:
    dat = eval(f.read())

#dat = [[2, 3, 4, 5, 6, 7], [4, 5, 6, 7, 8, 9]]
t = list(range(0, len(dat)))
dat = list(zip(*dat))

if len(dat) >= 1:
    plt.setp(plt.plot(t, dat[0], 'g.-'), alpha=0.2) # green
if len(dat) >= 2:
    plt.setp(plt.plot(t, dat[1], 'b.-'), alpha=0.2) # blue
if len(dat) >= 3:
    plt.setp(plt.plot(t, dat[2], 'y.-'), alpha=0.2) # yellow
if len(dat) >= 4:
    plt.setp(plt.plot(t, dat[3], 'k.-'), alpha=0.2) # black
if len(dat) >= 5:
    plt.setp(plt.plot(t, dat[4], 'r.-'), alpha=0.2) # red
if len(dat) >= 6:
    plt.setp(plt.plot(t, dat[5], 'c.-'), alpha=0.2) # cyan
if len(dat) >= 7:
    plt.setp(plt.plot(t, dat[6], 'm.-'), alpha=0.2) # purple
# not use: w for white

plt.tight_layout()
plt.grid()
plt.show()
