#!/bin/sh

st-flash --reset --format ihex write build/*.hex


# st-flash --area=option read
# 0xffeff8aa -> 0xebeff8aa (force boot from main flash, nrst as io)
# st-flash --area=option write 0xebeff8aa

# bit[26] => 0: nboot0 from option bit27 (bit27: 1)
# bit[29:28] => 2'b10: nrst as io

