#!/bin/sh

st-flash --reset --format ihex write build/*.hex


# st-flash --area=option read
# 0xffeff8aa -> 0xfbeff8aa (force boot from main flash)
# st-flash --area=option write 0xfbeff8aa

