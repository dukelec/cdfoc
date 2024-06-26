CDFOC Introduction
=======================================

<img src="doc/cdfoc_v4.avif" alt="Your browser may not support avif images!">

Sockets: RS-485: molex 5264 (4 pin x 2), motor: molex 5264 (3 pin), sensor: sh1.0 (10 pin)

Download this project:
```
git clone --recurse-submodules https://github.com/dukelec/cdfoc
```

<img src="doc/cdfoc_motor.avif">


## Block Diagram

<img src="doc/block_diagram.svg">


## GUI Tool

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

After power on, first write 5 to `state`, then write the target position to `tc_pos`, then the motor will rotate.

<img src="doc/cdbus_gui.avif">


After modifying the configuration, write 1 to `save_conf` to save the configuration to flash.  
If you need to restore the default configuration, change `magic_code` to another value and save it to flash. Then reapply power.


Plots:

<img src="doc/plot.avif">


## Protocol

The CDFOC communication protocol is the same as CDSTEP except for the differences in the register lists,
please refer to: https://github.com/dukelec/cdstep


## Operating

When switching between different modes, you need to write 0 to `state` first to return to idle mode.

### Calibration mode (state = 1)

Do not connect any loads to the motor during calibration.

#### Determining `motor_poles`

First, set the appropriate value to `cali_current`. It is recommended to set the value a little bit smaller to prevent the motor from overheating.

Then write 1 to `state` to enter the drag mode, where a constant current is continuously passed through some of the motor's coils
(please complete the subsequent operations as soon as possible, and then write 0 to `state` to prevent the motor from overheating).

Remember the initial position of the motor, then rotate the motor by hand, see how many times the motor jumps back to the initial position,
write the number of jumps to `motor_poles`, then write 1 to `save_conf` to save it.


#### Calibration Encoder

Write 1 to `dbg_en` to ensure that you can see the debug print in the GUI interface.

Start by setting the appropriate value for `cali_current`, it is recommended to set the value a little higher to allow the motor to lock more accurately.
Also be careful to prevent the motor from overheating.

Then write 1 to `state` to enter the drag mode, where a constant current is continuously passed through some of the motor's coils
(please complete the subsequent operations as soon as possible to prevent the motor from overheating).

Write 1 to `cali_run` to start the calibration, the motor coil will be energized sequentially, the motor will turn clockwise first, then counterclockwise,
and finally print the calculated encoder offset value, write it to `bias_encoder`, and finally save it. 

Write 1 to `cali_run` as soon as possible after writing 1 to `state`, so that the coils can be energized in turn, sharing the strain on individual coils. 

If the motor does not rotate clockwise first during calibration (the encoder value not increases),
you will need to swap any two wires of the motor to change the direction of motor rotation. 


### Torque Mode (state = 2)

Write 2 to `state` to enter torque mode, or current mode. 

The motor can then be rotated by writing the appropriate current value to `cal_current`. 

The unit of current is the LSB value of the 12bits ADC. 


### Speed Mode (state = 3)

Write 3 to `state` to enter speed mode.

Then write the appropriate speed value to `cal_speed` to rotate the motor.

One revolution of the motor is divided into 0x10000 units, e.g. from 0 to 0x10000 is one full revolution.  
The unit of speed is: how many position-units / second.


### Position Mode (state = 5)

Write 5 to `state` to enter position mode.

Then just write the appropriate values to `tc_pos`, `tc_speed`, `tc_accel`.

Alternatively these parameters can be updated using the quick-exchange command by default.

Mode 4 of `state` is a position mode without acceleration or deceleration and is not normally used.


## Command Demonstration

The address of the `state` itself is `0x0219` and its length is 1 byte.
To lock the motor and enter position mode after power up, send the following data to port 5:
```
20  19 02  05
```
`20` is the subcommand `write`, `19 02` is the little-endian for address 0x0219 (little-endian is used unless otherwise noted),
and `05` is the value to be written.


The complete command containing the CRC is (host address defaults to `0`, motor address defaults to `0xfe`, 3rd byte is data length, last two bytes are CRC):

```
00 fe 05  05 20  19 02  05  88 f5
```

The complete response package is:
```
fe 00 01  40  crc_l crc_h
```

Where `0x40` means no error, for more information please refer to the description of CDSTEP and CDNET.


### Quick Exchange Commands

`qxchg_set` uses only one element by default, pointing to the area where the table entries `tc_pos`, `tc_speed` and `tc_accel` are located.  
If you want to change the target position parameter `tc_pos` to 0, you can write data `20  00 00 00 00` to port 6 (`20` is the subcommand number).  

If you need to change both the target position and the target speed, e.g. position to 0x00010000 and speed to 0x00005000, writes: `20  00 00 01 00  00 50 00 00`.

Goes to 0 degrees, full command with CRC:
```
00 fe 06  06 20  00 00 00 00  63 99
```

Goes to 180 degrees, full command with CRC:  
(0x10000 units for one revolution, 0x8000 for 180 degrees)

```
00 fe 06  06 20  00 80 00 00  62 71
```

Rotates to the 5th revolution position and sends the speed value (0x00140000 or 14 revolutions per second) with the CRC:
```
00 fe 0a  06 20  00 00 05 00  00 00 14 00  fe 6e
```

Demonstration of return data for the above three commands:
```
fe 00 0b  40  xx xx xx xx yy yy yy yy  crc_l crc_h
```

where `0x40` means no error, `xx` and `yy` are defined by `qxchg_ret` to return 8 bytes of data such as `cal_pos`.



