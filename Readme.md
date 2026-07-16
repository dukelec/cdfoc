CDFOC Introduction
=======================================

<img src="doc/cdfoc_v4.avif" alt="Your browser may not support avif images!">

Sockets: RS-485: molex 5264 (4 pin x 2), motor: molex 5264 (3 pin), sensor: sh1.0 (10 pin)

Download this project:
```
git clone --recursive https://github.com/dukelec/cdfoc
```

<img src="doc/cdfoc_motor.avif">


## Protocol

CDFOC is an open-source Field-Oriented Control (FOC) motor controller that communicates over an RS485 interface.
 - Default baud rate: 115200 bps
 - Maximum speed: 50 Mbps
 - Default address: 0xfe

The underlying protocol is CDBUS, with the following frame format:  
`src, dst, len, [payload], crc_l, crc_h`

Each frame includes a 3-byte header, a variable-length payload, and a 2-byte CRC (identical to Modbus CRC).  
For more information on the CDBUS protocol, please refer to:
 - https://cdbus.org

The payload is encoded using the CDNET protocol. For detailed information, please refer to:
 - https://github.com/dukelec/cdnet
 - https://github.com/dukelec/cdnet/wiki/CDNET-Intro-and-Demo


## Block Diagram

<img src="doc/block_diagram.svg">


## GUI Tool

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

Example: Upon power-on, first write 5 to `state`, then write the target position to `tp_pos` to rotate the motor.

<img src="doc/cdbus_gui.avif">


After modifying the configuration, write 1 to `save_conf` to save the configuration to flash.  
If you need to restore the default configuration, change `magic_code` to another value and save it to flash. Then reapply power.


Plots:

<img src="doc/plot.avif">


## Operating

### Encoder Calibration

Do not connect any loads to the motor during calibration.

Write 1 to `dbg_en` to enable debug prints in the GUI.

Start by setting the appropriate value for `cali_voltage`. A higher value locks the rotor more firmly, but also causes the motor to heat up more quickly.
Also be careful to prevent the motor from overheating.

Write 1 to `enc_cali` to start the calibration. The firmware automatically enters voltage mode and applies `cali_voltage`, causing the motor to turn clockwise first, then counterclockwise.
Afterward, the calculated encoder offset value will be printed and written to `bias_encoder`. Be sure to save it to flash.


### Voltage Mode (state = 1)

Write 1 to `state` to enter open-loop voltage mode.

Write the output voltage directly to `tgt_vq` and `tgt_vd`. The voltage value is represented as a fraction of the bus voltage:
`output voltage = target value / 32768 * bus voltage`.

The phase-current samples are still available for monitoring, but they do not participate in the control calculation or affect the motor output in this mode.


### Torque Mode (state = 2)

Write 2 to `state` to enter torque mode, or current mode.

The motor can then be rotated by writing the appropriate q-axis current value to `tgt_iq`.

The current-control value uses a signed 15-bit full-scale representation stored in an `int16_t` field. One raw 12-bit ADC count corresponds to 8 current-control counts. Conversion to physical current depends on the board's current-sense circuit.


### Speed Mode (state = 3)

Write 3 to `state` to enter speed mode.

Then write the appropriate speed value to `tgt_speed` to rotate the motor.

One revolution of the motor is divided into 0x10000 units, e.g. from 0 to 0x10000 is one full revolution.  
The unit of speed is: how many position-units / second.


### Position Mode (state = 5)

Write 5 to `state` to enter position mode.

Then just write the appropriate values to `tp_pos`, `tp_speed`, `tp_accel`.

Alternatively these parameters can be updated using the quick-exchange command by default.

Mode 4 of `state` is a position mode without acceleration or deceleration and is not normally used.


## Command Demonstration

The address of the `state` itself is `0x0400` and its length is 1 byte.
To lock the motor and enter position mode after power up, send the following data to port 5:
```
20  00 04  05
```
`20` is the subcommand `write`, `00 04` is the little-endian for address 0x0400 (little-endian is used unless otherwise noted),
and `05` is the value to be written.


The complete command containing the CRC is (host address defaults to `0`, motor address defaults to `0xfe`, 3rd byte is data length, last two bytes are CRC):

```
00 fe 06  40 05  20  00 04  05  e9 fb
```

The complete response package is:
```
fe 00 03  05 40  00  crc_l crc_h
```

The last `0x00` means no error.


### Quick Exchange Commands

`qxchg_set` uses only one element by default, pointing to the area where the table entries `tp_pos`, `tp_speed` and `tp_accel` are located.  
If you want to change the target position parameter `tp_pos` to 0, you can write data `00 00 00 00` to port 6.  

If you need to change both the target position and the target speed, e.g. position to 0x00010000 and speed to 0x00005000, writes: `00 00 01 00  00 50 00 00`.

Goes to 0 degrees, full command with CRC:
```
00 fe 06  40 06  00 00 00 00  64 f8
```

Goes to 180 degrees, full command with CRC:  
(0x10000 units for one revolution, 0x8000 for 180 degrees)

```
00 fe 06  40 06  00 80 00 00  65 10
```

Rotates to the 5th revolution position and sends the speed value (0x00140000 or 14 revolutions per second) with the CRC:
```
00 fe 0a  40 06  00 00 05 00  00 00 14 00  ae c4
```

Demonstration of return data for the above three commands:
```
fe 00 0a  06 40  xx xx xx xx yy yy yy yy  crc_l crc_h
```

The `xx` and `yy` are defined by `qxchg_ret` to return 8 bytes of data such as `tgt_pos`.
