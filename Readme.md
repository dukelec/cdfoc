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

Example: Upon power-on, first write 5 to `state`, then write the target position to `tc_pos` to rotate the motor.

<img src="doc/cdbus_gui.avif">


After modifying the configuration, write 1 to `save_conf` to save the configuration to flash.  
If you need to restore the default configuration, change `magic_code` to another value and save it to flash. Then reapply power.


Plots:

<img src="doc/plot.avif">


## Operating

Sensorless mode:
 - Write `sl_start` = 1 for forward rotation, -1 for reverse rotation.
 - Write `state` = 0 to stop the motor.

Sensored mode operates the same as the master branch.

