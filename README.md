# cari-host
CARI Raspberry Pi host for the M17 Project's [Remote Radio Unit](https://github.com/M17-Project/rru-rf-hw).


### Dependencies:
- libzmq3
- libgpiod

### CLI parameters:

```
Usage: cari-host [OPTIONS]

CARI Host

Required options:
  -p, --pa=PIN          PA_EN GPIO pin number (integer)
  -n, --nrst=PIN        /RST GPIO pin number (integer)
  -b, --boot=PIN        BOOT0 GPIO pin number (integer)
  -d, --dl=PORT         Baseband downlink port number (integer)
  -c, --ctrl=PORT       Control port number (integer)
  -D, --device=PATH     UART device (string)
  -S, --speed=BAUD      UART baudrate (integer)

Optional options:
  -m, --me=PORT         Supervision (telemetry) channel downlink port number (integer)
  -r, --reset           Reset the device and exit
  -h, --help            Display this help message and exit

Example:
  cari-host -p 17 -n 27 -b 22 -d 17001 -c 17002 -m 17003 -D /dev/ttyAMA0 -S 460800
  cari-host --pa 17 --nrst 27 --boot 22 --dl 17001 --ctrl 17002 --me 17003 --device /dev/ttyAMA0 --speed 460800
```
