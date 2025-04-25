# cari-host
CARI Raspberry Pi host for the M17 Project's [Remote Radio Unit](https://github.com/M17-Project/rru-rf-hw).

## CLI parameters
| Parameter        | Description                                                       |
|------------------|-------------------------------------------------------------------|
| -pa *value*      | `PA_EN` GPIO pin number (integer)                                 |
| -nrst *value*    | `/RST` GPIO pin number (integer)                                  |
| -boot *value*    | `BOOT0` GPIO pin number (integer)                                 |
| -dl *value*      | Baseband downlink port number (integer)                           |
| -me *value*      | Supervision (telemetry) channel downlink port number (integer)    |
| -ctrl *value*    | Control port number (integer)                                     |
| -r               | Reset the device and exit                                         |
| -u *value*       | UART device (string)                                              |
| -s *value*       | UART baudrate (integer)                                           |

### Dependencies:
- libzmq3
- libgpiod

### Example
```bash
cari-host -pa 27 -nrst 17 -boot 22 -dl 17170 -ul 17171 -me 17172 -ctrl 17173 -d /dev/ttyAMA0 -s 460800
```