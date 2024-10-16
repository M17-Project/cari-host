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
