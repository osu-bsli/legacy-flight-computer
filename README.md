# Legacy Flight Computer
This a modified version of the original flight computer code from our 2022 competition.

- made in Python
- runs on a Raspberry Pi
- uses CircuitPython (via Adafruit Blinka) to communicate with sensors
- sends packets back to ground control using [packet-parser](https://github.com/osu-bsli/packet-parser)

## Raspberry Pi setup
1. Install [DietPi](https://dietpi.com/), or install the default OS and debloat it.
2. Install Python 3.?+
3. Install the [Adafruit Blinka](https://github.com/adafruit/Adafruit_Blinka) CircuitPython api/emulator library.
4. Clone this repo onto the Pi.

## Telemetry & packet format
AFAIK, here's how it works:
1. read bytes via `pyserial`
2. check for 4-byte starting delimiter (3735928559 = `0xDEADBEEF`)
3. determine packet type (next 2 bytes after delimiter)
4. if the packet is for arming/disarming:
   1. if arming by CAN is enabled, send corresponding CAN message
   2. otherwise, set corresponding GPIO to high
   3. `time.sleep(0.1)`
   4. set corresponding GPIO to low
5. fetch data from peripherals
6. do some math
7. send data to ground as one big packet

Inbound packet types:
| type | description                      |
|------|----------------------------------|
| 1    | arm telemetrum                   |
| 2    | arm stratologger                 |
| 3    | arm camera                       |
| 4    | disarm telemetrum                |
| 5    | dissarm stratologger             |
| 6    | disarm camera                    |
| 10   | set ground level for launch      |
| 11   | set ground level to 0 (a reset?) |
| 12   | start logging data to log file   |
| 13   | stop logging data to log file    |
| 99   | killswitch (sudo halt)           |

Inbound CAN message types:
| type                | data (bytes)                                                                            |
|---------------------|-----------------------------------------------------------------------------------------|
| `PWR_BRD_BAT_DATA`  | main battery voltage (4), main battery current (4)                                      |
| `PWR_BRD_RAIL_DATA` | 3v rail voltage (2), 5v rail voltage (2), main battery voltage (2), ADC temperature (2) |
| `ARM_DEVICE_ACK`    | none                                                                                    |
| `DISARM_DEVICE_ACK` | none                                                                                    |
| `ARMING_BAT_DATA`   | current (2), voltage (2)                                                                |

Outbound data (to ground) is all in 1 packet. The format is, in order:
1. header:
   1. `CRC_SYNC` (4)
   2. `CFC_ID` (2)
2. high g x/y/z:
   1. x (4)
   2. y (4)
   3. z (4)
3. magnetometer:
   1. x (4)
   2. y (4)
   3. z (4)
4. gyroscope:
   1. x (4)
   2. y (4)
   3. z (4)
5. acceleration:
   1. x (4)
   2. y (4)
   3. z (4)
6. gps:
   1. satellite count (1)
   2. latitude (4)
   3. longitude (4)
   4. ascent (4)
   5. ground speed (4)
7. telemetrum:
   1. status (1)
   2. current (4)
   3. voltage (4)
8. stratologger:
   1.  status (1)
   2.  current (4)
   3.  voltage (4)
9.  camera:
    1.  status (1)
    2.  current (4)
    3.  voltage (4)
10. 3v rail voltage (4)
11. 5v rail voltage (4)
12. main battery voltage (4)
13. main battery temp (4)
Total packet size is 114 bytes.

## TODO
Please self-assign tasks (put your name next to them) and check them off when done. Feel free to add/remove/modify tasks. This list is incomplete because none of us can predict *everything* that needs to be done. Neither do we know all the details of how to do each task - you will need to do research. If you have questions or need help, *it's always better to ask then to be silent*. If you don't know what to do, just pick something. If you don't have a lot of time, at least try to look at the code and understand some of it, or look through the documentation for some of the libraries, or read up on UART, etc. There's always something to do.

- [ ] change packet types [before 2023-04-12]
  - [ ] determing packet types -- dersu
    - [ ] inbound
    - [x] outbound
    - [ ] functionalities
  - [ ] packet.h -- dersu
  - [ ] packet.c
  - [ ] test_packet.h + test_packet.c
  - [ ] everywhere else
  - [ ] python wrapper
- [ ] figure out communication with sensers [before 2023-04-12]
  - [ ] i2c -- ayden
  - [ ] can -- peter
  - [ ] reading/writing
  - [ ] initialization
    - [ ] constants/registers
- [ ] test the existing code + board [on 2023-04-09]
  - [ ] make sure dependencies still work
- [ ] raspberry pi setup [after the above]
  - [ ] autostart
  - [ ] restart on crash?
  - [ ] where to save data
  - [ ] library/software configuration
  - [ ] turn off unused things, debloat, (DietPi?)
