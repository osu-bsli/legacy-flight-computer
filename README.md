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

## TODO
Please self-assign tasks (put your name next to them) and check them off when done. Feel free to add/remove/modify tasks. This list is incomplete because none of us can predict *everything* that needs to be done. Neither do we know all the details of how to do each task - you will need to do research. If you have questions or need help, *it's always better to ask then to be silent*. If you don't know what to do, just pick something. If you don't have a lot of time, at least try to look at the code and understand some of it, or look through the documentation for some of the libraries, or read up on UART, etc. There's always something to do.

- [ ] change packet types [before 2023-04-12]
  - [ ] determing packet types -- dersu
  - [ ] packet.h
  - [ ] packet.c
  - [ ] test_packet.h + test_packet.c
  - [ ] everywhere else
  - [ ] python wrapper
- [ ] figure out communication with sensers [before 2023-04-12]
  - [ ] i2c
  - [ ] can
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
