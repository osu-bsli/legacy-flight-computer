# All the constants used for CAN (CAN.h equivalent)
# SPI commands
CAN_RESET = 0xC0
CAN_READ = 0x03
CAN_WRITE = 0x02
CAN_BIT_MODIFY = 0x05
# Registers
CANCTRL = 0x0F
BFPCTRL = 0x0C
CANINTF = 0x2C
CANINTE = 0x2B
# Mask and filter addresses
RXM0SIDH = 0x20
RXM0SIDL = 0x21
RXM0EID8 = 0x22
RXM0EID0 = 0x23
RXM1SIDH = 0x24
RXM1SIDL = 0x25
RXM1EID8 = 0x26
RXM1EID0 = 0x27
# filters (just low and high since we dont use extended)
RXF0SIDH = 0x00
RXF0SIDL = 0x01
RXF1SIDH = 0x04
RXF1SIDL = 0x05
RXF2SIDH = 0x08
RXF2SIDL = 0x09
RXF3SIDH = 0x10
RXF3SIDL = 0x11
RXF4SIDH = 0x14
RXF4SIDL = 0x15
RXF5SIDH = 0x18
RXF5SIDL = 0x19
# Config registers
CNF1 = 0x2A
CNF2 = 0x29
CNF3 = 0x28
# CAN Status register
CANSTAT = 0x0E
# 3 Buffers each has 8 bytes but all sequential so as long as
# we know the address of the first one
TXB0D0 = 0x36
TXB1D0 = 0x46
TXB2D0 = 0x56
# message priorites
LOW_PRIORITY = 0x00
MED_LOW_PRIORITY = 0x01
MED_HIGH_PRIORITY = 0x02
HIGH_PRIORITY = 0x03
# Receive control register addresses
RXB0CTRL = 0x60
RXB1CTRL = 0x70
# 2 buffers with multiple bytes but just reference the first one
RXB0D0 = 0x66
RXB1D0 = 0x76
