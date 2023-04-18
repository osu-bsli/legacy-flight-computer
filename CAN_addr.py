# VERY IMPORTANT TO CHANGE FOR EVERY DEVICE ON THE CAN BUS!
# MUST BE UNIQUE FOR EVERY DEVICE ON CAN BUS
# Can only use least significant 4 bits (max 0x07)
MY_CAN_ADDR = 0x03


# Also use this file to define addresses for other dev on the bus, ex:
STRATOLOGGER = 0x01
TELEMETRUM = 0x02
CUSTOM_FC = 0x03
BATTERY_BOARD = 0x04
CAMERA = 0x05

# Define message types (can use 5 bits)
WAKEUP_DEVICE = 0x00
ARM_DEVICE = 0x01
DISARM_DEVICE = 0x02
SLEEP_DEVICE = 0x0A
PWR_BRD_BAT_DATA = 0x06
PWR_BRD_RAIL_DATA = 0x07
ARMING_BAT_DATA = 0x08
# BAT data from all arming boards uses same message type, use from addr to figure device

# data should be CAN address of which device
ARM_DEVICE_ACK = 0x03
DISARM_DEVICE_ACK = 0x04
