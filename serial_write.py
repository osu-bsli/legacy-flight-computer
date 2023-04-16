#!/usr/bin/env python
import sys
sys.path.append('../../')
sys.path.append('/')
sys.path.append('./CAN')
sys.path.append('/home/pi/CAN')
sys.path.append('../../')
sys.path.append('/')
# Need in case we run not as pi user
sys.path.append('/home/pi/.local/lib/python3.9/site-packages/')
import packet_util
import os
import time
import serial
import ctypes
import board
import busio
import adafruit_adxl34x
import RPi.GPIO as GPIO
from BMX160 import BMX160
from gpiozero import CPUTemperature
from MS5607 import MS5607
from ublox import Ublox
from adafruit_bus_device.spi_device import SPIDevice
from CAN_addr import *
from CAN_consts import *
import struct
import digitalio
import CAN
import bitbangio
import math
packetlib = __import__("packet-parser").packetlib
import ctypes


# i2c = bitbangio.I2C(scl=board.D3, sda=board.D2, frequency=400000)
i2c = busio.I2C(scl=board.D3, sda=board.D2, frequency=400000)
high_g_accelerometer = adafruit_adxl34x.ADXL345(i2c)

# max I2C clock speeds
# gps: 400k
# BMX160: 1M
# MS5607: 400k
# ADXL345: 400k (high g)

baro = MS5607()
bmx = BMX160(1)
bmx.begin()
bmx.wake_up()

gps = Ublox()
gps.start()

# We want to arm directly from this instead of over can
can_arming = False

ser = serial.Serial(
    port='/dev/ttyS0',  # Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.001
)
CFC_length = ctypes.c_uint8(87)
CFC_ID = ctypes.c_uint16(1)
CFC_SYNC = ctypes.c_uint32(16843009)

t_arm_pin = 17
t_disarm_pin = 22
s_arm_pin = 18
s_disarm_pin = 23
c_arm_pin = 27
c_disarm_pin = 24

# Ground offset initalize
gps_ground_level = 0
baro_ground_level = 0
baro_alt = 0
gps_alt = 0
logging_data = False

# GPIO.setmode(GPIO.BOARD)   #already set as BCM mode by ublox library
# Set these as inputs so they are high impedance and MCU can drive them
if can_arming:
    GPIO.setup(t_arm_pin, GPIO.IN)
    GPIO.setup(t_disarm_pin, GPIO.IN)
    GPIO.setup(s_arm_pin, GPIO.IN)
    GPIO.setup(s_disarm_pin, GPIO.IN)
    GPIO.setup(c_arm_pin, GPIO.IN)
    GPIO.setup(c_disarm_pin, GPIO.IN)
else:
    GPIO.setup(t_arm_pin, GPIO.OUT)
    GPIO.setup(t_disarm_pin, GPIO.OUT)
    GPIO.setup(s_arm_pin, GPIO.OUT)
    GPIO.setup(s_disarm_pin, GPIO.OUT)
    GPIO.setup(c_arm_pin, GPIO.OUT)
    GPIO.setup(c_disarm_pin, GPIO.OUT)

""" Code for CAN things here """
# Global variables for main battery data to be set in functions:
# Note global functions not working rn
mainBatteryVoltage = 0
mainBatteryCurrent = 0
mainBatteryReadTime = 0
v3_rail_voltage = 3.3
v5_rail_voltage = 5.0
mainBatteryTemperature = 0
# Setup can controller:
reset_pin = digitalio.DigitalInOut(board.D13)
reset_pin.direction = digitalio.Direction.OUTPUT
stby_pin = digitalio.DigitalInOut(board.D25)
stby_pin.direction = digitalio.Direction.OUTPUT
SCK = board.SCK
MOSI = board.MOSI
MISO = board.MISO
cs_pin = board.CE1
# Setup CAN controller (configures it too)
can = CAN.CAN_Controller(stby_pin, reset_pin, SCK, MOSI, MISO, cs_pin)


# Simple class to store data for each arming board
class ArmingBoard:
    arm_status = False
    # current in mA
    current = 0
    voltage = 0
    bat_update_time = time.time()
    arm_update_time = time.time()


# Define struct for each of the arming boards
telemetrum_board = ArmingBoard()
stratologger_board = ArmingBoard()
camera_board = ArmingBoard()


def doCANmessages():
    msg1 = can.CAN_Receive(RXB0D0)
    msg2 = can.CAN_Receive(RXB1D0)
    handleCANmsg(msg1)
    handleCANmsg(msg2)


def handleCANmsg(msg):
    global mainBatteryVoltage
    global mainBatteryCurrent
    global mainBatteryReadTime
    global telemetrum_board
    global stratologger_board
    global camera_board
    global v3_rail_voltage
    global v5_rail_voltage
    global mainBatteryTemperature

    # If type is 0x00 do nothing because not a real message
    if not msg.type:
        return

    # If we are using an arming board, we can determine which one it is here
    # then all the code is the same in the main if statements
    # since this assignment is not deep copy, it will change values for orignal
    # object also
    arm_board = False
    if msg.from_addr == STRATOLOGGER:
        arm_board = stratologger_board
    elif msg.from_addr == TELEMETRUM:
        arm_board = telemetrum_board
    elif msg.from_addr == CAMERA:
        arm_board = camera_board

    if msg.type == PWR_BRD_BAT_DATA:
        # save data to thing
        mainBatteryVoltage = struct.unpack('<f', msg.data[0:4])[0]
        mainBatteryCurrent = struct.unpack('<f', msg.data[4:8])[0]
        mainBatteryReadTime = time.time()
    elif msg.type == PWR_BRD_RAIL_DATA:
        # Should come in as little endian 3v rail voltage then 5v rail
        v3voltage = int.from_bytes(
            msg.data[0:2], byteorder='little', signed=False)
        v5voltage = int.from_bytes(
            msg.data[2:4], byteorder='little', signed=False)
        vBvoltage = int.from_bytes(
            msg.data[4:6], byteorder='little', signed=False)
        temp_ADC = int.from_bytes(
            msg.data[6:8], byteorder='little', signed=False)

        # Power board uses stable 3V reference
        # Reference voltage provided by chip on power board, could read
        # calibration values from powergboard
        v3_ref = 3.007

        #v3_rail_voltage = v3voltage * (3.0 / 4095.0) / (10.0 / 11.43) / 16
        # This voltage divider value based on measured voltages
        v3_rail_voltage = v3voltage * (v3_ref / 4095.0) / (0.86826347305) / 16
        # 5v rail resistances wrong
        v5_rail_voltage = v5voltage * (v3_ref / 4095.0) * (3.19240506329) / 16
        # battery voltage (divide by 16 done on board to reduce int size)
        mainBatteryVoltage = ((vBvoltage / 4095.0) * v3_ref) / 0.6
        # batter temperature (celcius):
        temp_voltage = (temp_ADC / 4095.0) * v3_ref / 16.0
        rtherm = (temp_voltage * 10000.0) / (v3_rail_voltage - temp_voltage)
        tKelvin = (3435.0 * 293.0) / (3435.0 +
                                      (293.0 * math.log(rtherm / 12090.0)))
        mainBatteryTemperature = tKelvin - 273.0

    elif msg.type == ARM_DEVICE_ACK:
        arm_board.arm_status = True
        arm_board.arm_update_time = time.time()
    elif msg.type == DISARM_DEVICE_ACK:
        arm_board.arm_status = False
        arm_board.arm_update_time = time.time()
    elif msg.type == ARMING_BAT_DATA:
        # Devices should send data as little endian unsigned int
        # also send current then voltage
        # Divide by 16 because 16 accumulated values
        current = int.from_bytes(
            msg.data[0:2], byteorder='little', signed=False) / 16.0
        voltage = int.from_bytes(
            msg.data[2:4], byteorder='little', signed=False) / 16.0
        # Convert ADC readings to true values
        if msg.from_addr == TELEMETRUM:
            voltage = voltage * (v3_rail_voltage / 4095.0) / (100.0 / 127.4)
        elif(msg.from_addr == STRATOLOGGER):
            voltage = voltage * (v3_rail_voltage / 4095.0) / (100.0 / 262.0)
        elif(msg.from_addr == CAMERA):
            voltage = voltage * (v3_rail_voltage / 4095.0) / (100.0 / 262.0)
        else:
            voltage = 0

        arm_board.voltage = voltage
        arm_board.current = current * v3_rail_voltage * 1000 / (2.0 * 4095.0)
        arm_board.bat_update_time = time.time()

    """
    # Printing stuff
    if arm_board:
        if msg.from_addr == STRATOLOGGER:
            print("Stratologger data:")
        elif msg.from_addr == TELEMETRUM:
            print("Telemetrum data:")
        elif msg.from_addr == CAMERA:
            print("Camera data:")
        print(f'==Current (mA): {arm_board.current}')
        print(f'==Voltage (V): {arm_board.voltage}')
        print(f'==Armed? {arm_board.arm_status}')
    else:
        print('Bat data:')
        print(f'==3V3 rail (V): {v3_rail_voltage}')
        print(f'==5V rail (V): {v5_rail_voltage}')
        print(f'==Main bat Voltage {vBvoltage}')
    """


arm_telemetrum_msg = CAN.CAN_message(
    dest_addr=TELEMETRUM, type=ARM_DEVICE)
arm_stratalogger_msg = CAN.CAN_message(
    dest_addr=STRATOLOGGER, type=ARM_DEVICE)
arm_camera_msg = CAN.CAN_message(
    dest_addr=CAMERA, type=ARM_DEVICE)
disarm_telemetrum_msg = CAN.CAN_message(
    dest_addr=TELEMETRUM, type=DISARM_DEVICE)
disarm_stratalogger_msg = CAN.CAN_message(
    dest_addr=STRATOLOGGER, type=DISARM_DEVICE)
disarm_camera_msg = CAN.CAN_message(
    dest_addr=CAMERA, type=DISARM_DEVICE)
""" End code for CAN things """

# can.CAN_Transmit(TXB0D0, arm_telemetrum_msg, MED_HIGH_PRIORITY)
# can.waitToSendCAN()
# can.CAN_Transmit(TXB0D0, disarm_telemetrum_msg, MED_HIGH_PRIORITY)
while 1:
    # check if new bytes available before trying to read them
    
    inbound_bytes: bytearray = bytearray()
    if ser.in_waiting > 0: # TODO: ask toby about implementation in illiad_data_controller.py
        inbound_bytes += ser.read_all()
    inbound_bytes_to_enqueue = min(len(inbound_bytes), packetlib._BUFFER_SIZE - 1)
    for i in range(inbound_bytes_to_enqueue):
        packetlib.enqueue(ctypes.c_ubyte(inbound_bytes[i]))

    if ser.in_waiting > 0:
        rawSer = ser.read(4)
        serRead = int.from_bytes(rawSer, byteorder='little', signed=False)
    elif ser.in_waiting > 200:
        # If there is too much data coming in then flush to not get behind
        ser.flush()
    else:
        # if nothing to read, 0 bytes
        serRead = bytes(0)

    if(serRead == 3735928559):
        print("sync found")
        rawSer = ser.read(2)  # read 2 bytes from serial
        serRead = 0
        serRead = int.from_bytes(rawSer, byteorder='little', signed=False)
        if(serRead == 1):
            print("t_a")
            if can_arming:
                can.CAN_Transmit(TXB0D0, arm_telemetrum_msg, MED_HIGH_PRIORITY)
            else:
                GPIO.output(t_arm_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(t_arm_pin, GPIO.LOW)
        elif(serRead == 2):
            print("s_a")
            if can_arming:
                can.CAN_Transmit(TXB0D0, arm_stratalogger_msg,
                                 MED_HIGH_PRIORITY)
            else:
                GPIO.output(s_arm_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(s_arm_pin, GPIO.LOW)
        elif(serRead == 3):
            print("c_a")
            if can_arming:
                can.CAN_Transmit(TXB0D0, arm_camera_msg, MED_HIGH_PRIORITY)
            else:
                GPIO.output(c_arm_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(c_arm_pin, GPIO.LOW)
        elif(serRead == 4):
            print("t_d")
            if can_arming:
                can.CAN_Transmit(
                    TXB0D0, disarm_telemetrum_msg, MED_HIGH_PRIORITY)
            else:
                GPIO.output(t_disarm_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(t_disarm_pin, GPIO.LOW)
        elif(serRead == 5):
            print("s_d")
            if can_arming:
                can.CAN_Transmit(TXB0D0, disarm_stratalogger_msg,
                                 MED_HIGH_PRIORITY)
            else:
                GPIO.output(s_disarm_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(s_disarm_pin, GPIO.LOW)
        elif(serRead == 6):
            print("c_d")
            if can_arming:
                can.CAN_Transmit(TXB0D0, disarm_camera_msg, MED_HIGH_PRIORITY)
            else:
                GPIO.output(c_disarm_pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(c_disarm_pin, GPIO.LOW)
        elif(serRead == 10):
            # Command to set ground level for launch
            gps_ground_level = gps_alt
            baro_ground_level = baro_alt
        elif(serRead == 11):
            # Command to set ground level for launch
            gps_ground_level = 0
            baro_ground_level = 0
        elif(serRead == 12):
            # Command to start data logging
            logging_data = True

            num = 0
            log_file = f'/home/pi/flight_data_{num}.data'
            while os.path.exists(log_file):
                num = num + 1
                log_file = f'/home/pi/flight_data_{num}.data'
            log_file_stream = open(log_file, 'wb')
            log_start_time = time.time()
        elif(serRead == 13):
            # Command to stop data logging
            logging_data = False
            log_file_stream.flush()
            log_file_stream.close()
        elif(serRead == 99):
            os.system("sudo halt")

    high_g_x, high_g_y, high_g_z = high_g_accelerometer.acceleration
    bmx_data = bmx.get_all_data()

    gps_alt = float(gps.GPSDAT["alt"])
    gps_satCount = int(gps.GPSDAT["SatCount"])
    gps_lat = float(gps.GPSDAT['lat'])
    gps_lon = float(gps.GPSDAT['lon']) * -1
    gps_ascent = float(gps.GPSDAT["accentRate"])
    gps_groundSpeed = float(gps.GPSDAT['groundSpeed'])

    cpu = CPUTemperature()
    cpu_temp = cpu.temperature

    temp = baro.getDigitalTemperature()
    real_temp = baro.getTemperature()
    pressure = baro.getDigitalPressure()
    converted = baro.convertPressureTemperature(pressure, temp)
    baro_alt = baro.getImperialAltitude(
        converted, baro.inHgToHectoPascal(29.95))

    # read can messages and things
    doCANmessages()

    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_HIGH_G_ACCELEROMETER, time.time(), (high_g_x, high_g_y, high_g_z)))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_GYROSCOPE, time.time(), (bmx_data[3], bmx_data[4], bmx_data[5])))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_ACCELEROMETER, time.time(), (bmx_data[6], bmx_data[7], bmx_data[8])))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_BAROMETER, time.time(), (baro_alt - baro_ground_level)))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_GPS, time.time(), (((gps_alt - gps_ground_level) * 3.281),gps_satCount, gps_lat, gps_lon, gps_ascent, gps_groundSpeed)))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_TELEMETRUM, time.time(), (
        telemetrum_board.arm_status,
        telemetrum_board.current,
        telemetrum_board.voltage)))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_STRATOLOGGER, time.time(), (
        stratologger_board.arm_status,
        stratologger_board.current,
        stratologger_board.voltage)))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_CAMERA, time.time(), (
        camera_board.arm_status,
        camera_board.current,
        camera_board.voltage)))
    ser.write(packet_util.create_packet(packet_util.PACKET_TYPE_BATTERY, time.time(), (mainBatteryVoltage, mainBatteryTemperature)))

    if logging_data:
        log_file_stream.write(
            bytes(ctypes.c_float(time.time() - log_start_time)))
        # don't need sync or id for data logging
        # log_file_stream.write(bytes(CFC_SYNC))
        # log_file_stream.write(bytes(CFC_ID))
        log_file_stream.write(bytes(ctypes.c_float(high_g_x)))
        log_file_stream.write(bytes(ctypes.c_float(high_g_y)))
        log_file_stream.write(bytes(ctypes.c_float(high_g_z)))
        log_file_stream.write(bytes(ctypes.c_float(bmx_data[0])))  # bmx x magn
        log_file_stream.write(bytes(ctypes.c_float(bmx_data[1])))  # bmx y magn
        log_file_stream.write(bytes(ctypes.c_float(bmx_data[2])))  # bmx z magn
        log_file_stream.write(bytes(ctypes.c_float(bmx_data[3])))  # bmx x gyro
        log_file_stream.write(bytes(ctypes.c_float(bmx_data[4])))  # bmx y gyro
        log_file_stream.write(bytes(ctypes.c_float(bmx_data[5])))  # bmx z gyro
        log_file_stream.write(
            bytes(ctypes.c_float(bmx_data[6])))  # bmx x accel
        log_file_stream.write(
            bytes(ctypes.c_float(bmx_data[7])))  # bmx y accel
        log_file_stream.write(
            bytes(ctypes.c_float(bmx_data[8])))  # bmx z accel
        log_file_stream.write(bytes(ctypes.c_float(cpu_temp)))
        log_file_stream.write(bytes(ctypes.c_float(real_temp)))
        # Subtract ground level
        log_file_stream.write(
            bytes(ctypes.c_float(baro_alt - baro_ground_level)))

        # conversion to ft, subtract ground level
        log_file_stream.write(bytes(ctypes.c_float(
            (gps_alt - gps_ground_level) * 3.281)))
        log_file_stream.write(bytes(ctypes.c_uint8(gps_satCount)))
        log_file_stream.write(bytes(ctypes.c_float(gps_lat)))
        log_file_stream.write(bytes(ctypes.c_float(gps_lon)))
        log_file_stream.write(bytes(ctypes.c_float(gps_ascent)))
        log_file_stream.write(bytes(ctypes.c_float(gps_groundSpeed)))

        # Write data recived from CAN bus
        # ser.write(bytes(ctypes.c_float(mainBatteryCurrent)))
        # ser.write(bytes(ctypes.c_float(mainBatteryVoltage)))

        # Note the order:
        for arm_board in [telemetrum_board, stratologger_board, camera_board]:
            log_file_stream.write(arm_board.arm_status.to_bytes(1, 'big'))
            log_file_stream.write(bytes(ctypes.c_float(arm_board.current)))
            log_file_stream.write(bytes(ctypes.c_float(arm_board.voltage)))

        log_file_stream.write(bytes(ctypes.c_float(v3_rail_voltage)))
        log_file_stream.write(bytes(ctypes.c_float(v5_rail_voltage)))
        log_file_stream.write(bytes(ctypes.c_float(mainBatteryVoltage)))
        log_file_stream.write(bytes(ctypes.c_float(mainBatteryTemperature)))
        log_file_stream.write(bytes('\n', 'ascii'))
