# Simulates the sending of arm/disarm packets

import serial
import packet_util as pk
import time

if __name__ == '__main__':
    # Setup sender port.
    port = serial.Serial(
        port='COM1',
        # baudrate=9600,
        baudrate=1410065407,
        stopbits=serial.STOPBITS_ONE,
        parity=serial.PARITY_NONE,
        bytesize=serial.EIGHTBITS,
    )

    option = -1
    timestamp = 0.0
    while option != 0:

        text = ''
        print('0 - exit')
        print('1 - arm telemetrum')
        print('2 - arm stratologger')
        print('3 - arm camera')
        print('4 - disarm telemetrum')
        print('5 - disarm stratologger')
        print('6 - disarm camera')
        print('7 - set starting altitude')
        print('8 - reset starting altitude')
        while text not in ['0', '1', '2', '3', '4', '5', '6', '7', '8']:
            text = input('> ')
        option = int(text)
        
        timestamp += 1.0

        if option == 0:
            pk.create_packet(pk.PACKET_TYPE_ARM_TELEMETRUM, timestamp, ())
            time.sleep(1.0)

        elif option == 1:
            pk.create_packet(pk.PACKET_TYPE_ARM_STRATOLOGGER, timestamp, ())
            time.sleep(1.0)

        elif option == 2:
            pk.create_packet(pk.PACKET_TYPE_ARM_CAMERA, timestamp, ())
            time.sleep(1.0)

        elif option == 4:
            pk.create_packet(pk.PACKET_TYPE_DISARM_TELEMETRUM, timestamp, ())
            time.sleep(1.0)

        elif option == 5:
            pk.create_packet(pk.PACKET_TYPE_DISARM_STRATOLOGGER, timestamp, ())
            time.sleep(1.0)

        elif option == 6:
            pk.create_packet(pk.PACKET_TYPE_DISARM_CAMERA, timestamp, ())
            time.sleep(1.0)
        
        elif option == 7:
            pk.create_packet(pk.PACKET_TYPE_SET_STARTING_ALTITUDE, timestamp, ())
        
        elif option == 8:
            pk.create_packet(pk.PACKET_TYPE_RESET_STARTING_ALTITUDE, timestamp, ())
    