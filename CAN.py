
from CAN_consts import *
from CAN_addr import *
import busio
import board
import time
import digitalio
from adafruit_bus_device.spi_device import SPIDevice


def _BV(n):
    return (0x01 << n)


class CAN_message:

    from_addr = 0x00
    dest_addr = 0x00
    type = 0x00
    data_len = 0x00
    data = bytearray(8)

    def __init__(self, from_addr=0x00, dest_addr=0x00, type=0x00, data_len=0x00, data=bytearray(8)):
        self.from_addr = from_addr
        self.dest_addr = dest_addr
        self.type = type
        self.data_len = data_len
        self.data = data


class CAN_Controller:

    """ microcontroller pin for standby, one for reset and spi module from
        busio library """

    def __init__(self, stby_pin, reset_pin, SCK, MOSI, MISO, CS):
        self.stby_pin = stby_pin
        self.reset_pin = reset_pin

        spi_bus = busio.SPI(SCK, MOSI, MISO)
        cs = digitalio.DigitalInOut(CS)
        self.spi_device = SPIDevice(spi_bus, cs, baudrate=10000000)
        self.configure()

    def configure(self):
        self.reset_pin.value = True
        self.stby_pin.value = False

        self.resetCAN()
        time.sleep(0.1)
        # Enable RX interrupts for receive buffer pins
        self.writeCAN(BFPCTRL, 0x0F)

        # Disable most interrupts on INT pin so it can be used just for wake up
        self.writeCAN(CANINTE, 0x40)

        # Configure CAN timing (~page 24 in data sheet)
        # Set PRSEG=8, PHSEG1=8, PHSEG2=8
        # SJW=4
        # NBT = 25*Tq
        # NBT = 103.125us, Bit rate = 9697 Hz
        # Tq = 4.125us
        # BRP = 16 (now 33) (CNF1 changed from D0 to E1)
        self.writeCAN(CNF1, 0xE1)
        # Set PRSEG=8, PHSEG1=8, PHSEG2=8
        self.writeCAN(CNF2, 0xE7)
        self.writeCAN(CNF3, 0x07)

        # Use filter/masks
        # Don't allow remote transfer requests (idk what it is)
        self.writeCAN(RXB0CTRL, 0x00)
        self.writeCAN(RXB1CTRL, 0x00)

        # Use filters to filter out messages not addressed to me:
        # Address is last 3 bits of SID
        # Set mask for buffer 0, we care ab the top 3 of the low
        self.writeCAN(RXM0SIDH, 0x00)
        self.writeCAN(RXM0SIDL, 0xE0)
        self.writeCAN(RXM0EID8, 0x00)
        self.writeCAN(RXM0EID0, 0x00)
        # for buffer 1
        self.writeCAN(RXM1SIDH, 0x00)
        self.writeCAN(RXM1SIDL, 0xE0)
        self.writeCAN(RXM1EID8, 0x00)
        self.writeCAN(RXM1EID0, 0x00)
        # Set all the filters to only accept from my address
        self.writeCAN(RXF0SIDH, MY_CAN_ADDR >> 3)
        self.writeCAN(RXF0SIDL, MY_CAN_ADDR << 5)
        self.writeCAN(RXF1SIDH, MY_CAN_ADDR >> 3)
        self.writeCAN(RXF1SIDL, MY_CAN_ADDR << 5)
        self.writeCAN(RXF2SIDH, MY_CAN_ADDR >> 3)
        self.writeCAN(RXF2SIDL, MY_CAN_ADDR << 5)
        self.writeCAN(RXF3SIDH, MY_CAN_ADDR >> 3)
        self.writeCAN(RXF3SIDL, MY_CAN_ADDR << 5)
        self.writeCAN(RXF4SIDH, MY_CAN_ADDR >> 3)
        self.writeCAN(RXF4SIDL, MY_CAN_ADDR << 5)
        self.writeCAN(RXF5SIDH, MY_CAN_ADDR >> 3)
        self.writeCAN(RXF5SIDL, MY_CAN_ADDR << 5)

        # Set register as 000 000 00 (normal mode no clock out)
        self.writeCAN(CANCTRL, 0x00)

    def waitToSendCAN(self):
        # TXBxCTRL register tells if message pending, is 6 behind D0
        while (self.readCAN((TXB0D0 - 6)) & _BV(3)):
            pass
        while (self.readCAN((TXB1D0 - 6)) & _BV(3)):
            pass
        while (self.readCAN((TXB2D0 - 6)) & _BV(3)):
            pass

    # Abort all pending transmissions
    def abortAll(self):
        self.writeCAN((TXB0D0 - 6), 0x00)
        self.writeCAN((TXB1D0 - 6), 0x00)
        self.writeCAN((TXB2D0 - 6), 0x00)

    """
     * Read bytes a the receive buffer on the can controller.
     * First checks to make sure a new message is available, then clears the message
     * from the CAN buffer so it can accept a new one
     * Input is RXB0D0 or RXB1D0
     * returns struct of message that was received
     * If no message available, returns all zeros in the struct
     * FILTERS AND MASKS would be a good thing to use so that we could
     * maybe assign each CAN devices an address or something and it would automatically
     * filter out messages that are not for it. I have not done this yet
     """

    def CAN_Receive(self, receive_buffer):
        message = CAN_message()
        # Use atomic block to disable interrupts while this section runs
        # First, check if new data is available
        new_data = 0x00
        if (receive_buffer == RXB0D0):
            new_data = self.readCAN(CANINTF) & _BV(0)
        elif (receive_buffer == RXB1D0):
            new_data = (self.readCAN(CANINTF) & _BV(1)) >> 1

        # If there is no new data, return all empty
        if (new_data == 0):
            message.from_addr = 0x00
            message.data_len = 0
            message.dest_addr = 0x00
            message.type = 0x00
            return message

        # Get the message type from the identifier from the SIDH reg:
        sidh = self.readCAN(receive_buffer - 5)
        sidl = self.readCAN(receive_buffer - 4)
        # Type is 3 highest bits in sidh
        message.from_addr = sidh >> 5
        # Type is 5 bits lowest in sidh
        message.type = (sidh & 0x1F)
        # dest addr is 3 highest in sidl
        message.dest_addr = sidl >> 5

        # RXBxDLC tells how many bytes received and is one less than RXBxD0
        message.data_len = self.readCAN(receive_buffer - 1) & 0x0F
        i = 0
        # Loop through for bytes that were received
        for i in range(message.data_len):
            message.data[i] = self.readCAN(receive_buffer + i)

        # Clear the bit in CANINTF so that we can receive more data
        # Probably should use SPI bit set instead of reading and writing
        if (receive_buffer == RXB0D0):
            self.writeCAN(CANINTF, self.readCAN(CANINTF) & ~_BV(0))
        elif (receive_buffer == RXB1D0):
            self.writeCAN(CANINTF, self.readCAN(CANINTF) & ~_BV(1))

        return message

    """
     * transmit_buffer x (TXBxD0) x is 0,1,2 (pass in address of data register 0)
     * to_send is message structure to send
     * priority is LOW_PRIORITY, MED_LOW_PRIORITY, MED_HIGH_PRIORITY, HIGH_PRIORITY
     * NOTE: This function uses standard message identifiers, not extended.
     * NOTE: This function could be made much smarter by automatically figuring out
     * which buffers are empty and using those
     """

    def CAN_Transmit(self, transmit_buffer, to_send, priority):
        from_addr = MY_CAN_ADDR
        message_type = to_send.type
        dest_addr = to_send.dest_addr
        n_bytes = to_send.data_len

        # Wait for transmit buffer to finish sending last message
        # Wait for bit 3 of TXBxCTRL to go to 0
        while (self.readCAN((transmit_buffer - 6)) & _BV(3)):
            pass

        # Load data into transmit identifier Buffers
        # Most significant 3 bits of ID are the from address
        # Next is the 5 bit message type
        # the standard identifier HIGH register is 5 less than first data reg
        self.writeCAN((transmit_buffer - 5), (from_addr << 5) | message_type)
        # Shift the 3 bits of dest address to top 3 bits of low register
        # Don't use extended messages
        # the standard identifier low register is 4 less than first data reg
        self.writeCAN((transmit_buffer - 4), (dest_addr << 5) & 0xE0)

        # Load data into CAN's transmit buffers
        for i in range(n_bytes):
            self.writeCAN((transmit_buffer + i), to_send.data[i])

        # TXBxDLC address is 1 behind data(how many bytes)
        self.writeCAN((transmit_buffer - 1), n_bytes)

        # CAN TXBxCTRL address is 6 behind data
        # Request transmit and set a priority
        # NOTE: Maybe we should use BIT MODIFY instead of writing to the entire register
        self.writeCAN((transmit_buffer - 6), _BV(3) | priority)

    def resetCAN(self):
        with self.spi_device as spi:
            spi.write(bytearray([CAN_RESET]))

    def bitModifyCAN(self, addr, bits, d):
        with self.spi_device as spi:
            spi.write(bytearray([CAN_BIT_MODIFY]))
            spi.write(bytearray([addr]))
            spi.write(bytearray([bits]))
            spi.write(bytearray([d]))

    def writeCAN(self, addr, d):
        with self.spi_device as spi:
            spi.write(bytearray([CAN_WRITE]))
            spi.write(bytearray([addr]))
            spi.write(bytearray([d]))

    def readCAN(self, addr):
        with self.spi_device as spi:
            spi.write(bytearray([CAN_READ]))
            spi.write(bytearray([addr]))
            buffer = bytearray(1)
            spi.readinto(buffer)
        return buffer[0]
