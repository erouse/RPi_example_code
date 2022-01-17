import serial
import subprocess
import shlex
from threading import Lock
import time

# Reference: http://www.technosoftmotion.com/ESM-um-html/communication_protocol_serial.htm


# The drive's axis ID code
axisIDCode = 0x0FF0

# The host's axis ID code (same as drive, with host bit set)
hostAxisIDCode = 0x0FF1

# Helper to print byte arrays
def prettifyBytearray(bytes):
    return ''.join(['%02x ' % byte for byte in bytes])

# Represents a TML instruction
class TMLMessage:
    # Access/Group ID code, Operation Code, Data
    def __init__(self, opcode, data=tuple(), idCode=axisIDCode):
        self.id = idCode
        self.opcode = opcode
        self.data = data

    # Returns the high byte of a 2-byte word
    def getHighByte(self, word):
        return word >> 8

    # Returns the low byte of a 2-byte word
    def getLowByte(self, word):
        return word & 0xFF

    # Return instruction (set of bytes) to send to the motor
    def getInstruction(self):
        dataList = []
        for d in self.data:
            dataList.extend([self.getHighByte(d), self.getLowByte(d)])

        instr = [4 + 2 * len(self.data),  # Message length
                 self.getHighByte(self.id),
                 self.getLowByte(self.id),
                 self.getHighByte(self.opcode),
                 self.getLowByte(self.opcode)] + dataList
        checksum = sum(instr) % 256
        output = (''.join([chr(byte) for byte in instr] + [chr(checksum)]))
        return output


# Handles sending TML instructions over the serial port
class TMLSerial:
    def __init__(self, port):
        #TODO: Set timeout to 2ms after testing
        self.serial = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO, timeout=0.01, writeTimeout=0.01, interCharTimeout=0.01)
        self.serial.flush()
        self.debug = False # Set debug parameter here

    def setBaudRate(self, baudRate):
        self.serial.baudrate = baudRate

    def send(self, msg):
        if ord(msg[0]) > 0x0c:
            print 'TMLSerial.send: WARNING - msg %s sent' % prettifyBytearray(bytearray(msg))

        numBytes = self.serial.write(msg)

        if numBytes != len(msg):
            print 'TMLSerial.send: WARNING - %i bytes of %i written for message %s' % (numBytes, len(msg), prettifyBytearray(bytearray(msg)))

        if self.debug:
            print 'Sent: ' + msg.encode('hex')

    def read(self, size=None):
        t_start = time.time()
        byte = ''
        if size == None:
            # print 'BAD'
            received = bytearray()
            while byte == '':
                byte = self.serial.read()
                if (time.time() - t_start) > 1.0: # Continue if waiting for no response
                    break

            while byte != '':
                received.append(byte)
                byte = self.serial.read()
        else:
            received = bytearray(self.serial.read(size))
        if self.debug:
            print 'Recv:', prettifyBytearray(received)
        return received


# Main interface to communicate with the board
class TML:
    def __init__(self, port):
        self.comm = TMLSerial(port)

        self.excessAckBytes = 0

        self.varAddrs = {
            # Format
            # 'var name': (address, is32Bits),

            'KDP':    (0x0262, False),
            'SFTKDP': (0x0263, False),
            'KIP':    (0x0260, False),
            'SFTKIP': (0x0261, False),
            'KPP':    (0x025E, False),
            'SFTKPP': (0x025F, False),
            'CPOS':   (0x029E, True),
            'CSPD':   (0x02A0, True),
            'CACC':   (0x02A2, True),
            'APOS':   (0x0228, True),
            'ASPD':   (0x022C, True),
            'MER':    (0x08FC, False),
            'TPOS':   (0x02B2, True),
        }


    # Utility functions

    # Keeps track of the number of expected responses so we can discard them when we want to read interrupt values
    def send(self, msg, ackBytes=1):
        self.comm.send(msg.getInstruction())
        resp = self.comm.read(1)
        if resp != 'O': # 'O' is handshake 0x4f from drive
            print 'Drive did not accept message: Failed with response %s- See line 125 in tml.py' % (str(prettifyBytearray(resp)))

    def read(self, size=None, ignoreConfirmationBytes=False):
        if ignoreConfirmationBytes and size != None:
            filteredBytes = bytearray()
            while len(filteredBytes) == 0:
                bytes = self.comm.read(size=size)
                self.excessAckBytes += bytes.count(chr(0x4f))
                filteredBytes = bytes.lstrip(chr(0x4f) + chr(0x0d))
                discardedBytes = bytes[:len(bytes) - len(filteredBytes)]
                if chr(0x0d) in discardedBytes:
                    print 'Received %i sync bytes! %s' % (discardedBytes.count(chr(0x0d)), prettifyBytearray(bytes))

            if len(filteredBytes) < size:
                bytes = filteredBytes + self.comm.read(size=(size - len(filteredBytes)))
        else:
            bytes = self.comm.read(size=size)

        return bytes

    # Returns the high word of a 32-bit int
    def getHighWord(self, val):
        return val >> 16

    # Returns the low word of a 32-bit int
    def getLowWord(self, val):
        return val & 0xFFFF

    def setVar(self, var, val):
        addr, is32Bits = self.varAddrs[var]
        opcode = (addr & 0x1FF) + (1 << 13)  # Get the 9 LSBs of the address and set bit 13 to get the opcode
        if is32Bits:
            if var == 'CPOS':
                val = long(val*145.1*((4.0*500.0)/360.))
                if val < 0:
                    val = int(val + 2**32)
                else:
                    val = int(val)
            data = [self.getLowWord(val), self.getHighWord(val)]
            opcode += 1 << 10  # Set bit 10 for 32-bit values 
        else:
            data = [val]

        msg = TMLMessage(opcode, data)
        self.send(msg)

    # TML commands

    # Sets trapezoidal position profile mode
    # MODE PP
    def setModePP(self):
        msg = TMLMessage(0b0101100100001001, [0b1011111111000001, 0b1000011100000001])
        self.send(msg)

    # Sets trapezoidal speed profile mode
    # MODE SP
    def setModeSP(self):
        msg = TMLMessage(0b0101100100001001, [0b1011101111000001, 0b1000001100000001])
        self.send(msg)

    # Position command is relative
    # CPR
    def setPosRel(self):
        msg = TMLMessage(0b0101100100001001, [0b1101111111111111, 0b0000000000000000])
        self.send(msg)

    # Position command is absolute
    # CPA
    def setPosAbs(self):
        msg = TMLMessage(0b0101100100001001, [0b1111111111111111, 0b0010000000000000])
        self.send(msg)
    # set Target Update Mode 1
    # TUM1
    def setTargetUpdateMode1(self):
        msg = TMLMessage(0b0101100100001001, [0b1111111111111111, 0b0100000000000000])
        self.send(msg)

    #*** WARNING ***
    #*** Commented because UPDATE is included in setting CPOS -- Large accels can cause large force transients from KDP.
    # Update on event
    # UPD
    # def update(self):
    #     # Not sure if this should be 0b0000000000001000 or 0b0000010000001000 - Binary Code Viewer and Online docs conflict
    #     msg = TMLMessage(0b0000000100001000) # This is correct
    #     self.send(msg)

    # Wait motion event
    # WAIT!
    def wait(self):
        msg = TMLMessage(0b0000010000001000)
        self.send(msg)

    # End of Initialization
    # ENDINIT
    def endInit(self):
        msg = TMLMessage(0b0000000000100000)
        self.send(msg)

    # Activate Control
    # AXISON
    def axisOn(self):
        msg = TMLMessage(0b0000000100000010)
        self.send(msg)

    # End of a TML program
    # END
    def end(self):
        msg = TMLMessage(0b0000000000000001)
        self.send(msg)

    # Stop motion
    # STOP
    def stop(self):
        msg = TMLMessage(0b0000000111000100)
        self.send(msg)

    # Abort cancelable TML function
    # ABORT
    def abort(self):
        msg = TMLMessage(0b0001110000000010)
        self.send(msg)

    # FAULT Reset
    # FAULTR
    def faultReset(self):
        msg = TMLMessage(0b0001110000000100)
        self.send(msg)

    # Ask one axis to return a 16/32 bit value from memory
    #   is32Bits - True if the data at the specified address is 32 bits, False otherwise
    # ?var - GiveMeData
    def giveMeData(self, addr, is32Bits=False):
        if is32Bits:
            opcode = 0xB005
        else:
            opcode = 0xB004

        msg = TMLMessage(opcode, [hostAxisIDCode, addr])
        self.send(msg)
        received = self.read()
        return received
   
    # Set Serial Communication Interface Baud Rate
    # SCIBR
    def setBaudRate(self, baudRate):
        # Note: baudRate should be one of [0,1,2,3,4] for baud rates [9600,19200,38400,56600,115200], respectively
        msg = TMLMessage(0b0000100000100000, [baudRate])
        print 'Changing RS232 baudrate...'
        self.send(msg)

    # Macros

    # Increases the baud rate to max
    def increaseBaudRate(self):
        self.setBaudRate(4) # This order is important
        self.comm.setBaudRate(115200)
    
    # Returns a dictionary of the information returned from the given TakeData answer message, or None on error 
    def parseTakeData(self, data, is32Bits=False, senderAxisID=False):
        dataDict = {}
        
        try:
            dataDict['length']       = data[0]
            dataDict['destAxisID']   = (data[1] << 8) + data[2]
            dataDict['opcode']       = (data[3] << 8) + data[4]
            if senderAxisID:
                dataDict['senderAxisID'] = (data[5] << 8) + data[6]
                offset = 2
            else:
                offset = 0
            dataDict['addr']         = (data[5+offset] << 8) + data[6+offset]
            dataDict['value']        = (data[7+offset] << 8) + data[8+offset]

            if is32Bits:
                dataDict['value']   += ((data[9+offset] << 8) + data[10+offset]) << 16
                checksumBit = 11+offset
            else:
                checksumBit = 9+offset

            dataDict['checksum']     = data[checksumBit]

            # Some sanity checks on the response
            if dataDict['checksum'] != sum(data[:checksumBit]) % 256:
                print 'TML.parseTakeData: invalid checksum'
                print [(x, hex(y)) for x, y in dataDict.items()]
                return None

            if dataDict['destAxisID'] != hostAxisIDCode:
                print 'TML.parseTakeData: destAxisID does not match hostAxisIDCode'
                return None

            if senderAxisID and dataDict['senderAxisID'] != axisIDCode:
                print 'TML.parseTakeData: senderAxisID does not match axisIDCode'
                return None
        except IndexError:
            print 'TML.parseTakeData: invalid message length'
            return None

        return dataDict

    # Returns a variable from memory, or None on error
    def getVarInterrupt(self):
        resp1 = self.read(size=12, ignoreConfirmationBytes=True)
        resp2 = self.read(size=12, ignoreConfirmationBytes=True)

        takeData1 = self.parseTakeData(resp1, is32Bits=True)
        takeData2 = self.parseTakeData(resp2, is32Bits=True)

        if takeData1 == None:
            print prettifyBytearray(resp1)
            print 'TML.getVarInterrupt: parseTakeData 1 failed'
            return None
        if takeData2 == None:
            print prettifyBytearray(resp2)
            print 'TML.getVarInterrupt: parseTakeData 2 failed'
            return None

        output = []

        output.append(takeData1['value'])
        output.append(int(takeData2['value'] >> 16))
        output.append(int(takeData2['value'] & 0x0000FFFF))

        return output
                         
    # Returns a variable from memory, or None on error
    def getVar(self, var):
        addr, is32Bits = self.varAddrs[var]
        resp = self.giveMeData(addr, is32Bits)
        #print ''.join(['%02x ' % byte for byte in resp])
        #print 'Recv: ' + ''.join(['%02x ' % byte for byte in resp])
        # if is32Bits:
        #     resp = resp[len(resp)-14:]
        # else:
        #     resp = resp[len(resp)-12:]
        # if resp[0] != 0x4f:# | 0x0a:
        #     print 'TML.getVar: giveMeData failed with response ' + ''.join(['%02x ' % byte for byte in resp])
        #     return None
        #print 'Recv: ' + ''.join(['%02x ' % byte for byte in resp])
        takeData = self.parseTakeData(resp, is32Bits, True)

        if takeData == None:
            print 'TML.getVar: parseTakeData failed'
            return None

        return takeData['value']

    # Should be called once before doing anything else
    def startUp(self):
        print 'Initializing iPOS board...'

        self.endInit()
        self.axisOn()
        self.setPosAbs()
        self.setModePP()
        self.setTargetUpdateMode1()
