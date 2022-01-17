import smbus
import time
ADDR_BLNKM = 0x09#Smart LED
bus = smbus.SMBus(1)

def initialize():
    bus.write_block_data(ADDR_BLNKM, 0x0, [0x6f]) #stop script
    bus.write_block_data(ADDR_BLNKM, 0x0, [0x63, 0, 0, 0]) #Off
    bus.write_block_data(ADDR_BLNKM, 0x0, [0x74, -20]) # Set time adjust
    bus.write_block_data(ADDR_BLNKM, 0x0, [0x66, 150]) # Set fade adjust

def gotoRGB(R, G, B):
    try:
        bus.write_block_data(ADDR_BLNKM, 0x0, [0x6e, R, G, B]) #go to RGB color
    except:
        pass
        print 'LED i2c error...'

def fadetoRGB(R, G, B):
    try:
        bus.write_block_data(ADDR_BLNKM, 0x0, [0x63, R, G, B]) #fade to RGB color
    except:
        pass
        print 'LED i2c error...'

def fadetoHSB(H, S, B):
    try:
        bus.write_block_data(ADDR_BLNKM, 0x0, [0x68, H, S, B]) #fade to HSB color
    except:
        pass
        print 'LED i2c error...'



# Some other commands that I've used:
#bus.write_block_data(ADDR_BLNKM, 0x0, [0x70,1,1,0]) #play script
#bus.write_block_data(ADDR_BLNKM, 0x0, [0x42, 1, 0, 1, 150, -20]) # Set startup params
#$bus.write_block_data(ADDR_BLNKM, 0x0, [0x70,15,1,0]) #play script
#print "done"

