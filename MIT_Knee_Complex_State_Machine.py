#!/usr/bin/python

import smbus, time, math, random, threading
from Adafruit_ADS1x15 import ADS1x15
import MPU6050_ER as mpu6050
import BLNKM as LED
from tml import TML
import RPi.GPIO as GPIO
import traceback

board = TML('/dev/ttyAMA0')

# ============================================================================
# State Machine, v1 3/15/13,  E J Rouse
# ============================================================================

# Configure i2c baudrate
#os.system("sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=1000000")
#subprocess.call(shlex.split('sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=1400000'))

# Initialise the ADC using the default mode (IC = ADS1115, default address)
adc = ADS1x15()

from DataLogger import dataLogger

data = []


# ============================================================================
# Initializations

#mpu6050.initialize()
print 'reading...'
board.read()

board.startUp()
time.sleep(0.01)
LED.initialize()
GPIO.setmode(GPIO.BOARD)
#Setup Comm Pin
GPIO.setup(12,GPIO.OUT)
GPIO.output(12, GPIO.HIGH)
#Setup Brake Pinout
GPIO.setup(16,GPIO.OUT)
GPIO.output(16,GPIO.HIGH)

trial_num = int(raw_input('Trial Number? '))
filename = 'Test%i_080713_MITkneeMITankle' % trial_num
dl = dataLogger(filename + '.txt')

i = 0.0

# ============================================================================
# State variable definitions

# State 1
thetak_brk_thresh = 0 # Threshold angle to turn on the brake
KD_S1 = 6000 # KDP during stance phase
vL_thresh1 = 5.0 # Voltage threshold to exit stance
theta_thresh1 = -8 # Theta threshold to exit stance
t_brk1 = 0.45 #B: 0.5, Time to wait before allowing out of stance phase
t_brk2 = -.3 # Time to turn brake off if knee is extended (power savings)
comp_thresh1 = 10 # Degrees of compression permitted before transition to State 2

# State 2
thetadot_thresh2 = -5.0 # Threshold to switch from State 2 to State 3 (swing flexion to swing extension)
theta_thresh2 = -20 # Angle must be less than this to switch from State 2 to State 3 (swing flexion to extension)
Q_const2 = 0.13 #L:0.1 B: 0.15, Quadratic coefficient for State 2 (swing flexion)
KP_S2 = 90 #BIOM: 75, Stiffness flexing the knee in swing flexion
KD_S2 = 50  # Constant damping during State 2 (swing flexion)

# State 3
theta_thresh3 = -30.0 # Threshold when damping function changes to quadratic; State 3 to State 4
KD_S3 = 25 #BIOM: 70, L: 25 Constant damping during State 3 (early swing extension) 

# State 4
vL_thresh4 = 1.6 # Voltage to indicate extension moment (heel contact), State 4 to State 1
Q_const4 = 1.9 #BIOM: 2.8 L:1.8, B: 1.9, Quadratic coefficient for State 4 (late swing extension)
thetadot_thresh4 = 5 # Threshold to switch from State 4 to State 1 (swing extension to stance)
theta_thresh4 = -4 #B: -5, Threshold that permits State 4 to State 1 (Swing extension to flexion)


thetak_thresh = -35.0 # Some agreegment between CPOS and this variable is necessary
theta_ramp_len = 7.0 # FLOAT Ramp length for stiffness value.  Stiffness begins at 0 and ramps to full in this length
vL_thresh_stance = 4.1 # Threshold to return to stance, ie weightbearing
KP = 100 # BIOM: 0 L: 100 B: 150 Stiffness during spring floating window 

vL_max = 0 # Load voltage maximum for determining toe off.
# Pretty hacky way to output the above var definitions
params = ['vL_thresh1', 'theta_thresh1', 't_brk1', 'comp_thresh1', 'thetadot_thresh2', 'theta_thresh2', 'Q_const2', 'KD_S2', 'theta_thresh3', 'KD_S3', 'vL_thresh4', 'Q_const4', 'thetadot_thresh4', 'theta_thresh4', 'thetak_thresh', 'theta_ramp_len', 'vL_thresh_stance', 'KP']
f = open(filename + '_params.txt', 'w')
for param in params:
    f.write('%s: %s\n' % (param, eval(param)))
f.close()


# Parameter Initializations
v0_offset = 1.70 # Offset voltage for encoder--MUST BE APPROXIMATELY 2V
KDP = 0
KPP = 0  
vL = adc.readADCSingleEnded(0, -1.0) # Significant testing on 3/13/13 indicates that channels 0/1 are switched in HW or library software
time.sleep(0.0025) # Delay to ensure correct capture of next ADC channel
v0 = adc.readADCSingleEnded(1, -1.0)
theta_prev = ((v0-v0_offset)*(360.0/4.987))
vL_prev = vL
thetadot_prev = 0.0
thetadot_prev2 = 0.0
t_theta_prev = 0.0
KPP_prev = 0 # Initializing previous KPP values for only changing if different
KDP_prev = 0
state = 2
ts = time.time()
vL_max_percentage = 0.95
erroneous_value_flag = 0
first_S1 = 1

print v0
print 'reading...'
board.read()
print 'starting...'
t0 = time.time()
#while (time.time() - t0) < 300:   

while True:
    try:
    # =====================================================================================
        # Data acquisition and manipuation
        brake = 0
        i = i + 1
        t1 = time.time() - t0
        GPIO.output(12, GPIO.LOW) # 1.8 ms delay between pin voltage change
        vL = adc.readADCSingleEnded(0, -1.0) # Significant testing on 3/13/13 indicates that channels 0/1 are switched in HW or library software
        time.sleep(0.002) # Delay to ensure correct capture of next ADC channel
        v0 = adc.readADCSingleEnded(1, theta_prev*(4.987/360.0)+v0_offset)
        DrvVals = board.getVarInterrupt()
        #AccYZ = mpu6050.getAccYZ()
        #Ang = math.atan2(AccYZ[0],AccYZ[1])

        if DrvVals[0] > (2**32)/2: # Account for rollover APOS position variable
            DrvVals[0] = DrvVals[0] - (2**32) 
        if DrvVals[1] > (2**16)/2: # Account for rollover of IQ current variable 
            DrvVals[1] =  DrvVals[1] - (2**16)
        if DrvVals[2] > (2**16)/2: # Account for rollover of UQREF voltage variable
            DrvVals[2] = DrvVals[2] - (2**16)
        DrvVals[0] = (DrvVals[0]*(360.0/(4.0*500.0)))/145.1 # Converting to degrees, scaling factor Kpf, from Technosoft Control Manual, Section 3.7, then dividing by gear ratio
        DrvVals[1] = DrvVals[1]*((2.0*20.0)/65520.0) # Converting to amps, from email from Gabriel to Elliott on 1/29/13
        DrvVals[2] = DrvVals[2]*(24.0/65534.0) # Converting to volts, scaling factor Kuf, from Technsoft Manual, Section 3.7
        thetak = ((v0-v0_offset)*(360.0/4.987)) # Converting to degrees, using datasheet
        t2 = time.time() - t0
        GPIO.output(12, GPIO.HIGH)
        #GyroX = mpu6050.getGyroX()
        thetadot = (2*((thetak - theta_prev)/(time.time()-t_theta_prev)) + thetadot_prev)/3
        thetam = DrvVals[0]
        if (math.fabs(thetak - theta_prev) > 15) and (erroneous_value_flag == 0): # Statement to handle erroneous thetak's
            thetak = theta_prev
            erroneous_value_flag = 1 # For only letting this happen once
        else:
            erroneous_value_flag = 0
        if math.fabs(vL - vL_prev) > 1.5: # Statement to handle erroneous thetak's
            vL = vL_prev 
        spring_comp = thetam - thetak # Postive value means energy in flexion spring
        
        
        
        # ======================================================================================
        # State Machine

        # Sliding K window state, implemented in State 3 and 4 only
        if thetak < thetak_thresh and (state == 3 or state == 4):
            if (thetak - theta_startk) < theta_ramp_len:
                KPP = int((math.fabs(thetak - theta_startk)/theta_ramp_len)*KP)
            else:
                KPP = KP
        else:
            theta_startk = thetak
            KPP = 0

        # Stance, State 1
        if state == 1:
            LED.fadetoRGB(10,255,10) # Green
            if first_S1 == 1:
                thetak_S1_start = thetak
                first_S1 = 0 
            if vL > vL_max:
                vL_max = vL
            
            if thetak < thetak_brk_thresh:
                if thetak > -1 and time.time() - ts > t_brk2:
                    brake = 0
                else:
                    brake = 1

            else:
                pass
                #KDP = int(math.fabs(thetak - thetak_S1_start)*2000)
                #KDP = 2500
                #brake = 0
            
            if time.time() - ts > t_brk1:
                if vL < vL_max_percentage*vL_max:
                    state = 2
                    first_S1 = 1
                    
                #if vL < vL_thresh1 and thetak > theta_thresh1 and math.fabs(spring_comp) < comp_thresh1 :
                #if thetak > theta_thresh1 and math.fabs(spring_comp) < comp_thresh1 :     

        # Swing Flexion, State 2
        elif state == 2:
            stance_flexion_trigger = 0
            LED.fadetoRGB(10,10,255) # Blue
            brake = 0
            #if vL > vL_thresh_stance:
            #    ts = time.time()
            #    state = 1
            #    continue
            KDP = int(((thetak - 0)**2)*Q_const2 + KD_S2)
            if thetak > thetak_thresh:
                KPP = KP_S2
            if thetadot > thetadot_thresh2 and thetak < theta_thresh2 : 
                state = 3

        # Swing Extension-pre damping profile, State 3
        elif state == 3:
            LED.fadetoRGB(255,10,10) # Red
            brake = 0
            #if vL > vL_thresh_stance:
            #    ts = time.time()
            #    state = 1
            #    continue
            KDP = KD_S3
           
            if thetak > theta_thresh3:
                state = 4

        # Swing Extension-damping profile, State 4
        elif state == 4:
            LED.fadetoRGB(255,255,255) # White
            brake = 0 
            vL_max = 0 # Reset vL max variable
            #if vL > vL_thresh_stance:
            #    ts = time.time()
            #    state = 1
            #    continue
            KDP = int(((thetak - theta_thresh3)**2)*Q_const4 + KD_S3)
            if (thetadot < thetadot_thresh4 and thetak > theta_thresh4): # or vL < vL_thresh4:
                ts = time.time()
                state = 1
        
        # Print some values for status...
        if i%50 == 0: 
            print "Knee Angle: %.2f; Motor Angle: %.2f;  Angle Voltage: %.2f;  Load Voltage: %.1f;  Spring Compression: %.1f;" % (thetak, thetam, v0, vL, spring_comp)
        # Turn the brake on
        if brake == 1:
            GPIO.output(16,GPIO.LOW)
        else:
            GPIO.output(16,GPIO.HIGH)    

        # Only change motor state values if they've been updated... 
        if (i > 2) and (KDP != KDP_prev):        
            board.setVar('KDP', KDP)
            #board.setVar('KDP',0)
            time.sleep(0.001)
        if (i > 2) and (KPP != KPP_prev):
            board.setVar('KPP', KPP)
            #board.setVar('KPP', 0)
            time.sleep(0.001)

        # Set values of previous variables
        thetadot_prev2 = thetadot_prev
        thetadot_prev = thetadot
        theta_prev = thetak
        t_theta_prev = time.time()
        KDP_prev = KDP
        KPP_prev = KPP
        vL_prev = vL
        
        # Data array
        data = [i] + [t1] + [brake+0.0] + [thetak] + [thetadot] + [vL]   + [thetam] + [DrvVals[1]] + [DrvVals[2]] + [state + 0.0] + [KPP + 0.0] + [KDP + 0.0] + [t2]
        #data = [i] + [t1] + [brake+0.0] + [thetak] + [thetadot] + [vL]  + [AccYZ[0]] + [AccYZ[1]] + [GyroX[0]] + [Ang] + [thetam] + [DrvVals[1]] + [DrvVals[2]] + [state + 0.0] + [KPP + 0.0] + [KDP + 0.0] + [t2]

        dl.appendData(data)
    except KeyboardInterrupt:
        print 'State machine stopped by user'
        break
    except Exception as e:
        print 'Unhandled exception in main loop:', e

print "Iterations: %.2f " % (i)
print "Elapsed Time: %.2f " % (time.time()-t0)
print "Mean Frequency: %.2f " % ((i + 0.0)/(time.time()-t0))
LED.fadetoRGB(0,0,0)
print 'reading...'
board.read()
board.setVar('KDP',2000)
time.sleep(0.01)
board.setVar('KPP',0)

dl.writeOut()

