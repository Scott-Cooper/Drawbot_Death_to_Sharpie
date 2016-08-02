#!/usr/bin/env python
""" drawbot v3.0
    http://dullbits.com/drawbot
"""

################################################################################
# %!expand -4
#
# Much thanks to:
#    David Braam, OctoPrint, G-code stuff
#    btcroston@googlemail.com, raspberry-gpio-python
#    http://threeblindmiceandamonkey.com/?p=16 matrix.h
#
################################################################################
# Command structure sent to Arduino:
#
#    Byte 1:
#        bit 7:  Unused
#        bit 6:  LED
#        bit 5:  Servo up, if high
#        bit 4:  Relax motors, if high
#        bit 3:  Motor1 step, if high
#        bit 2:  Motor1 step direction
#        bit 1:  Motor2 step, if high
#        bit 0:  Motor2 step direction
#    Byte 2:
#        bits 0-7: Byte contains offset for delay map on Arduino
#
################################################################################
# G-codes that are or should be implemented by this script
#
# G00 [Xx.xx] [Yx.xx] [Zx.xx] [Fx.xx]
# G01 [Xx.xx] [Yx.xx] [Zx.xx] [Fx.xx]
#      Draw a straight line from the current position to (X,Y) with a maximum
#      speed of F.  If you don.t specify an X,Y, or F, the current value will
#      used.  Z is used to pen up/down
#
# G04 [Px.xx]
#      Dwell, aka wait P milliseconds.  Remember that you need 1000
#      milliseconds to pause for one second.
#
# G20
#      Programming in inches (in).  The robot works internally in millimeters
#      (mm), so it will divide all coordinates you give it by 25.4 (mm/in).
#
# G21
#      Programming in millimeters (mm).  This is the default.
#
# G90
#      Absolute mode.
#
# G91
#      Relative mode.
#
################################################################################
# My stepper motors have 200 steps per revolution, 800 with quarter stepping
# Diameter of pulley = 20.828 mm (where string is, not the flange)
# Radius of pully = = 10.414 mm
# Circumference = Pi * D
# Circumference = 3.1415 * 20.828 = 65.43309178855596 mm
# 65.4330917142 mm / 800 = 0.0817913647362 mm per quarter step.
# About 12.22622 quarter steps per mm.
# Acutual testing reveled 296 quarter steps per inch
# Acutual testing reveled 11.65354 quarter steps per mm
# About 77.635 full steps per inch.
#
# Distance between centers of motors = 1149.35 mm
# Distance of C1 and C2 = 812.8 mm from center of motor
# Flange diameter =  26.3906 mm
# Half flange = 13.1953 mm 
# Length of C1 and C2 minus flange = 799.6047 mm 
# X_Total * 11.65354 = 13393.996199 
################################################################################
#
#           (0,0)             c3 
#           O+++++++++++++++++++++++++++++++++++++O
#           +* ac2                        ac1  *  +
#           + *                             *     +
#           +  *                         *        +
#           +   * c1               c2 *           +
#        y1 +    *                 *              + y2
#           +     *             *                 +
#           +      *         *                    +
#           +       * ac3 *                       +
#           +        * *                          +
#           ++++++++++O++++++++++++++++++++++++++++
#               x1                   x2
#
################################################################################
# The string departs the pully at the point that is perpendicular to the string.
# At the home position its a perfect 45 degrees.
#
# c3 = center of motors - sqrt ( radius of pully^2 + radius of pully^2 ) * 2
#
#                         +
#                         +
#                      +++++++    
#                  ++     +     ++
#                +        +        +
#               +         +         +
#              +          +          +
#             +           +           +
#        ++++ +++++++++++ O +++++++++ + ++++
#             +          /+           +
#              +       /  +          +
#               +    /    +         +
#                + /      +        +
#                /\++     +     ++
#              /    \  +++++++     
#            /        \   +
#          /            \ +
#        /                \        
#      /                    \        
#    /                        \        
#                               \        
#
################################################################################


from __future__ import division
import sys
import os
import signal
import math
import time
import serial
import RPi.GPIO as GPIO
import settings as s
from collections import deque

__date__    = "2013-11-19"
__author__  = "Scott Cooper"
__license__ = "GPL"
__version__ = "1.0.1"
__email__   = "scottslongemailaddress@gmail.com"


class MoveData:
    pass

def mm(inches):
    return inches * 25.4 

def inch(mm):
    return mm / 25.4

def signal_handler(signal, frame):
    move_to(oldPos[0], oldPos[1], 0, 0, 0)
    print "motor1_c:      ", motor1_c
    print "motor2_c:      ", motor2_c
    print "step_jump1:  ", step_jump1
    print "step_jump2:  ", step_jump2
    print
    print 'Clean exit.  Check stepper for enable.'
    ser.write(chr(48))
    ser.write(chr(50))
    ser.flush()
    ser.close()
    GPIO.cleanup()
    sys.exit(0)

def pythagorean(a, b): 
    return math.sqrt(a*a + b*b)

def calc_xy(): 
    # Law of Cosines, ac2 is expressed in radians
    ac2 = math.acos(float(c1*c1 + c3*c3 - c2*c2) / float(2*c1*c3))
    x_current = cos(HALF_PI - ac2) * c1
    y_current = sin(HALF_PI - ac2) * c1

def calc_tension(c1, c2):
    # Law of Cosines, ac1 & ac2 is expressed in radians
    m = 72 
    ac1 = math.acos(float(c2*c2 + c3*c3 - c1*c1) / float(2*c2*c3))
    ac2 = math.acos(float(c1*c1 + c3*c3 - c2*c2) / float(2*c1*c3))
    t1 = (math.cos(ac2) * m) / (math.cos(ac1) * math.sin(ac2) + math.sin(ac1) * math.cos(ac2))
    t2 = (math.cos(ac1) * m) / (math.cos(ac1) * math.sin(ac2) + math.sin(ac1) * math.cos(ac2))
    return t1, t2

def step_to(x, y, z):
    global  motor1_c
    global  motor2_c
    global  step_jump1
    global  step_jump2
    global  led 

    # c1 & c2 are expressed im mm * br, which is not in motor steps
    c1 = pythagorean(init_x + x, init_y + y) 
    c2 = pythagorean(init_x - x, init_y + y)
    #t1, t2 = calc_tension (c1, c2)

    # Scale c1 & c2 in terms of physical motor steps
    c1 = int(round(c1 * steps_per_mm / s.br))
    c2 = int(round(c2 * steps_per_mm / s.br))

    if z == 1:
        #speed = 25  # insane fast gondola test
        speed =  45  # Super slow for super fine copic
        #speed = 35  # Not safe, printed girl copic org
        #speed = 27  # Really fast for KC Jimmi
        #speed = 10 + int((160-t1-t2)/2)
    else:
        speed = 15  

    finished = False
    while not finished:

        byte1 = 0 

        # Step motor 1
        if c1 != motor1_c:
            byte1 |= 8
            if c1 - motor1_c > 0:
                motor1_c += 1
            else:
                motor1_c -= 1
                byte1 |= 4

        # Step motor 2
        if c2 != motor2_c:
            byte1 |= 2
            if c2 - motor2_c > 0:
                motor2_c += 1
                byte1 |= 1
            else:
                motor2_c -= 1

        #Set LED
        byte1 = byte1 | (int(led) << 6)

        #Set servo pen postion
        byte1 = byte1 | (int(not z) << 5)
    
        if not calc_only:
            while (GPIO.input(11) == GPIO.LOW):
                time.sleep(0.01)

            ser.write(chr(byte1))
            ser.write(chr(speed))
    
        if c1 != motor1_c:
            step_jump1 += 1

        if c2 != motor2_c:
            step_jump2 += 1

        finished = ((c1 == motor1_c) and (c2 == motor2_c))


################################################################################
# Algorithm was developed by Jack Elton Bresenham in 1962
# http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
# Traslated from pseudocode labled "Simplification" from the link above.
################################################################################
def bresenham(x0, y0, x1, y1, z0):
    dx = abs(x1-x0)
    dy = abs(y1-y0) 
    if (x0 < x1):
        sx = 1
    else:
        sx = -1
    if (y0 < y1):
        sy = 1
    else:
        sy = -1
    err = dx-dy
    while (1):
        step_to(x0, y0, z0)
        if ((x0 == x1) and (y0 == y1)):
            break
        e2 = 2*err
        if (e2 > -dy): 
            err = err - dy
            x0 = x0 + sx
        if ((x0 == x1) and (y0 == y1)): 
            step_to(x0, y0, z0)
            break
        if (e2 < dx): 
            err = err + dx
            y0 = y0 + sy 

def move_to(x0, y0, x1, y1, z0):
    bresenham(int(x0*s.br), int(y0*s.br), int(x1*s.br), int(y1*s.br), z0)
    #print "motor1_c:      ", motor1_c
    #print "motor2_c:      ", motor2_c

def getCodeInt(line, code):
    n = line.find(code) + 1
    if n < 1:
        return None
    m = line.find(' ', n)
    try:
        if m < 0:
            return int(line[n:])
        return int(line[n:m])
    except:
        return None

def getCodeFloat(line, code):
    n = line.find(code) + 1
    if n < 1:
        return None
    m = line.find(' ', n)
    try:
        if m < 0:
            return float(line[n:])
        return float(line[n:m])
    except:
        return None

def parse_gcode(gcodeFile):
    global led
    global oldPos
    global pos

    scale = 1.0
    posAbs = True

    for line in gcodeFile:

        if (GPIO.input(4) == GPIO.LOW):
            signal_handler(signal.SIGINT, 0)

        print line.rstrip('\n')
        comment = ''

    	led = not led

        if ';' in line:
            comment = line[line.find(';')+1:].strip()
            line = line[0:line.find(';')]

        if '(' in line:
            comment = line[line.find('(')+1:].strip().rstrip(')')
            line = line[0:line.find('(')]

        oldPos = pos
        G = getCodeInt(line, 'G')
        if G is not None:
            if G == 0 or G == 1:    #Move
                x = getCodeFloat(line, 'X')
                y = getCodeFloat(line, 'Y')
                z = getCodeFloat(line, 'Z')
                oldPos = pos
                pos = pos[:]
                if posAbs:
                    if x is not None:
                        pos[0] = x * scale
                    if y is not None:
                        pos[1] = y * scale
                    if z is not None:
                        pos[2] = z * scale
                else:
                    if x is not None:
                        pos[0] += x * scale
                    if y is not None:
                        pos[1] += y * scale
                    if z is not None:
                        pos[2] = z * scale
                move_to(oldPos[0], oldPos[1], pos[0], pos[1], pos[2])
            elif G == 4:    #Delay
                S = getCodeFloat(line, 'S')
                if S is not None:
                    continue
                P = getCodeFloat(line, 'P')
                if P is not None:
                    continue
            elif G == 20:   #Units are inches
                scale = 25.4
            elif G == 21:   #Units are mm, default
                scale = 1.0
            elif G == 90:   #Absolute position
                posAbs = True
            elif G == 91:   #Relative position
                posAbs = False
            elif G == 28:   #Home
                x = getCodeFloat(line, 'X')
                y = getCodeFloat(line, 'Y')
                z = getCodeFloat(line, 'Z')
                center = [0.0, 0.0, 0.0]
                if x is None and y is None and z is None:
                    pos = center
                else:
                    pos = pos[:]
                    if x is not None:
                        pos[0] = center[0]
                    if y is not None:
                        pos[1] = center[1]
                    if z is not None:
                        pos[2] = center[2]
                    move_to(oldPos[0], oldPos[1], pos[0], pos[1], pos[2])
            else:
                print "     Command un-known: G" + str(G)

def process_gcode(filename):
    if os.path.isfile(filename):
        filename = filename
        _fileSize = os.stat(filename).st_size
        with open(filename, "r") as f:
            parse_gcode(f)


################################################################################


opt = ''
if len(sys.argv) == 2:
    program_name = sys.argv[0]
    opt = sys.argv[1]

# Hey everybody its global time
#calc_only = True 
calc_only = False 
movedata = []

steps_per_mm = 800 / s.circ
ros = s.circ / (2 * math.pi)   # Radius of string on pully
c3 = int(round((s.com + (math.cos(math.radians(45)) * ros * 2)) * s.br))
init_x = c3 / 2
init_y = c3 / 2
motor1_c = int(round(pythagorean(init_x, init_y) * steps_per_mm / s.br))
motor2_c = motor1_c
step_jump1 = 0
step_jump2 = 0
led = 0
oldPos = [0.0, 0.0, 0.0]
pos = [0.0, 0.0, 0.0]

print
print 'Raspberry pi Serial v3.0'
print 'Options: ', opt
print 'Press Ctrl+C to exit'
print

print "c3:            ", c3
print "init_x:        ", init_x
print "init_y:        ", init_y
print "motor1_c:      ", motor1_c
print "motor2_c:      ", motor2_c
print "Radius:        ", ros 
print "Circumference: ", s.circ
print "steps_per_mm:  ", steps_per_mm
print "s.br:          ", s.br

signal.signal(signal.SIGINT, signal_handler)
GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.IN)
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0)
ser.open()
ser.isOpen()

# Reset the Arduino and wait for it to reboot
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, False)
time.sleep(0.25)
GPIO.setup(18, GPIO.IN)
time.sleep(2)

step_to(0, 0, 0)
# Wait for the "GO" button to be pressed
print
print "Press the \"GO\" button to proceed"
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#while (GPIO.input(4) == GPIO.HIGH):
#    time.sleep(0.1)

process_gcode("./gcode.txt")

for idx,md in enumerate(movedata):
    if hasattr(md, 'gcode'):
        print md.gcode
    else:
        #print \
            repr(idx).ljust(4), \
            ' x:',   repr(md.x).ljust(3), \
            ' y:',   repr(md.y).ljust(3), \
            ' z:',   repr(md.z).ljust(2), \
            ' c1:',  repr(md.c1).ljust(4), \
            ' c2:',  repr(md.c2).ljust(4), \
            ' m1:',  repr(md.motor1_c).ljust(4), \
            ' m2:',  repr(md.motor2_c).ljust(4), \
            ' ', bin(md.byte1)[2:].zfill(8), \
            ' t1:',  repr(md.t1).ljust(4), \
            ' t2:',  repr(md.t2).ljust(4)

print "motor1_c:      ", motor1_c
print "motor2_c:      ", motor2_c
print "step_jump1:    ", step_jump1
print "step_jump2:    ", step_jump2

move_to(oldPos[0], oldPos[1], 0, 0, 0)
#step_to(0, 0, 0)
ser.write(chr(48))
ser.write(chr(50))
ser.flush()
ser.close()
GPIO.cleanup()
sys.exit(0)

