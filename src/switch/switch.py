#!/usr/bin/python

import sys
from os import path, access, R_OK, system
import serial

if len(sys.argv) > 2:
    print "Usage: %s [capture_file]" % sys.argv[0]
    exit(1)

# set input file name
infile = '/dev/ttyACM0'
if len(sys.argv) == 2:
    infile = sys.argv[1]

# check for permissions
if not (path.isfile(infile) or access(infile, R_OK)):
    print "Error: can't read from %s." % infile
    exit(1)

# iterate in file
s = serial.Serial(port=infile, baudrate=57600)
for i in range(0, 30):
    s.readline()
while True:
#for line in s.xreadlines():
    line = s.readline()
    value = float(line.strip())
    if value < -20.0:
        print value
        system("/usr/bin/xdotool key Shift_R")
        for i in range(0, 50):
            s.readline()
