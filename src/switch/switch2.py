#!/usr/bin/python

import sys
from os import path, access, R_OK, system
import serial
import datetime
import time

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

DURATION = 100  # ms
TIMEOUT = datetime.timedelta(milliseconds=DURATION)
THRESHOLD = -10.0

last_sniff = 0.0
start_time = 0
active = False
min = 0

# iterate in file
s = serial.Serial(port=infile, baudrate=57600)
for i in range(0, 30):
    s.readline()
while True:
#for line in s.xreadlines():
    line = s.readline()
    value = float(line.strip())
    if active == False:
        if last_sniff > THRESHOLD \
                and value <= THRESHOLD:
             print "Ativado."
             active = True
             start_time = datetime.datetime.now()
             min = value
    else:
        if value < THRESHOLD:
            if value < min:
                min = value
        else:
            print "Desativado."
            active = False
            length = datetime.datetime.now() - start_time
            if length < TIMEOUT:
                print str((min, length.microseconds/1000.0))
                system("/usr/bin/xdotool key Shift_R")
    last_sniff = value

