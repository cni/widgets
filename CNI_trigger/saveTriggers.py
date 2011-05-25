#!/usr/bin/env python

import serial
import sys, signal, time
from datetime import datetime

device = '/dev/ttyACM1'

running = True

def handler(signum, frame):
    global running
    running = False

# Set the signal handler to allow clean exit with a TERM signal
signal.signal(signal.SIGTERM, handler)

if __name__ == "__main__":
    filename = sys.argv[1]
    timeRef = datetime.now()
    with open(filename, 'a') as f:
        f.write('%% Reference time: %s\n' % str(timeRef))
        ser = serial.Serial(device, 115200, timeout=1)
        time.sleep(0.1)
        # Send an out pulse
        ser.write('[t]\n');
        # Display the firmware greeting
        out = ser.readlines()
        for l in out: print(l),
        # Send the command to enable input pulses
        ser.write('[p]\n');

        while running:
            n = ser.inWaiting()
            if n>0:
              s = ser.read(n)
              if s[0]=='p':
                ts = datetime.now() - timeRef
                totalSeconds = (ts.microseconds + (ts.seconds + ts.days * 24 * 3600) * 10**6) / float(10**6)
                # Print the time stamp as an offset from the reference time, in milliseconds.
                f.write('%0.9f\n' % totalSeconds)

        ser.close()

exit(0)

#import pylab
#pylab.plot(tic,resp)
#pylab.plot(tic,ppg)
