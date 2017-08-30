Raspberry Pi Widget Info
==========

This document provides general information about the QRS complex detection widget on Raspberry Pi 3.

Joe Sommer, 2017

## Software Setup ##
  The UART of the Pi has been reconfigured so that it connects to serial instead of Bluetooth within the device. 
To read in serial data within a Python program, please import the module 'serial' and initialize with the following:

ser = serial.Serial() 
ser.bytesize = 8      
ser.baudrate = 115200  
ser.port = '/dev/ttyAMA0'  
ser.stopbits = 1

For more specific code on reading and interpreting serial data, please look at the documentation for Save_Serial_Pi.py and 
QRS_Detection_RT.py. 


## Hardware Setup ## 
  To connect the Raspberry Pi 3 to the serial cable, please refer to the images at the following link: http://imgur.com/a/TFTnM
The GPIO pins are located on the back of the Pi. 

  For those of you who cannot view the link, here are the instructions: 
  'The serial cable connects to pins 1 (3.3V), 6 (GND), and 10 (RXD). 
The red wire connects to 3.3V (pin 1, very bottom left), the black wire connects to GND (pin 6, 3rd pin from the top left), 
and the yellow pin connects to RXD (pin 10, 5th pin from the top left).'


## Programs ## 
  This folder contains three programs: Save_Serial_Pi.py, QRS_Detection.py, and QRS_Detection_RT.py. 
  
  Save_Serial_Pi.py is a simple program that reads in serial data for a certain amount of time and stores the information in 6 
different text files. 
  
  QRS_Detection.py performs QRS complex detection on a text file of prerecorded single-vector data. It is the algorithm and code on which 
QRS_Detection_RT.py is based on. 

  QRS_Detection_RT.py is currently still in development, but will ideally perform the algorithm from QRS_Detection.py on serial data 
that is being read in real-time. This code is structured so that the algorithm itself is a class with an iteration function that 
gets called, so ideally this algorithm can easily be replaced with a more robust, multi-vectored one later on. 
