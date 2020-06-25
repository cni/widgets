# Serial Trigger Box - HW Version 2 

This is the hardware (KiCad project) description as well as the new code to run on the hardware

## Building and programming

This project uses the Teensy ++2.0. Refer to the [PJRC](https://www.pjrc.com) site for description of how to download the Arduino IDE and Teensy loader. 

## Usage

The instructions on the [CNI Wiki](https://cni.stanford.edu/wiki/MR_Hardware#USB-to-Serial_Port_Trigger) describe the use of this hardware in conjunction with the GE MRI system.  Briefly, it provides a USB serial connection for a user computer to either detect the scope trigger pulse from the scanner or to send a TTL pulse to the EXT trigger input on the scanner. 

If connecting with a serial interface (such as the Arduinio serial monitor, or a Unix serial terminal program such as cu, putty, screen, etc.) to the serial trigger box, the following welcome screen is displayed. 

```Welcome Screen
*********************************************************
* CNI Trigger firmware version 3.0
* Copyright 2020 Adam Kerr<akerr@stanford.edu>
* http://cniweb.stanford.edu/wiki/CNI_widgets
*********************************************************

CNI Trigger Ready. Send the ? command ([?]) for help.
```

The help menu is as follows. 

```Help Menu
CNI Trigger Device
Sends TTL pulses out on pin 6.
Listens for pulses in on pin 1.

Commands:
[t]   will send a trigger pulse. This also disables the input pulse
      being sent over serial. Send a [p] command to re-enable it.

[o,N] set the trig output pulse duration to N milliseconds. Send with
      no argument to show the current pulse duration.
      Default duration is 5 ms.

[l,N] set the LED pulse durations to N milliseconds. Send with
      no argument to show the current pulse duration.
      Default duration is 250 ms.

[p]   Send a 'p' over the serial port when trigIn pulses detected. Send a [t] to disable.

[r]   reset default state.

[s,D] Send data (D) out over the local serial port. D can be up to 64
      bytes any binary data, except that it cannot contain the ASCII codes
      for '[', ']', or ','. The data are sent out at 57600 bps.

```
