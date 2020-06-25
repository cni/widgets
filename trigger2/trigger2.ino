/*
 * CNI trigger sketch for Arduino.
 *
 * A simple sketch to generate TTl pulses when told to do so through a serial
 * port command.
 *
 * When running this sketch on a Teensy, be sure to set the "USB Type" to
 * "Serial" (under the Arduino Tools menu).
 *
 * This sketch will also listen for input pulses and send out a serial port
 * character whenever one is detected. It uses an external interrupt, so it
 * can reliably detect very brief pulses. We use this to detect the GE 3.3v
 * scope trigger pulses (~2 usec pulse duration).  It will also flash a trigger 
 * detect LED.
 *
 * Copyright 2020 Robert F. Dougherty and Adam B. Kerr
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You might have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * HISTORY:
 * 2011.03.28 RFD (bobd@stanford.edu) finished a working version.
 * 2011.05.24 RFD: cleaned up source & comments, removed down-counter code,
 *                 and added a serial message upon every input pulse that's
                   detected.
 * 2013.05.28 RFD: added 8 more triggerable outputs. (See the help text
 *                 below for usage info.)
 * 2020.06.17 ABK: Fixed a problem with the output pulse duration code, 
 *                 Cleaned up some time calculations. 
 * 2020.06.17 ABK: Major rewrite to work with new hardware that includes
 *                 a trigger detect LED that is pulsed every time a trigger
 *                 pulse is seen. 
 */

#define VERSION "3.0"

// Flash library is available from http://arduiniana.org/libraries/Flash/
#include <Flash.h>
#include <Messenger.h>

#define BAUD 57600

#define DEFAULT_TRIG_OUT_PULSE_MSEC 5
#define DEFAULT_LED_PULSE_MSEC 250
#define DEFAULT_ECHO_TRIG_IN_STATE 0

// The approximate duration of the output pulse, in milliseconds.
// Note that we don't account for delays in the code, so it will
// always be a bit longer than this.
unsigned int g_trigOutDur = DEFAULT_TRIG_OUT_PULSE_MSEC;
unsigned int g_ledDur = DEFAULT_LED_PULSE_MSEC;

// The input pin should be the pin mapped to INT0, unless you change
// the interrupt number below. The output pin can be any digital pin.
#if defined(__AVR_AT90USB1286__)
  // Teensy2.0++ has LED on D6 and INT0 on pin 0 (D0)
  const byte g_buttonPin = 0;
  const byte g_trigInPin = 1;
  const byte g_detectLedPin = 4;
  const byte g_trigOutLedPin = 5;
  const byte g_trigOutPin = 6;
#else
  #error "Unknown board!"
#endif

HardwareSerial g_uart = HardwareSerial();

byte g_buttonInPulseEdge = RISING;
byte g_trigInPulseEdge = RISING;

volatile byte g_echoTrigInPulseSerial = false;

volatile byte g_trigOutPinOn = false;
volatile unsigned long g_trigOutStart;

volatile byte g_detectLedPinOn = false;
volatile unsigned long g_detectStart;

// Instantiate Messenger object used for serial port communication.
Messenger g_message = Messenger(',','[',']');

// Create the Message callback function. This function is called whenever a complete
// message is received on the serial port.
void messageReady() {
  int val[10];
  byte i,j;
  char serialBuffer[64];
  if(g_message.available()) {
    // get the command byte
    char command = g_message.readChar();
    switch(command) {

    case '?': // display help text
      Serial << F("CNI Trigger Device\n");
      Serial << F("Sends TTL pulses out on pin ") << (int)g_trigOutPin << F(".\n");
      Serial << F("Listens for pulses in on pin ") << (int)g_trigInPin << F(".\n");
      Serial << F("\nCommands:\n");
      Serial << F("[t]   will send a trigger pulse. This also disables the input pulse\n");
      Serial << F("      being sent over serial. Send a [p] command to re-enable it.\n\n");
      Serial << F("[o,N] set the trig output pulse duration to N milliseconds. Send with\n");
      Serial << F("      no argument to show the current pulse duration.\n");
      Serial << F("      Default duration is ") << DEFAULT_TRIG_OUT_PULSE_MSEC << F(" ms.\n\n");
      Serial << F("[l,N] set the LED pulse durations to N milliseconds. Send with\n");
      Serial << F("      no argument to show the current pulse duration.\n");
      Serial << F("      Default duration is ") << DEFAULT_LED_PULSE_MSEC << F(" ms.\n\n");
      Serial << F("[p]   Send a 'p' over the serial port when trigIn pulses detected. Send a [t] to disable.\n\n");
      Serial << F("[r]   reset default state.\n\n");
      Serial << F("[s,D] Send data (D) out over the local serial port. D can be up to 64\n");
      Serial << F("      bytes any binary data, except that it cannot contain the ASCII codes\n");
      Serial << F("      for '[', ']', or ','. The data are sent out at ") << BAUD << F(" bps.\n\n");
      break;

    case 'o': // Set out-pulse duration (msec)
      i = 0; 
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i>1){
        Serial << F("ERROR: Set output pulse duration requires no more than one param.\n");
      }else if(i==1){
        g_trigOutDur = val[0];
      }
      Serial << F("Output pulse duration is set to ") << g_trigOutDur << F(" msec.\n");
      break;

    case 'l': // Set detect-pulse duration (msec)
      i = 0; 
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i>1){
        Serial << F("ERROR: Set detect pulse duration requires no more than one param.\n");
      }else if(i==1){
          g_ledDur = val[0];
      }
      Serial << F("LED pulse durations are set to ") << g_ledDur << F(" msec.\n");
      break;

    case 't': // force output trigger
      triggerOut();
      // Automatically disable input pulse detection
      Serial << F("Disabling echo of input pulses\n");
	    setEchoTrigInState(false);
      break;

    case 'p': // turn on input pulse detection
      Serial << F("Enabling echo of input pulses\n");
      setEchoTrigInState(true);
      break;

    case 'r': // reset
	    setEchoTrigInState(false);
	  
	    g_ledDur = DEFAULT_LED_PULSE_MSEC;

      digitalWrite(g_trigOutPin, LOW);
      digitalWrite(g_trigOutLedPin, LOW);
	    g_trigOutPinOn = false; 
      g_trigOutDur = DEFAULT_TRIG_OUT_PULSE_MSEC;
	    g_buttonInPulseEdge = RISING;
	    attachInterrupt(digitalPinToInterrupt(g_buttonPin),triggerOut,g_buttonInPulseEdge);

      digitalWrite(g_detectLedPin, LOW);
	    g_detectLedPinOn = false; 
	    g_trigInPulseEdge = RISING;
	    attachInterrupt(digitalPinToInterrupt(g_trigInPin),triggerDetect,g_trigInPulseEdge);

      flashLights();

      break;

    case 's': // stream serial data
      j = 0;
      if(g_message.available()){
        j = g_message.copyString(serialBuffer, 64);
        g_uart.write((byte*)serialBuffer,j);
      }else{
        Serial << F("WARNING: empty string.\n");
      }
      break;

    default:
      Serial << F("[") << command << F("]\n");
      Serial << F("ERROR: Unknown command.\n\n");
    } // end switch
  } // end while
}

void setup()
{
  Serial.begin(BAUD);
  Serial << F("\n\n");
  Serial << F("*********************************************************\n");
  Serial << F("* CNI Trigger firmware version ") << VERSION << F("\n");
  Serial << F("* Copyright 2020 Adam Kerr<akerr@stanford.edu>\n");
  Serial << F("* http://cniweb.stanford.edu/wiki/CNI_widgets\n");
  Serial << F("*********************************************************\n\n");

  // This probably isn't necessary- external interrupts work even in OUTPUT mode.
  if (g_buttonInPulseEdge == FALLING)
	  pinMode(g_buttonPin, INPUT_PULLUP);
  else
	  pinMode(g_buttonPin, INPUT);

  if (g_trigInPulseEdge == FALLING)
	  pinMode(g_trigInPin, INPUT_PULLUP);
  else
	  pinMode(g_trigInPin, INPUT);

  g_ledDur = DEFAULT_LED_PULSE_MSEC;

  pinMode(g_trigOutPin, OUTPUT);
  pinMode(g_trigOutLedPin, OUTPUT);
  digitalWrite(g_trigOutPin, LOW);
  digitalWrite(g_trigOutLedPin, LOW);
  g_trigOutPinOn = false; 
  g_trigOutDur = DEFAULT_TRIG_OUT_PULSE_MSEC;
  g_buttonInPulseEdge = RISING;
  attachInterrupt(digitalPinToInterrupt(g_buttonPin),triggerOut,g_buttonInPulseEdge);

  pinMode(g_detectLedPin, OUTPUT);
  digitalWrite(g_detectLedPin, LOW);
  g_detectLedPinOn = false; 
  g_trigInPulseEdge = RISING;
  attachInterrupt(digitalPinToInterrupt(g_trigInPin),triggerDetect,g_trigInPulseEdge);
  
  // Disable input pulse echo to serial
  setEchoTrigInState(false);

  // Attach the callback function to the Messenger
  g_message.attach(messageReady);

  // Set up the UART to support streaming serial data through.
  g_uart.begin(BAUD);

  flashLights(); 
  
  Serial << F("CNI Trigger Ready. Send the ? command ([?]) for help.\n\n");
}

void loop(){
  // Reset the output, if needed:
  if(g_trigOutPinOn){
    // Turn off the output pin after the requested duration.
    if(millis()-g_trigOutStart > g_trigOutDur) { 
      digitalWrite(g_trigOutPin, LOW);
	}
	if(millis()-g_trigOutStart > g_ledDur) { 
	    g_trigOutPinOn = false; 
		digitalWrite(g_trigOutLedPin, LOW);
	    attachInterrupt(digitalPinToInterrupt(g_buttonPin),triggerOut,g_buttonInPulseEdge);
	}
  }

  if(g_detectLedPinOn){
    // Turn off the output pin after the requested duration.
    if(millis()-g_detectStart > g_ledDur) { 
      digitalWrite(g_detectLedPin, LOW);
	    g_detectLedPinOn = false; 
	  }
  }

  // Handle Messenger's callback:
  if(Serial.available())  g_message.process(Serial.read()); 
}

void setEchoTrigInState(byte state){
	g_echoTrigInPulseSerial = state; 
}

// The following is an interrupt routine that is run each time
// a pulse is detected on the trigger input pin.
void triggerDetect(){
  // Turn on detect LED, disable interrupts until output pulse completed 
  digitalWrite(g_detectLedPin, LOW);
  g_detectStart = millis();
  digitalWrite(g_detectLedPin, HIGH);
  g_detectLedPinOn = true;

  // Echo 'p' to serial line if desired
  if(g_echoTrigInPulseSerial)
    Serial << F("p");
}

void triggerOut(){
  // Turn on trigOut LED, disable interrupts until output pulse completed 
  digitalWrite(g_trigOutPin, LOW);
  g_trigOutStart = millis();
  digitalWrite(g_trigOutPin, HIGH);
  digitalWrite(g_trigOutLedPin, HIGH);
  g_trigOutPinOn = true;
  detachInterrupt(digitalPinToInterrupt(g_buttonPin));
  // Serial << F("t");
}

void flashLights(){
  int i; 

  // Pulse each pin on in order then off
  digitalWrite(g_detectLedPin, HIGH);
  delay(500);
  digitalWrite(g_trigOutLedPin, HIGH);
  delay(500);
  digitalWrite(g_detectLedPin, LOW);
  delay(500);
  digitalWrite(g_trigOutLedPin, LOW);
  delay(500);

  // Flash both LED rapidly
  for (i = 0; i<3; i++) {
    digitalWrite(g_detectLedPin, HIGH);
	  digitalWrite(g_trigOutLedPin, HIGH);
	  delay(200);
	  digitalWrite(g_detectLedPin, LOW);
	  digitalWrite(g_trigOutLedPin, LOW);
	  delay(200);
  }
}
