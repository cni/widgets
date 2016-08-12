/*
 * CNI pulse-to-key
 *
 *
 * This sketch will listen for input pulses and send out a keypress whenever 
 * one is detected. It uses an external interrupt, so it can reliably detect 
 * very brief pulses. We use this to detect the GE 3.3v scope trigger pulses 
 * (~4 usec pulse duration).
 *
 *
 * Copyright 2015 Robert F. Dougherty.
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
 * 2015.08.20 RFD (bobd@stanford.edu) wrote it, borrowing code from trigger.
 * 
 */

#define VERSION "1.0"

#define DEFAULT_OUT_PULSE_MSEC 5
#define DEFAULT_IN_PULSE_STATE 0

// Most Arduino boards have two external interrupts:
// 0 (on digital pin 2) and 1 (on digital pin 3). On the
// Teensy, these pins aremight be different (check docs).
#define INTERRUPT 0

// The approximate duration of the output pulse, in milliseconds.
// Note that we don't account for delays in the code, so it will
// always be a bit longer than this.
unsigned int g_outPulseDuration = DEFAULT_OUT_PULSE_MSEC;

// The input pin should be the pin mapped to INT0, unless you change
// the interrupt number below. The output pin can be any digital pin.
#if defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11 and INT0 on pin 5 (D0)
  const byte g_outPin = 11;
  const byte g_inPin = 5;
#else
  #error "Unknown board!"
#endif

byte g_inPulseEdge = RISING;

volatile byte g_outPinOn = false;
volatile unsigned long g_outPulseStart;
volatile unsigned long g_digitalStart;

char g_pulse_key = '5';

void setup(){
  
  pinMode(g_inPin, INPUT);
  pinMode(g_outPin, OUTPUT);
  digitalWrite(g_outPin, LOW);
  
  // Enable input pulse detection
  // Turn on the internal pull-up resistor if we want to detect falling edges.
  if(g_inPulseEdge==FALLING)
    digitalWrite(g_inPin, HIGH);
  else
    digitalWrite(g_inPin, LOW);
  attachInterrupt(INTERRUPT, triggerIn, g_inPulseEdge);
  // detachInterrupt(INTERRUPT);
}

void loop(){
  // Reset the output, if needed:
  if(g_outPinOn){
    // Turn off the output pin after the requested duration.
    // Detect and correct counter wrap-around:
    if(millis()<g_outPulseStart) g_outPulseStart += 4294967295UL;
    if(millis()-g_outPulseStart > g_outPulseDuration)
      digitalWrite(g_outPin, LOW);
  }
}

// The following is an interrupt routine that is run each time
// a pulse is detected on the trigger input pin.
void triggerIn(){
  triggerOut();
  Keyboard.print(g_pulse_key);
}

void triggerOut(){
  // First force the pin low, in case it was already on. This will ensure that
  // we get a change on the pin no matter what state we were in.
  if(g_outPinOn) digitalWrite(g_outPin, LOW);
  digitalWrite(g_outPin, HIGH);
  g_outPinOn = true;
  g_outPulseStart = millis();
}

