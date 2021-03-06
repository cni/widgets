/*
 * CNI touch
 *
 * Sketch to read a 4-wire resitive touch screen and stream absolute position
 * coordinates via a serial port. Alternatively, you can emulate a USB mouse,
 * making it more like a large (single-touch) trackpad. (See "#define ABSOLUTE"
 * for changing it from absolute to mouse-like.)
 *
 * It's been developed for and tested with a Keytec touch screen and a teensy 2, 
 * but could be easily adapted to run on the more powerful teensy 3.1. 
 *
 * More details:
 *
 *   http://www.pjrc.com/store/teensy.html
 * Four-wire resistive touch glass from Keytec:
 *
 *   http://www.magictouch.com/KTT-084LAM.html
 * 
 * Note that we chose an 8.4 inch glass, but a smaller size might be better to 
 * reduce arm movements. It seems that the Keytec glass is now hard to get. This
 * alternative from Adafruit seems to work well:
 *
 *   https://www.adafruit.com/products/1676
 *
 * The circuit is trivial-- you just need to wire the touch screen to four of 
 * the microcontroller pins and a pushbutton to one of the pins and to ground. 
 * We laser-cut some acrylic sheets to build a nice case for it. But we started 
 * by just mounting it to a piece of cardboard and that worked fine!
 *
 * We ran the 6 wires from the touchscreen and pushbutton through an old 
 * ethernet cable (which has 8 wires; 2 extra if you want to add more buttons. 
 * The firmware will support up to 4). This was run to our penetration panel, 
 * where each wire connected to a pin on a db9 pen-panel RF filter. The 
 * microcontroller is connected to the other side of the filter, and thus 
 * outside the scan room.
 * 
 * See the CNI wiki page for a picture, as well as links to matlab and python 
 * code to read the touch values:
 *
 *   http://cni.stanford.edu/wiki/MR_Hardware#CNI_Touch-pad
 *
 * The pins used are #define configured below. For the teensy 2, we connected 
 * the four wires for the touch screen to pins 18-21 and a pushbutton to pin 0.
 *
 * Copyright 2011 Robert F. Dougherty (bobd@stanford.edu)
 */

#include "pins_arduino.h"

#include <stdarg.h>
#include <stdio.h>

// *** DEVICE MODE (mouse-emulating trackpad or absolute position via serial-port) ***
// ABSOLUTE mode delivers absolute position values on the serial port instead of 
// mouse movements. If you want it to behave like a simple track-pad that emulates 
// a mouse, comment-out the line below and change the teensy USB device type from
// 'serial' to 'keyboard/mouse' (under the Arduino 'tools' menu). 
//#define ABSOLUTE

// *** WIRING UP THE CONNECTIONS ***
// We will switch data-direction and digial values often and in time-critical code,
// so we'll use direct mapping to the data-direction register (DDR) and the digital
// value register (PORT).
// NOTE: if you chane the pins that you use, change these values appropriately!
#if defined(__AVR_ATmega32U4__)
  // Teensy2.0
  #define TOUCH_PORTREG CORE_PIN21_PORTREG
  #define TOUCH_DDRREG  CORE_PIN21_DDRREG
  #define TOUCH_YPOS CORE_PIN18_BITMASK
  #define TOUCH_XNEG CORE_PIN19_BITMASK
  #define TOUCH_YNEG CORE_PIN20_BITMASK
  #define TOUCH_XPOS CORE_PIN21_BITMASK
  #define TOUCH_YNEG_PIN 20
  #define TOUCH_XNEG_PIN 19
  // We'll read the four inputs on port B (pins 0-3)
  #define BUTTON_PINREG CORE_PIN0_PINREG
  static const byte c_buttonMask[] = {CORE_PIN0_BITMASK,CORE_PIN1_BITMASK,CORE_PIN2_BITMASK,CORE_PIN3_BITMASK};
  #define BUTTON_1_PIN 0
  #define BUTTON_2_PIN 1
  #define BUTTON_3_PIN 2
  #define BUTTON_4_PIN 3
  // LED will be on when a touch is detected
  #define LED 11
#else
  // Teensy3.1
  #define byte uint8_t
  #define TOUCH_PORTREG CORE_PIN5_PORTREG
  #define TOUCH_DDRREG  CORE_PIN5_DDRREG
  #define TOUCH_YPOS digitalPinToBitMask(16)
  #define TOUCH_XNEG digitalPinToBitMask(17)
  #define TOUCH_YNEG digitalPinToBitMask(18)
  #define TOUCH_XPOS digitalPinToBitMask(19)
  #define TOUCH_YNEG_PIN 18
  #define TOUCH_XNEG_PIN 17
  // We'll read the four inputs on port B (pins 0-3)
  #define BUTTON_PINREG CORE_PIN5_PINREG
  static const byte c_buttonMask[] = {digitalPinToBitMask(5),digitalPinToBitMask(6),digitalPinToBitMask(7),digitalPinToBitMask(8)};
  #define BUTTON_1_PIN 5
  #define BUTTON_2_PIN 6
  #define BUTTON_3_PIN 7
  #define BUTTON_4_PIN 8
  // LED will be on when a touch is detected
  #define LED 13
#endif

#define BUTTON_DEBOUNCE_TIME 25
#define BUTTON_LOCK_INPUT_TIME 250

// The touch readings are buffered, and some values at the beginning and end of a touch
// epoch are discarded. These must be powers of two, and no more than 8 bits. A larger
// buffer makes for smoother movements, but reduces responsiveness. The values below
// worked well for us, with a good balance between smoothness and responsiveness.
#define BUFF_BITS 5 //5
#define BUFF_SIZE (1<<BUFF_BITS)
#define KEEP_BITS 4 //4
#define KEEP_SIZE (1<<KEEP_BITS)
#define DISCARD_SIZE (BUFF_SIZE-KEEP_SIZE)

// Should we allow tap-to-click? This is working, but isn't very smooth.
// So we kept this off and used a separate button for clicking.
byte g_doClick = false;

// The gain is an integer from 1 to ~30. There's some bit-shifting going on, 
// so g_gain=8 is actualy unity, g_gain=12 is an effective gain of 1.5, etc. 
int g_gain = 8;
int g_upper_move_thresh = g_gain<<3 * 1000;

// The threshold for detecting a touch. This is generally about half the applied potential
// (in 10-bit ADC units: 512). But we found that a slightly lower value elimated some junk
// introduced by lightly grazing the touch surface.
int g_touch_thres = 400;

byte g_accel = true;

// Milliseconds before we start auto-repeating a keystroke.
unsigned int g_keyAutoRepeatDelay = 250;
byte g_reverse_x = false;
byte g_reverse_y = false;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
#ifdef ABSOLUTE
  Serial.begin(57600);
#endif
}

// Since this controller is dedicated to reading the touch panel and buttons, we use a
// simple polling loop. Another option is have an interrupt-driven approach. But that
// would be overkill here.
void loop()
{
  static byte numDiscarded;
  static byte touching;
  static byte tracking;
  static int buffX[BUFF_SIZE];
  static int buffY[BUFF_SIZE];
  static long sumX, sumY;
  static unsigned int curBuffIndex;
  static int lastPosX, lastPosY;
  static unsigned long lastTouchedStart;
  static unsigned long lastTouchedEnd;
  static unsigned long lastKeyPress[2];
  byte curReadIndex;
  int posX, posY;
  byte buttonState;
  unsigned long curMillis;
  int meanPosX;
  int meanPosY;

  // Get the de-bounced state of the four buttons.
  // We currently use the third button to emulate a left-click and the fourth the emulate a right-click.
  buttonState = readButtonState();
  curMillis = millis();

  #ifndef ABSOLUTE
  Mouse.set_buttons(buttonState&c_buttonMask[2] | buttonState&c_buttonMask[1], 0, buttonState&c_buttonMask[3]);
  // To emulate scroll wheel: Mouse.scroll(val): val=[-127,+127] (+ to scroll up, - to scroll down)
  // The other two buttons send keystrokes.
  if(buttonState&c_buttonMask[0] && curMillis-lastKeyPress[0]>g_keyAutoRepeatDelay){
    Keyboard.print('1');
    lastKeyPress[0] = curMillis;
  }
  #endif

  // Read and process the touchpad.
  if(!readTouchPad(&posX, &posY)){
    // Not currently being touched.
    // Check for a double-tap event.
    if(g_doClick && touching){
      static unsigned long lastTapped;
      // If 'touching' was true, then we *were* just being touched; we only get here if
      // a touch epoch just ended. Check to see if that previous touch was a quick 'tap'.
      lastTouchedEnd = millis();
      if(lastTouchedEnd-lastTouchedStart < 200){
        // the last touch looked like a tap, so if the previous tap that we detected wasn't
        // too long ago, we'll fire a double-tap event (left-click). Otherwise, we'll just
        // record the time of this tap.
        if(lastTouchedEnd-lastTapped<500){
          #ifndef ABSOLUTE
          Mouse.click();
          #endif
        }else{
          lastTapped = lastTouchedEnd;
        }
      }
    }
    // Reset the state vars to the no-touch condition
    digitalWrite(LED, LOW);
    touching = false;
    tracking = false;
    sumX = 0;
    sumY = 0;
    curBuffIndex = 0;
    numDiscarded = 0;
  }else{
    // We were touched!
    touching = true;
    if(numDiscarded<DISCARD_SIZE){
      // touch just started- let the loop iterate DISCARD_SIZE times to discard the initial readings
      if(numDiscarded == 0){
        // The very first loop interation since a new touch started; record the time and turn on the LED.
        lastTouchedStart = millis();
        digitalWrite(LED, HIGH);
      }
      numDiscarded++;
    }else if(!tracking){
      // discard is finished, but we're not tracking yet- that means that we're filling the buffer.
      buffX[curBuffIndex] = posX;
      buffY[curBuffIndex] = posY;
      if(curBuffIndex >= DISCARD_SIZE){
        curReadIndex = curBuffIndex-DISCARD_SIZE;
        sumX += buffX[curReadIndex];
        sumY += buffY[curReadIndex];
      }
      curBuffIndex++;
      if(curBuffIndex==BUFF_SIZE){
        lastPosX = sumX>>KEEP_BITS;
        lastPosY = sumY>>KEEP_BITS;
        tracking = true;
      }
    }else{
      // Now we are tracking.
      // Remove the oldest value from the sum:
      byte oldIndex = curBuffIndex%BUFF_SIZE;
      sumX -= buffX[oldIndex];
      sumY -= buffY[oldIndex];
      // And store the current (newest) value in that buffer position:
      buffX[oldIndex] = posX;
      buffY[oldIndex] = posY;
      // And update the sum with the newest value:
      curReadIndex = (curBuffIndex+BUFF_SIZE-DISCARD_SIZE)%BUFF_SIZE;
      sumX += buffX[curReadIndex];
      sumY += buffY[curReadIndex];
      // We let this counter overflow. As long as everything is a power of 2, we're good.
      curBuffIndex++;
      // Now compute the mean using a bit-shift to do integer division:
      meanPosX = sumX>>KEEP_BITS;
      meanPosY = sumY>>KEEP_BITS;
      // Integer math- g_gain=8 is actually unity gain due to the 3-bit shift to the right here (i.e., /8).
      int moveX =  ((meanPosX-lastPosX) * g_gain)>>3;
      int moveY = -((meanPosY-lastPosY) * g_gain)>>3;
      if(abs(moveX)>g_upper_move_thresh || abs(moveY)>g_upper_move_thresh){
        // Values higher than the threshold are almost always noise junk, so don't count them.
        moveX = 0;
        moveY = 0;
      }
      if(g_accel){
        // Implement mouse velocity acceleration
        int accel = isqrt(moveX*moveX + moveY*moveY);
        if(accel>1){
          moveX *= accel;
          moveY *= accel;
        }
      }
      #ifndef ABSOLUTE
        // Mouse move values range from -127 to +127; +X moves right, +Y moves down.
        Mouse.move(moveX, moveY);
      #endif
      //}
      lastPosX = meanPosX;
      lastPosY = meanPosY;
    }
  }
  #ifdef ABSOLUTE
  static unsigned long prevMillis;
  if(curMillis-prevMillis>=10 && tracking){
    if(meanPosX>999) meanPosX = 999;
    if(meanPosY>999) meanPosY = 999;
    char tmp[20]; // resulting string limited to 20 chars
    snprintf(tmp, 20, "X=%03d;Y=%03d;B=%03d;\n", meanPosX, meanPosY, buttonState);
    Serial.print(tmp);
    prevMillis = curMillis;
  }
  #endif
}

// The following quickly reads (via direct register access) the state of our buttons
// and implements a simple debounce filter, so the state of a button will only change
// after the pin state has changed for some time (BUTTON_DEBOUNCE_TIME).
byte readButtonState()
{
  static byte curButtonState;
  static byte pendingStateChange;
  static unsigned long bMillis[4];
  unsigned long curMillis = millis();

  // Read the state of the buttons
  byte rawButtonState = BUTTON_PINREG;
  // Use "not xor" to find bits that are different
  byte stateChange = ~(rawButtonState ^ curButtonState);

  for(byte i=0; i<4; i++){
    if(stateChange & c_buttonMask[i]){
      // Button i's state has changed, but we won't let it go through
      // unless it remains changed for longer than the debounce time.
      if(pendingStateChange & c_buttonMask[i]){
        // there was a pending state change for button i. Has debounce time expired?
        if(curMillis-bMillis[i]>BUTTON_DEBOUNCE_TIME){
          // debounce time expired, so toggle the state for this button and clear its pending flag.
          curButtonState ^= c_buttonMask[i];
          pendingStateChange &= ~c_buttonMask[i];
        }
      }else{
        // It wasn't pending, so mark it as so and set the debounce timer.
        pendingStateChange |= c_buttonMask[i];
        bMillis[i] = curMillis;
      }
    }
  }
  return(curButtonState);
}

// Read the 4-wire resistive touchpad. We use direct register access to do the pin-mode
// switching quickly. This routine is called often and its speed affects the responsiveness
// of the interface, so we need to be as efficient as possible here.
// Returns TRUE if touched, and puts the coords in posX and posY.
byte readTouchPad(int* posX, int* posY)
{
  // In standby mode YNEG is pulled up, so we know that we are touched if YNEG goes low.
  if(analogRead(TOUCH_YNEG_PIN)<g_touch_thres){
    // Horizontal read: pull one X high and the other low to create a horizontal voltage
    // gradient, and then read out the voltage from one of the Y pads.
    TOUCH_DDRREG &= ~(TOUCH_YPOS | TOUCH_YNEG); // Set YPOS and YNEG as inputs
    TOUCH_DDRREG |=  (TOUCH_XPOS | TOUCH_XNEG); // Set XPOS and XNEG as outputs
    TOUCH_PORTREG &= ~(TOUCH_YPOS | TOUCH_YNEG | TOUCH_XPOS); // Pull these 3 low
    TOUCH_PORTREG |=  (TOUCH_XNEG); // Pull XNEG high (try pulling YPOS high too?)
    // wait a bit for things to settle, then read from YNEG
    delayMicroseconds(500);
    *posX = analogRead(TOUCH_YNEG_PIN);

    // Vertical read: pull one Y high and the other low to create a vertical voltage
    // gradient, and then read out the voltage from one of the X pads.
    TOUCH_DDRREG &= ~(TOUCH_XPOS | TOUCH_XNEG); // Set XPOS and XNEG as inputs
    TOUCH_DDRREG |=  (TOUCH_YPOS | TOUCH_YNEG); // Set YPOS and YNEG as outputs
    TOUCH_PORTREG &= ~(TOUCH_XPOS | TOUCH_XNEG | TOUCH_YPOS); // Pull these 3 low
    TOUCH_PORTREG |=  (TOUCH_YNEG); // Pull YNEG high  (try pulling XPOS high too?)
    // wait a bit for things to settle, then read from XNEG
    delayMicroseconds(500);
    *posY = analogRead(TOUCH_XNEG_PIN);

    // Go back to standby mode
    TOUCH_DDRREG &= ~(TOUCH_YPOS | TOUCH_XNEG | TOUCH_YNEG);
    TOUCH_DDRREG |=  (TOUCH_XPOS);
    TOUCH_PORTREG &= ~(TOUCH_XPOS | TOUCH_XNEG | TOUCH_YPOS);
    TOUCH_PORTREG |=  TOUCH_YNEG;
    if(g_reverse_x) *posX = -*posX;
    if(g_reverse_y) *posY = -*posY;
    return(true);
  }else{
    return(false);
  }
}

/*
 * Integer square-root approximation, by Jim Ulery.
 * from http://www.azillionmonkeys.com/qed/sqroot.html
 */
unsigned int isqrt(unsigned long val) {
  unsigned long temp, g=0, b = 0x8000, bshft = 15;
  do {
    if(val >= (temp = (((g << 1) + b)<<bshft--))) {
      g += b;
      val -= temp;
    }
  } while(b >>= 1);
  return g;
}


/* I found the signals to be good enough for use as a trackpad without calibration. But if you did
 * want to calibrate:
 * Measure lower left (ll), lower right (lr), upper left (ul), upper right (ur) and middle (md).
 * Then,
 *    m = [X_ll, X_lr, X_ul, X_ur, X_md; Y_ll, Y_lr, Y_ul, Y_ur, Y_md];
 *    cal = [[0,1,0,1,.5; 0,0,1,1,.5];[1 1 1 1 1]] * pinv([m;[1 1 1 1 1]])
 * to use the cal matrix:
 *    relativeLocation = cal*[Xd;Yd;1]
 *    Xr = cal(0,0)*Xd+cal(1,0)*Yd+cal(2,0)
 *    Yr = cal(0,1)*Xd+cal(1,1)*Yd+cal(2,1)
 * e.g.,
 *    m = [440,50,600,120,360; 92,40,480,280,230];
 *    cal = round([[0,10000,0,10000,5000; 0,0,10000,10000,5000];[1 1 1 1 1]] * pinv([m;[1 1 1 1 1]]))
 *    cal = [-25    9   10829
 *           -10   34     498
 *             0    0       1]
 *
 * void updateCalibrationCoeffs(int Xt1, int Yt1, int Xt2, int Yt2, int Xt3, int Yt3){
 *   int Xd1 = 900; int Xd2 = 500; int Xd3 = 100;
 *   int Yd1 = 500; int Yd2 = 900; int Yd3 = 100;
 *   // See the AVR341 datasheet, e.g. http://www.adafruit.com/datasheets/AVR341.pdf
 *   g_coeff[0] = (long)(Xd1*(Yt2-Yt3) + Xd2*(Yt3-Yt1) + Xd3*(Yt1-Yt2)) / (Xt1*(Yt2-Yt3) + Xt2*(Yt3-Yt1) + Xt3*(Yt1-Yt2));
 *   g_coeff[1] = (long)(g_coeff[0]*(Xt3-Xt2) + Xd2 - Xd3) / (Yt2-Yt3);
 *   g_coeff[2] = (long)Xd3 - g_coeff[0]*Xt3 - g_coeff[1]*Yt3;
 *   g_coeff[3] = (long)(Yd1*(Yt2-Yt3) + Yd2*(Yt3-Yt1) + Yd3*(Yt1-Yt2)) / (Xt1*(Yt2-Yt3) + Xt2*(Yt3-Yt1) + Xt3*(Yt1-Yt2));
 *   g_coeff[4] = (long)(g_coeff[3]*(Xt3-Xt2) + Yd2 - Yd3) / (Yt2-Yt3);
 *   g_coeff[5] = (long)Yd3 - g_coeff[3]*Xt3 - g_coeff[4]*Yt3;
 * }
 *
 * void getCoords(int rawTouchX, int rawTouchY, int *touchX, int *touchY){
 *   *touchX = (int)(g_coeff[0]*rawTouchX + g_coeff[1]*rawTouchY + g_coeff[2]);
 *   *touchY = (int)(g_coeff[3]*rawTouchX + g_coeff[4]*rawTouchY + g_coeff[5]);
 * }
 */


