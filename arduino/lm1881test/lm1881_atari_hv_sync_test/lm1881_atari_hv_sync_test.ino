// some of this code (delayX4Cycles() )  was derived from https://github.com/arduino
// SO it's LGPL 2.1
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


// sync variables
const int COMPOSITE_SYNC_1881 = 2;
const int VERTICAL_SYNC_1881 = 3;
volatile int verticalLine = 0;

// controller pins
const int ATARI_PIN_1 = 8;  // stick FWD is the light gun trigger
const int ATARI_PIN_6 = 9;  // stick trigger and used to set the signal for the light gun

  
// offsets
const int minY = 0;
const int minX = 0;

// input variables
// changed y and x to figure out edges.
// http://www.atarihq.com/atcomp/xegs.html
// MattRatt says :
//  Light gun values range from 0 to 227. 

// You'll notice with your test program that GUN-Y only varies from about 17 to 115. 
byte y = 230;  // line count ( 6  gives me about 0.  5 jumps to 130?. 
               // use 40-237:  40 gives me 17 and 237 gives me 115.  

//  horizontal readings are quite odd.  
// Point the gun to the far left of the display and 
// GUN-X will read about 88. Moving from left to right, 
// the reading will reach 227 at about column 34. Then 
// suddenly it drops to 0 and increases again to about 
// 30 at column 39.
byte x = 100;  // Simulate horizontal delay with delayX4Cycles() function
// x =5 gives me about 73, 20x ~= 87 Go with 21 as the low end of the range
// 100x ~= 159  , 160x ~=212, 175x seems to be the edge of it. I see it values like 1,2,225,226,227
// 180x ~= 2-5, 200 ~= 20  210 seems to be the one to use
// use 21-210:   ( 21 gives me about 88, 210 gives me approx 30 )


// modified delayMicroseconds to use the smallest possible wait
// to increase horizontal resolution
void delayX4Cycles(unsigned int c)
{
  if (--c == 0)
    return;

  // busy wait
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t" // 2 cycles
    "brne 1b" : "=w" (c) : "0" (c) // 2 cycles
  );
}

void compositeSyncInterrrupt() {
  verticalLine++;
  if ( verticalLine == minY + y ) { // && x != 255 ) {
    delayX4Cycles(minX + x);
    digitalWrite( ATARI_PIN_6, LOW );
    delayMicroseconds( 5 ); // arbitrary.
    digitalWrite( ATARI_PIN_6, HIGH );

  }
}

void verticalSyncInterrrupt() {
  verticalLine = 0;
}

void setup() {
  // serial communication
  Serial.begin(9600);

  // Sync Splitter
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);

  // controller pins 
  pinMode(ATARI_PIN_1, OUTPUT);
  pinMode(ATARI_PIN_6, OUTPUT); 
  digitalWrite(ATARI_PIN_1, HIGH);
  digitalWrite(ATARI_PIN_6, HIGH);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
}

void loop() { 
  // pull the trigger every once in awhile
  delay(2000);
  digitalWrite(ATARI_PIN_1, LOW);
  delay(500);
  digitalWrite(ATARI_PIN_1, HIGH);
}
