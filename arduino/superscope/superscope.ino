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

#define USE_BLUETOOTH_SERIAL

#ifdef USE_BLUETOOTH_SERIAL
// Leonardo Bluetooth use real TX/TR
#define SERIAL_COM Serial1
#else
// Use USB 
#define SERIAL_COM Serial
#endif


// sync variables
const int COMPOSITE_SYNC_1881 = 2;
const int VERTICAL_SYNC_1881 = 3;
volatile int verticalLine = 0;
 
// controller pins
const int SUPERSCOPE_PIN_2 = 8;  // External Latch on SNES port
const int IC4021_START_15 = 9;  // trigger goes to shifter chip pin 15
const int IC4021_TURBO_14 = 10;  // turbo goes to shifter chip pin 14
const int IC4021_PAUSE_13 = 11;  //  pause goes to shifter chip pin 13
const int IC4021_TRIGGER_12 = 12;  // start goes to shifter chip pin 12

// offsets
const int minY = 40; //   40~ish near top. ~260 is the bottom   (range ~220)
const int minX = 1;  //  1 is the left? seems odd.  183 is near the right  but disappears past that range ~182)


// input variables
volatile byte y = 90;  // line count
volatile byte x = 100;  // Simulate TH delay
byte buttons = 0;

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
  // apparently we need to have more than one line
  if ( verticalLine >= minY + y && verticalLine < minY + y + 8 ) {  
    delayX4Cycles(minX + x);
    digitalWrite( SUPERSCOPE_PIN_2, LOW ); // TH active is 0
    delayMicroseconds( 4 ); // arbitrary.
    digitalWrite( SUPERSCOPE_PIN_2, HIGH );
  }
}


void verticalSyncInterrrupt() {
  verticalLine = 0;
}

void setup() {
  // serial communication
  SERIAL_COM.begin(9600);

  // Sync Splitter
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);

  // controller pins 
  pinMode(SUPERSCOPE_PIN_2, OUTPUT);
  pinMode(IC4021_TRIGGER_12, OUTPUT);
  pinMode(IC4021_TURBO_14, OUTPUT); 
  pinMode(IC4021_PAUSE_13, OUTPUT);
  pinMode(IC4021_START_15, OUTPUT); 
  digitalWrite(SUPERSCOPE_PIN_2, HIGH);
  digitalWrite(IC4021_TRIGGER_12, HIGH);
  digitalWrite(IC4021_TURBO_14, HIGH);
  digitalWrite(IC4021_PAUSE_13, HIGH);
  digitalWrite(IC4021_START_15, HIGH);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
}

void loop() {
  if (SERIAL_COM.available() > 2)
  {
    x = SERIAL_COM.read();
    y = SERIAL_COM.read();
    buttons = SERIAL_COM.read(); 
    // set buttons
    if ( buttons & 0x01 ) {
      digitalWrite(IC4021_TRIGGER_12, LOW);
    } else {
      digitalWrite(IC4021_TRIGGER_12, HIGH);
    }
    if ( buttons & 0x02 ) {
      digitalWrite(IC4021_PAUSE_13, LOW);
    } else {
      digitalWrite(IC4021_PAUSE_13, HIGH);
    }
    if ( buttons & 0x04 ) {
      digitalWrite(IC4021_PAUSE_13, LOW);
    } else {
      digitalWrite(IC4021_PAUSE_13, HIGH);
    }
    if ( buttons & 0x08 ) {
      digitalWrite(IC4021_START_15, LOW);
    } else {
      digitalWrite(IC4021_START_15, HIGH);
    }
  }
    if ( buttons & 0x10 ) {
      digitalWrite(IC4021_TURBO_14, LOW);
    } else {
      digitalWrite(IC4021_TURBO_14, HIGH);
    }

}
