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
const int STUNNER_PIN_4 = 8;  // TH and used to set the signal for the light gun
const int STUNNER_PIN_6_TRIGGER = 9;  // TL is the light gun trigger
const int STUNNER_PIN_5_START = 10;  // TR is the light gun start

// offsets
const int minY = 40; //   40~ish near top. ~253 is near the bottom   (range ~213)
const int minX = 1;  //  1 is the left? seems odd.  180 is near the right  but disappears past that range ~181)


// input variables
volatile byte y = 120;  // line count
volatile byte x = 90;  // Simulate TH delay
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
    digitalWrite( STUNNER_PIN_4, LOW ); // TH active is 0
    delayMicroseconds( 4 ); // arbitrary.
    digitalWrite( STUNNER_PIN_4, HIGH );
  }
}


void verticalSyncInterrrupt() {
  verticalLine = 0;
}

void setup() {
  // serial communication
  SERIAL_COM.begin(9600);

  //Serial.begin(9600);

  // Sync Splitter
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);

  // controller pins 
  pinMode(STUNNER_PIN_4, OUTPUT);
  pinMode(STUNNER_PIN_5_START, OUTPUT);
  pinMode(STUNNER_PIN_6_TRIGGER, OUTPUT); 
  digitalWrite(STUNNER_PIN_4, HIGH);
  digitalWrite(STUNNER_PIN_5_START, HIGH);
  digitalWrite(STUNNER_PIN_6_TRIGGER, HIGH);

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
      digitalWrite(STUNNER_PIN_6_TRIGGER, LOW);
    } else {
      digitalWrite(STUNNER_PIN_6_TRIGGER, HIGH);
    }
    if ( buttons & 0x08 ) {
      digitalWrite(STUNNER_PIN_5_START, LOW);
    } else {
      digitalWrite(STUNNER_PIN_5_START, HIGH);
    }
  }

}
