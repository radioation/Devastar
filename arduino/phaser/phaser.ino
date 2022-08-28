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
const int PHASER_PIN_7 = 8;  // TH and used to set the signal for the light gun
const int PHASER_PIN_6 = 9;  // TL is the light gun trigger

// offsets
const int minY = 24; // 24 appears to be the top of the screen, 247 the bottom  ( 247 -24  = 223  range )
const int minX = 15; // 15 appears to give me about 30, 193 gets me to about 180  ( 193-15 = 178 range )


// input variables

byte y = 112;  // line count
byte x = 89;   // Simulate TH delay
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
  if ( verticalLine == minY + y ) { // && x != 255 ) {
    delayX4Cycles(minX + x);
    digitalWrite( PHASER_PIN_7, LOW );
    delayMicroseconds( 5 ); // arbitrary.
    digitalWrite( PHASER_PIN_7, HIGH );
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
  pinMode(PHASER_PIN_7, OUTPUT);
  pinMode(PHASER_PIN_6, OUTPUT); 
  digitalWrite(PHASER_PIN_7, HIGH);
  digitalWrite(PHASER_PIN_6, HIGH);

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
    if ( buttons & 0x02 ) {
      digitalWrite(PHASER_PIN_6, LOW);  // phaser pins are active low https://allpinouts.org/pinouts/connectors/input_device/sega-master-system-light-phaser-3050/
    } else {
      digitalWrite(PHASER_PIN_6, HIGH);
    }

  }

}
