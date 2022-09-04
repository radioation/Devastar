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
const int TH_PIN = 8;

const int B_PIN = 9;
const int A_PIN = 10;
const int C_PIN = 11;
const int S_PIN = 12;

// offsets
const int minY = 30;
const int minX = 70;

// input variables
byte y = 140;  // line count
byte x = 200;  // Simulate TH delay
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
  if ( verticalLine == minY + y && x != 255 ) {
    delayX4Cycles(minX + x);
    digitalWrite( TH_PIN, HIGH );
    delayMicroseconds( 5 ); // arbitrary.
    digitalWrite( TH_PIN, LOW );

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
  pinMode(TH_PIN, OUTPUT);
  digitalWrite(TH_PIN, LOW);

  pinMode(B_PIN, OUTPUT);
  digitalWrite(B_PIN, HIGH);
  pinMode(A_PIN, OUTPUT);
  digitalWrite(A_PIN, HIGH);
  pinMode(C_PIN, OUTPUT);
  digitalWrite(C_PIN, HIGH);
  pinMode(S_PIN, OUTPUT);
  digitalWrite(S_PIN, HIGH);

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
      digitalWrite(A_PIN, LOW);
    } else {
      digitalWrite(A_PIN, HIGH);
    }
    if ( buttons & 0x02 ) {
      digitalWrite(B_PIN, LOW);
    } else {
      digitalWrite(B_PIN, HIGH);
    }

    if ( buttons & 0x04 ) {
      digitalWrite(C_PIN, LOW);
    } else {
      digitalWrite(C_PIN, HIGH);
    }

    if ( buttons & 0x08 ) {
      digitalWrite(S_PIN, LOW);
    } else {
      digitalWrite(S_PIN, HIGH);
    }
  }

}
