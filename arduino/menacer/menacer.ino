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
const int TH_PIN = 8;


const int minY = 30;
const int minX = 19;
int y = 140;  // line count
int x = 200;  // Simulate TH delay


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
    //delayMicroseconds( x );
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
  Serial.begin(9600);
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);

  pinMode(TH_PIN, OUTPUT);
  digitalWrite(TH_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
}

void loop() {
   if(Serial.available() > 1)
   {
      x = Serial.read();
      y = Serial.read();
   }
}
