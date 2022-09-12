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
volatile int verticalLine = 0;  // runnig count

// controller pins
const int STUNNER_PIN_4 = 8;  // TH and used to set the signal for the light gun
const int STUNNER_PIN_6_TRIGGER = 9;  // TL is the light gun trigger
const int STUNNER_PIN_5_START = 10;  // TR is the light gun start

  
// offsets
volatile int minY = 0;
volatile int minX = 0; 

volatile short y = 260; //   40~ish near top. ~255 is the bottom
volatile short x = 100; //  1 is the left? seems odd.  185 is the right (disappears, need to look into faster uController)



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
    delayMicroseconds( 3 ); // arbitrary.
    digitalWrite( STUNNER_PIN_4, HIGH );
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
  
  //Serial.println(verticalLine);
  
  if (Serial.available()){
    char val = Serial.read(); // Read a character
    switch(val) {
      case 'w':
        y--;
        break;
      case 'a':
        x--;
        break;
      case 's':
        y++;
        break;
      case 'd':
        x++;
        break;
      case 'W':
        y -=10;
        break;
      case 'A':
        x -= 10;
        break;
      case 'S':
        y += 10;
        break;
      case 'D':
        x += 10;
        break;
      case 'q':    
        Serial.print("Start\n");
        digitalWrite(STUNNER_PIN_5_START, LOW); 
        delay(150);
        digitalWrite(STUNNER_PIN_5_START, HIGH);  
        break; 
      case 'e':
        Serial.println((String)"Trigger x:"+x+" y:"+y);
        digitalWrite(STUNNER_PIN_6_TRIGGER, LOW); 
        delay(150);
        digitalWrite(STUNNER_PIN_6_TRIGGER, HIGH);   
        break; 
      case 't':
        y = 40;
        break;
      case 'r':
        x = 183;
        break;
      case 'b':
        y = 250;
        break;
      case 'l':
        x = 1;
        break; 
      case 'm':
        x = 95;
        y = 155;
        break;
    }
  }
}
