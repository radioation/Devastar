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
const int SUPERSCOPE_PIN_2 = 8;  // External Latch on SNES port
const int IC4021_START_15 = 9;  // trigger goes to shifter chip pin 15
const int IC4021_TURBO_14 = 10;  // turbo goes to shifter chip pin 14
const int IC4021_PAUSE_13 = 11;  //  pause goes to shifter chip pin 13
const int IC4021_TRIGGER_1 = 12;  // start goes to shifter chip pin 12

  
// offsets
volatile int minY = 0;
volatile int minX = 0; 

volatile short y = 130; //   
volatile short x = 100; //   



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
  Serial.begin(9600);

  // Sync Splitter
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);
 
  // controller pins 
  pinMode(SUPERSCOPE_PIN_2, OUTPUT);
  pinMode(IC4021_TRIGGER_1, OUTPUT);
  pinMode(IC4021_TURBO_14, OUTPUT); 
  pinMode(IC4021_PAUSE_13, OUTPUT);
  pinMode(IC4021_START_15, OUTPUT); 
  digitalWrite(SUPERSCOPE_PIN_2, HIGH);
  digitalWrite(IC4021_TRIGGER_1, HIGH);
  digitalWrite(IC4021_TURBO_14, HIGH);
  digitalWrite(IC4021_PAUSE_13, HIGH);
  digitalWrite(IC4021_START_15, HIGH);

  // interrupts
  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
}

void loop() { 
  
  //Serial.println(verticalLine);
  
  if (Serial.available()){
    char val = Serial.read(); // Read a character
    switch(val) {
      case 'w':  // up
        y--;
        break;
      case 'a':  // left
        x--;
        break;
      case 's':  // down
        y++;
        break;
      case 'd':  // right
        x++;
        break;
      case 'W':   // up by ten
        y -=10;
        break;
      case 'A':    // left by ten
        x -= 10;  
        break;
      case 'S':   // down by ten
        y += 10;  
        break;
      case 'D':  // right by ten
        x += 10;
        break;
      case 'q':     // Trigger button
        Serial.println((String)"Trigger x:"+x+" y:"+y); 
        digitalWrite(IC4021_TRIGGER_1, LOW); 
        delay(150);
        digitalWrite(IC4021_TRIGGER_1, HIGH);  
        break; 
      case 'e':     // Start
        Serial.print("Start\n");
        digitalWrite(IC4021_START_15, LOW); 
        delay(150);
        digitalWrite(IC4021_START_15, HIGH);   
        break; 
      case 'z':     // Turbo
        Serial.print("Turbo\n");
        digitalWrite(IC4021_TURBO_14, LOW); 
        delay(150);
        digitalWrite(IC4021_TURBO_14, HIGH);  
        break; 
      case 'c':     // Pause
        Serial.print("Pause\n");
        digitalWrite(IC4021_PAUSE_13, LOW); 
        delay(150);
        digitalWrite(IC4021_PAUSE_13, HIGH);   
        break; 
      case 't':
        y = 40;  // move to the top
        break;
      case 'r':  // move to the right
        x = 183;
        break;
      case 'b':  // move to the bottom
        y = 250;
        break;
      case 'l':  // move to the left
        x = 1;
        break; 
      case 'm':  // move to the middle
        x = 100;
        y = 155;
        break;
      case 'o': // point offscreen 
        y = 100;
        x = 0;
        break; 
    }
  }
}
