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
const int TL_PIN = 7;
const int TH_PIN = 8;    // PB4

const int B_PIN = 9;     // PB5
const int A_PIN = 10;    // PB6
const int C_PIN = 11;    // PB7
const int S_PIN = 12;    // PD6
  
// offsets
const int minY = 0;
const int minX = 0;

// input variables
// changed y and x to figure out edges.
volatile byte y = 170;  // line count ( 30 to 250 seems to be the Y range )
volatile short x = 170;  // Simulate TH delay ( seems to be 73 to 263 ) with delayX4Cycles() function )
volatile byte buttons = 0;


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

void TLInterrupt() {
  int tlState = digitalRead(TL_PIN);

  if ( tlState == LOW ) { 
    // faling edge, so deactivate all of the buttons, but leave TH ( PB0 ) alone
    // PB5 - B
    // PB6 - A/Trigger
    // PB7 - C
    // PD6 - Start
    PORTB = PORTB & B00011111; //  
    PORTD = PORTD & B10111111;

  } else { 
    // rising edge, set the buttons as needed.
    byte pb = PORTB;
    // TODO: May make more sense to set PB* bits in Raspberry PI properly.
    if ( buttons & 0x01 ) {
      // B
      pb = pb | B00100000;
    }
    if ( buttons & 0x02 ) {
      // A - trigger
      pb = pb | B01000000;
    }
    if ( buttons & 0x04 ) {
      // C
      pb = pb| B10000000;
    }
    if ( buttons & 0x08) {
      // S 
      PORTD = PORTD | B01000000;
    }
    PORTB = PORTB | pb; // port b
  }
}


void setup() {
  // serial communication
  Serial.begin(9600);

  // Sync Splitter
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);
  // TL line from Sega Genesis / MegaDrive
  pinMode(TL_PIN, INPUT);

  // controller pins
  pinMode(TH_PIN, OUTPUT);
  digitalWrite(TH_PIN, LOW);

  pinMode(B_PIN, OUTPUT);
  digitalWrite(B_PIN, LOW);
  pinMode(A_PIN, OUTPUT);
  digitalWrite(A_PIN, LOW);
  pinMode(C_PIN, OUTPUT);
  digitalWrite(C_PIN, LOW);
  pinMode(S_PIN, OUTPUT);
  digitalWrite(S_PIN, LOW);

  // Setup Interrupts
  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(TL_PIN), TLInterrupt, CHANGE); // PIN 7 interrupt requires leonardo (or equiv)
}

void loop() { 
 /*
delay(500);
digitalWrite(A_PIN, HIGH); // button A ON
PORTB = PORTB | B00100000; // port b button B ON

delay(500);
digitalWrite(A_PIN, LOW); // button A OFF
PORTB = PORTB & B11011111; // port b button B OFF
 */

/* Pulse test
delay(500);
buttons = 15;
delay(500);
buttons = 0;
*/
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
      case 'q':     // A
        Serial.println((String)"Trigger x:"+x+" y:"+y); 
        buttons ^= 0x02;
        break; 
      case 'e':     // B
        Serial.print("B\n");
        buttons ^= 0x01;
        break; 
      case 'z':     // C
        Serial.print("C\n");
        buttons ^= 0x04;
        break; 
      case 'c':     // Start
        Serial.print("Start\n");
        buttons ^= 0x08; 
        break; 
      case 't':
        y = 40;  // move to the top
        break;
      case 'r':  // move to the right
        x = 263;
        break;
      case 'b':  // move to the bottom
        y = 250;
        break;
      case 'l':  // move to the left
        x = 73;
        break; 
      case 'm':  // move to the middle
        x = 163;
        y = 136;
        break;
      case 'o': // point offscreen 
        y = 100;
        x = 0;
        break; 
    }

}
