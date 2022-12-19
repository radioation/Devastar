

#include "HID-Project.h"
// #include <SoftwareSerial.h>


void setup() {

  Serial1.begin(9600);
  //Serial.begin(9600);
  AbsoluteMouse.begin();
}

void loop() {


  if (Serial1.available() )  {
    // read byte data from Bluetooth
    unsigned char data = Serial1.read();
    if (data & 0x80) { // is it a start byte?
      byte x1 = data & 0x7f; 
      // next 4 bytes are the rest of the  data
      while(Serial1.available()<4); 
      short int x2 = Serial1.read();
      if( x2 & 0x40 ) {
        x2 |= 0x180;
      }
      unsigned char y1 = Serial1.read();
      short int y2 = Serial1.read();
      if( y2 & 0x40 ) {
        y2 |= 0x180;
      }
      unsigned char buttons = Serial1.read();
      // get X/Y from bytes.
      short x = x1 | (x2 << 7);
      short y = y1 | (y2 << 7);

      // compute mouse position.
      int mx = (float(x) / 1920.0) * 65535 - 32768;
      int my = (float(y) / 1080.0) * 65535 - 32768;
      AbsoluteMouse.moveTo(mx, my);

      
      if (buttons & 0x01) {
        AbsoluteMouse.click();
      }
      if (buttons & 0x02) {
        AbsoluteMouse.click(MOUSE_RIGHT);
      }
      if (buttons & 0x04) {
        AbsoluteMouse.click(MOUSE_MIDDLE);
      }
    }
  }
}
