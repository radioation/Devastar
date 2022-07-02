// sync variables
const int COMPOSITE_SYNC_1881 = 2;
const int VERTICAL_SYNC_1881 = 3;
volatile int verticalLine = 0;
const int TH_PIN = 8;

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
  if( verticalLine == y ) {
    //delayMicroseconds( x );
    delayCycles(x);
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
  Serial.println("Sync Test:");
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);

  pinMode(TH_PIN, OUTPUT);
  digitalWrite(TH_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
}

void loop() {
  while (Serial.available() > 0)
{
  x  = Serial.parseInt();
 Serial.println(x);
}
  // put your main code here, to run repeatedly:

}
