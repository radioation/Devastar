// sync variables
const int COMPOSITE_SYNC_1881 = 2;
const int VERTICAL_SYNC_1881 = 3;
volatile int verticalLine = 0;
const int TH_PIN = 8;

int y = 102;  // line count
int x = 45;      // delay


void compositeSyncInterrrupt() {
  verticalLine++;
  if( verticalLine == y ) {
    delayMicroseconds( x );
    digitalWrite( TH_PIN, HIGH );
    delayMicroseconds( 5 );
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
  // put your main code here, to run repeatedly:
  Serial.println(verticalLine);

}
