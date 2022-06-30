// sync variables
const int COMPOSITE_SYNC_1881 = 2;
const int VERTICAL_SYNC_1881 = 3;
volatile int verticalLine = 0;

void compositeSyncInterrrupt() {
  verticalLine++;
}

void verticalSyncInterrrupt() {
    verticalLine = 0;
}
void setup() {
  Serial.begin(9600);
  Serial.println("Sync Test:");
  pinMode(COMPOSITE_SYNC_1881, INPUT);
  pinMode(VERTICAL_SYNC_1881, INPUT);

  attachInterrupt(digitalPinToInterrupt(COMPOSITE_SYNC_1881), compositeSyncInterrrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(VERTICAL_SYNC_1881), verticalSyncInterrrupt, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(verticalLine);

}
