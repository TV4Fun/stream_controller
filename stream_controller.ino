#define DEBUG_FULL
#define DEBUG
const byte kBaseWaterPin = 2;
const byte kMaxWaterPin = 13;
const byte kShutPin = A1;
const byte kMovePin = A0;

const int kNumReadings = 3;  // Number of readings to average over for error correction
const int kReadingDelayMillis = 1;
const int kPollingInterval = 50;  // Millisecond delay between updates

const byte kTargetWaterLevel = 9;
const float kStreamLagMillis = 35000.0;  // Time from when a valve is adjusted to when we expect to see a change.
const float kValveMoveTimeMillis = 3500.0;  // Time to fully open or shut the valve
const int kAdjDeadZone = 250;  // Set a minimum adjustment to avoid making a lot of tiny changes
const float kFullOpenFillRate = 15.0 / 45000.0;

enum {
  STOPPED = 0,
  SHUTTING = -1,
  OPENING = 1
} valveMotionState = STOPPED;

byte lastWaterLevel = 0;
unsigned long lastChangeTimeMillis = 0;
unsigned long lastUpdateTimeMillis = 0;
float pendingAdjustments = 0.0;
long toMove = 0;
float fillRate = 0.0;

#ifdef DEBUG
void printPinOutputs() {
    for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin) {
      //Serial.print("Pin ");
      //Serial.print(pin);
      //Serial.print(": ");
      //Serial.println(digitalRead(pin));
      if (digitalRead(pin) == HIGH)
        Serial.print(pin);
      else
        Serial.print(0);
      Serial.print(' ');
  }
  Serial.print(lastWaterLevel);
  Serial.print('\n');
}
#endif

byte getWaterLevel() {
  /*byte waterLevel;
  bool validRead;
  do {
#ifdef DEBUG
    printPinOutputs();
#endif
    delay(10);
    waterLevel = kMaxWaterPin + 1;
    validRead = true;
    bool lastWasHigh = false;
    for (byte pin = kMaxWaterPin; pin >= kBaseWaterPin; --pin) {
      bool isHigh = digitalRead(pin) == HIGH;
      if (isHigh && !lastWasHigh)
        waterLevel = kMaxWaterPin + kBaseWaterPin - pin;
      else if (!isHigh && lastWasHigh)
        validRead = false;
      lastWasHigh = isHigh;
    }
  } while (!validRead);*/

  // Average over multiple reads for error reduction.
  float totalWaterLevel = 0.0;
  
  for (int i = 0; i < kNumReadings; ++i) {
    #ifdef DEBUG
      //printPinOutputs();
    #endif
    int waterLevel = 1;
    for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin) {
      if (digitalRead(pin) == LOW)
        ++waterLevel;
    }
    totalWaterLevel += waterLevel;
    delay(kReadingDelayMillis);
  }
  byte waterLevel = round(totalWaterLevel / kNumReadings);

  if (abs(waterLevel - lastWaterLevel) >= 2) {
    unsigned long changeTime = millis();
    unsigned long deltaT = changeTime - lastChangeTimeMillis;
    fillRate = (float)(waterLevel - lastWaterLevel) / (float)(deltaT);
    lastWaterLevel = waterLevel;
    lastChangeTimeMillis = changeTime;
  } else if ((waterLevel < lastWaterLevel && fillRate > 0.0) || (waterLevel > lastWaterLevel && fillRate < 0.0))
    fillRate = 0.0;
  
  return waterLevel;
}

void openValve() {
#ifdef DEBUG_FULL
  Serial.println("openValve");
#endif
  digitalWrite(kShutPin, LOW);
  digitalWrite(kMovePin, HIGH);
  valveMotionState = OPENING;
}

void shutValve() {
#ifdef DEBUG_FULL
  Serial.println("shutValve");
#endif
  digitalWrite(kShutPin, HIGH);
  digitalWrite(kMovePin, HIGH);
  valveMotionState = SHUTTING;
}

void stopValve() {
#ifdef DEBUG_FULL
  Serial.println("stopValve");
#endif
  digitalWrite(kMovePin, LOW);
  valveMotionState = STOPPED;
}

void moveValve(long ms) {
#ifdef DEBUG_FULL
  Serial.print("moveValve: ");
  Serial.println(ms);
#endif
  /*ms = constrain(ms, -kValveMoveTimeMillis, kValveMoveTimeMillis);  // Avoid making ridiculously large adjustments.
  if (ms < 0)
    shutValve();
  else
    openValve();
  delay(abs(ms));
  stopValve();
  pendingAdjustments += ms;*/
  toMove = constrain(toMove + ms, -kValveMoveTimeMillis, kValveMoveTimeMillis);  // Avoid making ridiculously large adjustments.
  if (toMove <= -kAdjDeadZone)
    shutValve();
  else if (toMove >= kAdjDeadZone)
    openValve();
  else if ((toMove <= 0 && valveMotionState == OPENING) || (toMove >= 0 && valveMotionState == SHUTTING))
    stopValve();
}

void setup() {
  pinMode(kShutPin, OUTPUT);
  pinMode(kMovePin, OUTPUT);
  digitalWrite(kMovePin, LOW);

  for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin)
    pinMode(pin, INPUT_PULLUP);
    
#ifdef DEBUG
  Serial.begin(9600);
  printPinOutputs();
  Serial.println("Started.");
#endif

  getWaterLevel();
  fillRate = -0.00001;
}

void handleChange() {
  float targetFillRate = (float)(kTargetWaterLevel - lastWaterLevel) / kStreamLagMillis;
  float targetAdjustment = kValveMoveTimeMillis * (targetFillRate - fillRate) / kFullOpenFillRate - pendingAdjustments - toMove;
#ifdef DEBUG_FULL
  Serial.print("toMove: ");
  Serial.println(toMove);
  Serial.print("lastWaterLevel: ");
  Serial.println(lastWaterLevel);
  Serial.print("targetFillRate: ");
  Serial.println(targetFillRate * 1000.0);
  Serial.print("fillRate: ");
  Serial.println(fillRate * 1000.0);
  Serial.print("targetAdjustment: ");
  Serial.println(targetAdjustment);
  Serial.print("pendingAdjustments: ");
  Serial.println(pendingAdjustments);

#endif
  moveValve(targetAdjustment);
}

void loop() {
  getWaterLevel();
  unsigned long updateTime = millis();
  unsigned long deltaT = updateTime - lastUpdateTimeMillis;
  toMove -= (long)(deltaT) * valveMotionState;
  Serial.print("deltaT: ");
  Serial.println(deltaT);
  Serial.print("valveMotionState: ");
  Serial.println(valveMotionState);
  pendingAdjustments = (pendingAdjustments + (float)(deltaT) * valveMotionState) * pow(0.5, (float)(deltaT) / kStreamLagMillis);
  handleChange();
  lastUpdateTimeMillis = updateTime;
  delay(kPollingInterval);
}
