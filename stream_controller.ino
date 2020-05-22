// #define DEBUG_PINS  // Use to debug individual readings from each pin. Outputs for Serial Plotter.
#define DEBUG_VARS  // Use to debug full set of variables. Outputs in text for Serial Monitor.
const byte kBaseWaterPin = 2;
const byte kMaxWaterPin = 13;
const byte kShutPin = A1;
const byte kMovePin = A0;

const byte kTargetWaterLevel = 9;
const float kStreamLagMillis = 35000.0;  // Time from when a valve is adjusted to when we expect to see a change.
const float kValveMoveTimeMillis = 3500.0;  // Time to fully open or shut the valve
const float kFullOpenFillRate = 30.0 / 45000.0;
const float kReadingAlpha = 0.1;  // Blend reading changes to reduce noise. 1 -> Always use latest reading.
const byte kBaseWaterLevel = 1;  // Lowest water level that can be read.
const float kBaseFillRate = -0.00001;  // Fill rate with no water input.
const float kAdjustmentDamping = 0.5;  // Avoid hunting by damping adjustment factor.

enum {
  STOPPED = 0,
  SHUTTING = -1,
  OPENING = 1
} valveMotionState = STOPPED;

float lastWaterLevel = 0.0;
unsigned long lastUpdateTimeMillis = 0;
float pendingAdjustments = 0.0;
long toMove = 0;
float fillRate = 0.0;
float toMoveActual = 0.0;  // Use a weighted average to reduce hunting;
const float kToMoveAlpha = 0.1;  // Weight for toMoveActual. 1 -> actual toMove.
float adjDeadZone = 400.0;  // Set a minimum adjustment to avoid making a lot of tiny changes. Make it variable so we can still fine tune.

#ifdef DEBUG_PINS
void printPinOutputs() {
    for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin) {
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

float getWaterLevel(unsigned long deltaT) {
  #ifdef DEBUG_PINS
    printPinOutputs();
  #endif
  int waterLevel = kBaseWaterLevel;
  for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin) {
    if (digitalRead(pin) == LOW)
      ++waterLevel;
  }
  float waterLevelReading = kReadingAlpha * (float)waterLevel + (1.0 - kReadingAlpha) * lastWaterLevel;
#ifdef DEBUG_VARS
  Serial.print("waterLevel: ");
  Serial.println(waterLevel);
  Serial.print("waterLevelReading: ");
  Serial.println(waterLevelReading);
#endif
  fillRate = (waterLevelReading - lastWaterLevel) / (float)(deltaT);
  lastWaterLevel = waterLevelReading;
  
  return waterLevel;
}

void openValve() {
#ifdef DEBUG_VARS
  Serial.println("openValve");
#endif
  //adjDeadZone *= 2.0;
  digitalWrite(kShutPin, LOW);
  digitalWrite(kMovePin, HIGH);
  valveMotionState = OPENING;
}

void shutValve() {
#ifdef DEBUG_VARS
  Serial.println("shutValve");
#endif
  //adjDeadZone *= 2.0;
  digitalWrite(kShutPin, HIGH);
  digitalWrite(kMovePin, HIGH);
  valveMotionState = SHUTTING;
}

void stopValve() {
#ifdef DEBUG_VARS
  Serial.println("stopValve");
#endif
  digitalWrite(kMovePin, LOW);
  valveMotionState = STOPPED;
}

void moveValve(long ms) {
#ifdef DEBUG_VARS
  Serial.print("moveValve: ");
  Serial.println(ms);
#endif
  toMove = constrain(toMove + ms, -kValveMoveTimeMillis, kValveMoveTimeMillis);  // Avoid making ridiculously large adjustments.
  toMoveActual = kToMoveAlpha * toMove + (1 - kToMoveAlpha) * toMoveActual;
  if (toMoveActual <= -adjDeadZone) {
    shutValve();
  }
  else if (toMoveActual >= adjDeadZone) {
    openValve();
  }
  else if ((toMoveActual <= 0 && valveMotionState == OPENING) || (toMoveActual >= 0 && valveMotionState == SHUTTING))
  {
    stopValve();
  }
}

void setup() {
  pinMode(kShutPin, OUTPUT);
  pinMode(kMovePin, OUTPUT);
  digitalWrite(kMovePin, LOW);

  for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin)
    pinMode(pin, INPUT_PULLUP);
    
#if defined(DEBUG_VARS) || defined(DEBUG_PINS)
  Serial.begin(9600);
  Serial.println("Started.");
#endif
#ifdef DEBUG_PINS
  printPinOutputs();
#endif

  getWaterLevel(0);
  fillRate = kBaseFillRate;
}

void handleChange() {
  float targetFillRate = (float)(kTargetWaterLevel - lastWaterLevel) / kStreamLagMillis;
  float targetAdjustment = kValveMoveTimeMillis * (targetFillRate - fillRate) / kFullOpenFillRate - pendingAdjustments - toMoveActual;
#ifdef DEBUG_VARS
  Serial.print("toMove: ");
  Serial.println(toMove);
  Serial.print("toMoveActual: ");
  Serial.println(toMoveActual);
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
  Serial.print("adjDeadZone: ");
  Serial.println(adjDeadZone);
#endif
  moveValve(targetAdjustment);
}

void loop() {
  unsigned long updateTime = millis();
  unsigned long deltaT = updateTime - lastUpdateTimeMillis;
  getWaterLevel(deltaT);
  toMove -= (long)(deltaT) * valveMotionState;
  toMoveActual -= (float)(deltaT) * valveMotionState;
  Serial.print("deltaT: ");
  Serial.println(deltaT);
  Serial.print("valveMotionState: ");
  Serial.println(valveMotionState);
  pendingAdjustments = (pendingAdjustments + (float)(deltaT) * valveMotionState) * pow(0.5, (float)(deltaT) / kStreamLagMillis);
  //adjDeadZone *= pow(0.5, (float)(deltaT) / 2000.0);
  handleChange();
  lastUpdateTimeMillis = updateTime;
  delay(25);
}
