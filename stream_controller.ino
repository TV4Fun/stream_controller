// #define DEBUG_PINS  // Use to debug individual readings from each pin. Outputs for Serial Plotter.
// #define DEBUG_VARS  // Use to debug full set of variables. Outputs in text for Serial Monitor. Note that each output takes about 200ms, which can change the behavior of your controller a lot.
#define DEBUG_BINARY  // Send out raw binary data at high speed for reading by a customized program. Do not use in combination with either other DEBUG variable.
#ifdef DEBUG_BINARY
#  define WRITE(x) Serial.write((byte*)(&x), sizeof(x))
#endif
const byte kBaseWaterPin = 2;
const byte kMaxWaterPin = 13;
const byte kShutPin = A1;
const byte kMovePin = A0;

const float kTargetWaterLevel = 9.0;
const float kStreamLagMillis = 35000.0;  // Time from when a valve is adjusted to when we expect to see a change.
const float kValveMoveTimeMillis = 3500.0;  // Time to fully open or shut the valve
const float kFullOpenFillRate = 15.0 / 45000.0;
const float kReadingAlpha = 0.1;  // Blend reading changes to reduce noise. 1 -> Always use latest reading.
const byte kBaseWaterLevel = 1;  // Lowest water level that can be read.
const float kBaseFillRate = -0.00001;  // Fill rate with no water input.
const float kAdjDeadZone = 250.0;  // Set a minimum adjustment to avoid making a lot of tiny changes. Make it variable so we can still fine tune.
const long kPollingInterval = 25;

float lastWaterLevelReading;

enum {
  STOPPED = 0,
  SHUTTING = -1,
  OPENING = 1
} valveMotionState = STOPPED;

#ifdef DEBUG_PINS
void printPinOutputs() {
    for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin) {
      if (digitalRead(pin) == HIGH)
        Serial.print(pin);
      else
        Serial.print(0);
      Serial.print(' ');
  }
  Serial.print('\n');
}
#endif

byte getWaterLevel() {
  #ifdef DEBUG_PINS
    printPinOutputs();
  #endif
  byte waterLevel = kBaseWaterLevel;
  for (byte pin = kBaseWaterPin; pin <= kMaxWaterPin; ++pin) {
    if (digitalRead(pin) == LOW)
      ++waterLevel;
  }

  return waterLevel;
}

float getDampenedWaterLevel(unsigned long deltaT) {
  byte waterLevel = getWaterLevel();
  float deltaTSeconds = (float)(deltaT) / 1000.0;
  float alpha = pow(kReadingAlpha, deltaTSeconds);
  float waterLevelReading = alpha * (float)waterLevel + (1.0 - alpha) * lastWaterLevelReading;
#ifdef DEBUG_VARS
  Serial.print("waterLevel: ");
  Serial.println(waterLevel);
  Serial.print("waterLevelReading: ");
  Serial.println(waterLevelReading);
#endif
#ifdef DEBUG_BINARY
  WRITE(waterLevel);
  WRITE(waterLevelReading);
#endif
  lastWaterLevelReading = waterLevelReading;
  
  return waterLevelReading;
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
  static long toMove = 0;
#ifdef DEBUG_VARS
  Serial.print("moveValve: ");
  Serial.println(ms);
#endif
  toMove = constrain(toMove + ms, -kValveMoveTimeMillis, kValveMoveTimeMillis);  // Avoid making ridiculously large adjustments.
  if (toMove <= -kAdjDeadZone) {
    shutValve();
  }
  else if (toMove >= kAdjDeadZone) {
    openValve();
  }
  else if ((toMove <= 0 && valveMotionState == OPENING) || (toMove >= 0 && valveMotionState == SHUTTING))
  {
    stopValve();
  }

#ifdef DEBUG_BINARY
  WRITE(toMove);
#endif
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
#elif defined(DEBUG_BINARY)
  Serial.begin(115200);
#endif
#ifdef DEBUG_PINS
  printPinOutputs();
#endif

  lastWaterLevelReading = getWaterLevel();
}

float getServoGain(float error, unsigned long deltaT) {
  const static float kIGain = -1.0;
  const static float kI2Gain = -1.0;
  const static float kPGain = -1.0;
  const static float kDGain = -1.0;
  const static float kD2Gain = 1.0;

  static float lastError = 0.0;
  static float i = 0.0;
  static float i2 = 0.0;
  static float lastD = kBaseFillRate;

  float d = (error - lastError) / deltaT;
  float d2 = (d - lastD) / deltaT;
  i += error * deltaT;
  i2 += i * deltaT;

  lastError = error;
  lastD = d;

#ifdef DEBUG_BINARY
  WRITE(i2);
  WRITE(i);
  WRITE(error);
  WRITE(d);
  WRITE(d2);
#endif

  return i2 * kI2Gain + i * kIGain + error * kPGain + d * kDGain + d2 * kD2Gain;
}

void loop() {
  static unsigned long lastUpdateTimeMillis = 0;
  unsigned long updateTime = millis();
  unsigned long deltaT = updateTime - lastUpdateTimeMillis;
#ifdef DEBUG_BINARY
  const unsigned long kSentinel = 0xDEADBEEF;
  WRITE(kSentinel);
  WRITE(updateTime);
  WRITE(deltaT);
  WRITE(valveMotionState);
#endif
  float error = kTargetWaterLevel - getDampenedWaterLevel(deltaT);
  moveValve(getServoGain(error, deltaT));
  #ifdef DEBUG_VARS
    Serial.print("deltaT: ");
    Serial.println(deltaT);
    Serial.print("valveMotionState: ");
    Serial.println(valveMotionState);
  #endif

  lastUpdateTimeMillis = updateTime;
  delay(kPollingInterval);
}
