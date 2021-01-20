//#define DEBUG_PLOTTER  // Output variables for Serial Plotter
//#define DEBUG_SENSOR  // Use to debug individual readings from each pin. Outputs for Serial Monitor.
//#define DEBUG_VARS  // Use to debug full set of variables. Outputs in text for Serial Monitor. Note that each output takes about 200ms, which can change the behavior of your controller a lot.
// #define DEBUG_BINARY  // Send out raw binary data at high speed for reading by a customized program. Do not use in combination with either other DEBUG variable.
#ifdef DEBUG_BINARY
#  define WRITE(x) Serial.write((byte*)(&x), sizeof(x))
#endif
const byte kShutPin = 12;  // White wire
const byte kMovePin = 13;  // Grey wire
const byte kDepthPin = A0;  // Orange wire from eTape

// Connect jumper from 3.3V to ARef. Connect Red wire on eTape and relay controller to 5V. Connect black wire on eTape and relay controller to GND.

// All times are in seconds unless otherwise specified.
const float kTargetWaterLevel = 9.0;
const float kStreamLag = 35.0;  // Time from when a valve is adjusted to when we expect to see a change.
const float kValveMoveTime = 3.5;  // Time to fully open or shut the valve
const float kFullOpenFillRate = 15.0 / 45.0;
const float kReadingAlpha = 0.975;  // Blend reading changes to reduce noise. 0 -> Always use latest reading.
const float kBaseFillRate = -0.15;  // Fill rate with no water input.
const float kAdjDeadZone = 0.25;  // Set a minimum adjustment to avoid making a lot of tiny changes. Make it variable so we can still fine tune.
const unsigned long kPollingIntervalMillis = 25;

// Water level parameters, levels in inches, resistance in ohms.
const float kMinWaterLevel = 1.0;  // Lowest water level that can be read.
const float kMaxWaterLevel = 12.4; // Highest water level that can be read.
const float kMaxLevelResistance = 400.0; // Sensor resistance at highest level.
const float kMinLevelResistance = 2000.0; // Sensor resistance at lowest level.
const float kRRef = 2000.0;  // Resistance of series reference sensor.
const float kDividerVin = 5.0;  // Voltage into sensor divider.
const float kARefVolts = 3.3;  // Analog reference voltage.
const float kWaterLevelRange = kMaxWaterLevel - kMinWaterLevel;
const float kInchesPerOhm = kWaterLevelRange / (kMaxLevelResistance - kMinLevelResistance);
const byte kAnalogRef = EXTERNAL;

float lastWaterLevelReading;
float estPosition = 0.0;

enum {
  STOPPED = 0,
  SHUTTING = -1,
  OPENING = 1
} valveMotionState = STOPPED;

float getSensorResistance() {
  float reading = (float)analogRead(kDepthPin);
#ifdef DEBUG_SENSOR
  Serial.print("Sensor reading: ");
  Serial.println(reading);
#endif
  float v_sense = reading / 1023.0 * kARefVolts;
#ifdef DEBUG_SENSOR
  Serial.print("Calculated voltage: ");
  Serial.println(v_sense);
#endif
  return kRRef / (kDividerVin / v_sense - 1.0);
}

float getWaterLevel() {
  float r_sense = getSensorResistance();
#ifdef DEBUG_SENSOR
  Serial.print("Calculated sensor resistance: ");
  Serial.println(r_sense);
  float reading = (r_sense - kMinLevelResistance) * kInchesPerOhm + kMinWaterLevel;
  Serial.print("Calculated water level: ");
  Serial.println(reading);
#endif
  
  return (r_sense - kMinLevelResistance) * kInchesPerOhm + kMinWaterLevel;
}

float getDampenedWaterLevel(float deltaT) {
  float waterLevel = getWaterLevel();
  float alpha = pow(kReadingAlpha, deltaT);
  float waterLevelReading = (1.0 - alpha) * waterLevel + alpha * lastWaterLevelReading;
#ifdef DEBUG_PLOTTER
  Serial.print(waterLevel);
  Serial.print(' ');
  Serial.print(waterLevelReading);
  Serial.print(' ');
#endif
  
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

void moveValve(float seconds, float deltaT) {
  static float toMove = 0.0;
#ifdef DEBUG_VARS
  Serial.print("moveValve: ");
  Serial.println(seconds);
#endif

  float oldToMove = toMove;
  toMove = constrain(toMove + seconds - valveMotionState * deltaT, -kValveMoveTime, kValveMoveTime);  // Avoid making ridiculously large adjustments.
  estPosition += constrain(toMove + seconds, -kValveMoveTime, kValveMoveTime) - toMove;
  estPosition = constrain(estPosition, 0.0, kValveMoveTime);  // Keep our estimate of valve positon to where it is physically possible to be.
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
  WRITE(estPosition);
#endif
#ifdef DEBUG_PLOTTER
  Serial.println(toMove);
#endif
}

void setup() {
  analogReference(kAnalogRef);

  pinMode(kShutPin, OUTPUT);
  pinMode(kMovePin, OUTPUT);
  digitalWrite(kMovePin, LOW);
    
#if defined(DEBUG_VARS) || defined(DEBUG_SENSOR)
  Serial.begin(9600);
  Serial.println("Started.");
#elif defined(DEBUG_PLOTTER)
  Serial.begin(9600);
  Serial.println("Level Dampened d d2 activation toMove");
#elif defined(DEBUG_BINARY)
  Serial.begin(115200);
#endif

  lastWaterLevelReading = getWaterLevel();
}

float getServoGain(float error, float deltaT) {
  const static float kIGain = 1.0 / kStreamLag;
  const static float kIAlpha = 0.9;  // Exponent to reduce integral by per time 1 -> do not reduce over time.
  const static float kI2Gain = kIGain / kStreamLag;
  const static float kI2Alpha = 0.9;  // Exponent to reduce second integral by per time 1 -> do not reduce over time.
  const static float kPGain = -0.1;
  const static float kPosGain = -0.2;
  const static float kDGain = -kStreamLag / kFullOpenFillRate / 10.0;
  const static float kD2Gain = 0.0; // kDGain / kFullOpenFillRate;

  static float lastError = 0.0;
  static float i = 0.0;
  static float i2 = 0.0;
  static float lastD = 0.0;

  float d = deltaT == 0 ? 0.0 : (error - lastError) / deltaT;
  float d2 =  deltaT == 0 ? 0.0 : (d - lastD) / deltaT;
  i = i * pow(kIAlpha, deltaT) + error * deltaT;
  i2 = i2 * pow(kI2Alpha, deltaT) + i * deltaT;

  lastError = error;
  lastD = d;
  float activation = /*i2 * kI2Gain + i * kIGain +*/ error * kPGain + d * kDGain + d2 * kD2Gain + kPosGain * estPosition;

#ifdef DEBUG_BINARY
  WRITE(i2);
  WRITE(i);
  WRITE(error);
  WRITE(d);
  WRITE(d2);
  WRITE(activation);
#endif
#ifdef DEBUG_PLOTTER
  Serial.print(d);
  Serial.print(' ');
  Serial.print(d2);
  Serial.print(' ');
  Serial.print(activation);
  Serial.print(' ');
#endif

  return activation;
}

void loop() {
  static unsigned long lastUpdateTimeMillis = 0;
  unsigned long updateTime = millis();
  float deltaT = (float)(updateTime - lastUpdateTimeMillis) / 1000.0;
#ifdef DEBUG_BINARY
  Serial.write(0xDE);
  Serial.write(0xAD);
  Serial.write(0xBE);
  Serial.write(0xEF);
  WRITE(updateTime);
  WRITE(deltaT);
  WRITE(valveMotionState);
#endif
  float error = getDampenedWaterLevel(deltaT) - kTargetWaterLevel;
  moveValve(getServoGain(error, deltaT), deltaT);
#ifdef DEBUG_VARS
  Serial.print("deltaT: ");
  Serial.println(deltaT);
  Serial.print("valveMotionState: ");
  Serial.println(valveMotionState);
#endif

  lastUpdateTimeMillis = updateTime;
  delay(kPollingIntervalMillis);
}
