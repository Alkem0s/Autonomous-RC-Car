#include <Servo.h>


// ========== PIN DEFINITIONS ==========
// Sensors
const int TRIG1 = 2; const int ECHO1 = 3;
const int TRIG2 = 4; const int ECHO2 = 5;
const int TRIG3 = 6; const int ECHO3 = 7;
const int TRIG4 = 8; const int ECHO4 = 9;

// Motor and Servo
const byte ESC_PIN = 10;      // ESC pin for motor
const byte SERVO_PIN = 11;    // Servo pin for steering

// ========== ESC PULSE SETTINGS ==========
const int PULSE_MIN = 1000;      // Full Reverse
const int PULSE_NEUTRAL = 1500;  // Stop
const int PULSE_MAX = 2000;      // Full Forward

// ========== CALIBRATION SETTINGS ========
const int RIGHT_ANGLE = 91;

// ========== SENSOR SETTINGS ==========
const unsigned long READ_INTERVAL = 250;
const int SAMPLES = 6;
const unsigned long TIMEOUT = 12000UL;

// Sensor Calibration
float SENSOR_OFFSET_CM = 0.0;
float SENSOR_SCALE = 1.0;

// ========== TIMING VARIABLES ==========
unsigned long lastSensorRead = 0;
unsigned long intervalSensors = 50;
unsigned long lastAutonomousUpdate = 0;
unsigned long intervalAutonomous = 100;

// ========== OBJECTS ==========
Servo esc;
Servo steeringServo;

// ========== DEBUG FLAGS ==========
bool DEBUG_SENSORS = false;
bool DEBUG_BLUETOOTH = false;
bool DEBUG_SERIAL = false;

// ========== CONTROL VARIABLES ==========
int currentSpeed = PULSE_NEUTRAL;
int currentAngle = 90;  // Center position

bool autonomousMode = false;

// ========== RECTANGLE MODE FLAGS ==========
bool rectangleMode = true;
bool repeatRectangle = true;

unsigned long lastRectStateChange = 0;
int rectSide = 0;
enum RectState { RECT_FORWARD, RECT_TURNING, RECT_PAUSED } rectState = RECT_FORWARD;

const unsigned long forwardTime  = 300;
const unsigned long turningTime  = 300;
const unsigned long pauseTime    = 8000;

const unsigned long forwardSpeed    = 1530;
const unsigned long turnSpeed    = 1530;

// ====== BRAKING FUNCTION PROTOTYPES ======
void startBraking(int targetPulse = PULSE_NEUTRAL);
void updateBraking();

// ========== BRAKING SETTINGS ==========
const int BRAKE_STEP = 10;           // µs change per step
const unsigned long BRAKE_INTERVAL = 20; // ms between steps

// ========== OBSTACLE BRAKING SETTINGS ==========
const float OBSTACLE_BRAKE_CM = 120.0;   // start braking below this distance
const float OBSTACLE_STOP_CM  = 90.0;   // full stop below this distance

bool obstacleBrakeActive = false;

bool brakingActive = false;
int brakeTarget = PULSE_NEUTRAL;
unsigned long lastBrakeUpdate = 0;

// serial buffer
String serialBuffer = "";

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);
  
  // Initialize ultrasonic sensors
  pinMode(TRIG1, OUTPUT); digitalWrite(TRIG1, LOW);
  pinMode(TRIG2, OUTPUT); digitalWrite(TRIG2, LOW);
  pinMode(TRIG3, OUTPUT); digitalWrite(TRIG3, LOW);
  pinMode(TRIG4, OUTPUT); digitalWrite(TRIG4, LOW);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(ECHO4, INPUT);
  
  // Initialize ESC and Servo
  esc.attach(ESC_PIN, PULSE_MIN, PULSE_MAX);
  steeringServo.attach(SERVO_PIN);
  
  esc.writeMicroseconds(PULSE_NEUTRAL);
  steeringServo.write(RIGHT_ANGLE);
  delay(5000);
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long now = millis();

  // Read sensors periodically
  if (every(lastSensorRead, intervalSensors)) {
   // readAndCheckSensors();
    obstacleBrakeCheck();
  }
  
  // Autonomous steering control
  if (autonomousMode && every(lastAutonomousUpdate, intervalAutonomous)) {
   // autonomousSteering();
  }
  
  // Handle Bluetooth commands
  if (Serial1.available()) {
    handleBluetoothCommand();
  }
  
  // Handle Serial Monitor commands
  if (Serial.available()) {
    handleSerialCommand();
  }
  
  updateBraking();

  if (!obstacleBrakeActive) {
    driveRectangleMode();
  }
  
}

// ========== AUTONOMOUS CONTROL ==========
//  TODO IMPLEMENT ; void autonomousSteering() {

void obstacleBrakeCheck() {
  float d1 = readSensorMedianCm(TRIG1, ECHO1, SAMPLES);
  float d2 = readSensorMedianCm(TRIG2, ECHO2, SAMPLES);
  float d3 = readSensorMedianCm(TRIG3, ECHO3, SAMPLES);
  float d4 = readSensorMedianCm(TRIG4, ECHO4, SAMPLES);

  float minDist = 9999;

  if (d1 > 0 && d1 < minDist) minDist = d1;
  if (d2 > 0 && d2 < minDist) minDist = d2;
  if (d3 > 0 && d3 < minDist) minDist = d3;
  if (d4 > 0 && d4 < minDist) minDist = d4;

  if (DEBUG_SERIAL) {
    printSensorReadings(d1, d2, d3, d4, minDist);
  }

  // Nothing detected
  if (minDist == 9999) {
    obstacleBrakeActive = false;
    return;
  }

  // EMERGENCY STOP
  if (minDist <= OBSTACLE_STOP_CM) {
    brakingActive = false;              // cancel smooth braking
    setSpeed(PULSE_NEUTRAL);             // hard stop
    obstacleBrakeActive = true;
    return;
  }

  // SOFT BRAKE
  if (minDist <= OBSTACLE_BRAKE_CM) {
    obstacleBrakeActive = true;

    // Map distance to speed target
    int target =
      map((int)(minDist * 10),
          (int)(OBSTACLE_STOP_CM * 10),
          (int)(OBSTACLE_BRAKE_CM * 10),
          PULSE_NEUTRAL,
          currentSpeed);

    startBraking(target);
  } else {
    obstacleBrakeActive = false;
  }
}

// ========== RECTANGLE DRIVER ==========
void driveRectangleMode() {
  if (!rectangleMode) return;

  unsigned long now = millis();

  switch (rectState) {

    case RECT_FORWARD:
      setSteering(RIGHT_ANGLE);
      setSpeed(1900);

      if (now - lastRectStateChange >= forwardTime) {
        //startBraking(1650);   // slow before turn
        rectState = RECT_TURNING;
        lastRectStateChange = now;
      }
      break;

    case RECT_TURNING:
      setSteering(45);        // left turn; use 135 for right turn
      setSpeed(turnSpeed);

      if (now - lastRectStateChange >= turningTime) {
        rectSide++;

        if (rectSide >= 4) {
          // Completed rectangle
          startBraking(PULSE_NEUTRAL);
          rectState = RECT_PAUSED;
          lastRectStateChange = now;

          if (!repeatRectangle) rectangleMode = false;
        } else {
          rectState = RECT_FORWARD;
          lastRectStateChange = now;
        }
      }
      break;

    case RECT_PAUSED:
      setSteering(RIGHT_ANGLE);

      if (!brakingActive && repeatRectangle &&
          (now - lastRectStateChange >= pauseTime)) {
        rectSide = 0;
        rectState = RECT_FORWARD;
        lastRectStateChange = now;
      }
      break;
  }
}

// ========== SENSOR FUNCTIONS ==========
void readAndCheckSensors() {
  if (!DEBUG_SENSORS) return;
  
  float d1 = readSensorMedianCm(TRIG1, ECHO1, SAMPLES);
  float d2 = readSensorMedianCm(TRIG2, ECHO2, SAMPLES);
  float d3 = readSensorMedianCm(TRIG3, ECHO3, SAMPLES);
  float d4 = readSensorMedianCm(TRIG4, ECHO4, SAMPLES);
  
  Serial.print("S1: "); printDistance(d1);
  Serial.print(" | S2: "); printDistance(d2);
  Serial.print(" | S3: "); printDistance(d3);
  Serial.print(" | S4: "); printDistance(d4);
  Serial.println();
}

unsigned long singlePulseDuration(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, TIMEOUT);
  return duration;
}

float readSensorMedianCm(int trigPin, int echoPin, int samples) {
  unsigned long arr[5]; // Max 5 sample yeterli
  if (samples > 5) samples = 5;
  int validCount = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    unsigned long dur = pulseIn(echoPin, HIGH, TIMEOUT);

    if (dur > 0) {
      arr[validCount++] = dur;
    }
  }

  if (validCount == 0) return -1;

  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (arr[j] < arr[i]) {
        unsigned long t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }

  unsigned long medianDur;
  if (validCount % 2 == 1) medianDur = arr[validCount / 2];
  else medianDur = (arr[validCount / 2 - 1] + arr[validCount / 2]) / 2;

  // Convert duration to cm: speed of sound ~343 m/s
  float distanceCm = medianDur / 58.0; // typical for HC-SR04
  return (distanceCm + SENSOR_OFFSET_CM) * SENSOR_SCALE;
}

void printDistance(float d) {
  if (d < 0) Serial.print("NO READING");
  else {
    Serial.print(d, 1);
    Serial.print("cm");
  }
}

// ========== BLUETOOTH CONTROL ==========
void handleBluetoothCommand() {
  char command = Serial1.read();
  
  if (DEBUG_BLUETOOTH) {
    Serial.print("BT Command: ");
    Serial.println(command);
  }
  
  switch (command) {
    case 'F': // Forward
      autonomousMode = false;
      setSpeed(1600);
      if (DEBUG_BLUETOOTH) Serial.println("Moving Forward");
      break;
      
    case 'B': // Backward
      autonomousMode = false;
      setSpeed(1400);
      if (DEBUG_BLUETOOTH) Serial.println("Moving Backward");
      break;
      
    case 'S': // Stop
      autonomousMode = false;
      setSpeed(PULSE_NEUTRAL);
      if (DEBUG_BLUETOOTH) Serial.println("Stopped");
      break;
      
    case 'L': // Turn Left
      autonomousMode = false;
      setSteering(45);
      if (DEBUG_BLUETOOTH) Serial.println("Steering Left");
      break;
      
    case 'R': // Turn Right
      autonomousMode = false;
      setSteering(135);
      if (DEBUG_BLUETOOTH) Serial.println("Steering Right");
      break;
      
    case 'C': // Center
      autonomousMode = false;
      setSteering(90);
      if (DEBUG_BLUETOOTH) Serial.println("Steering Centered");
      break;
      
    case 'A': // Autonomous Mode
      autonomousMode = !autonomousMode;
      if (DEBUG_BLUETOOTH) {
        Serial.print("Autonomous Mode: ");
        Serial.println(autonomousMode ? "ON" : "OFF");
      }
      break;
      
    default:
      if (DEBUG_BLUETOOTH) Serial.println("Unknown command!");
      break;
  }
}

// ========== SERIAL MONITOR CONTROL ==========
void handleSerialCommand() {
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') {
      continue;   // ignore CR
    }

    if (c == '\n') {
      buf[idx] = 0;

      if (idx > 0) {
        // parse
        int v1 = 0;
        int v2 = 0;
        bool hasSecond = false;

        // find space
        char *space = strchr(buf, ' ');
        if (space) {
          *space = 0;
          space++;

          if (isNumber(buf)) v1 = atoi(buf);
          if (isNumber(space)) {
            v2 = atoi(space);
            hasSecond = true;
          }
        } else {
          if (isNumber(buf)) v1 = atoi(buf);
        }

        // apply logic
        if (hasSecond) {
          if (v1 >= PULSE_MIN && v1 <= PULSE_MAX) {
            setSpeed(v1);
            if (v2 >= 0 && v2 <= 180) setSteering(v2);
          } else if (v1 >= 0 && v1 <= 180) {
            setSteering(v1);
            if (v2 >= PULSE_MIN && v2 <= PULSE_MAX) setSpeed(v2);
          }
        } else {
          if (v1 >= PULSE_MIN && v1 <= PULSE_MAX) setSpeed(v1);
          else if (v1 >= 0 && v1 <= 180) setSteering(v1);
        }
      }

      idx = 0;
      continue;
    }

    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

// ========== MOTOR & SERVO CONTROL ==========
void setSpeed(int pulse) {
  currentSpeed = constrain(pulse, PULSE_MIN, PULSE_MAX);
  esc.writeMicroseconds(currentSpeed);
}

void setSteering(int angle) {
  currentAngle = constrain(angle, 0, 180);
  steeringServo.write(currentAngle);
}

void startBraking(int targetPulse = PULSE_NEUTRAL) {
  brakeTarget = constrain(targetPulse, PULSE_MIN, PULSE_MAX);
  brakingActive = true;
}

void updateBraking() {
  if (!brakingActive) return;

  unsigned long now = millis();
  if (now - lastBrakeUpdate < BRAKE_INTERVAL) return;
  lastBrakeUpdate = now;

  if (currentSpeed > brakeTarget) {
    currentSpeed -= BRAKE_STEP;
    if (currentSpeed < brakeTarget) currentSpeed = brakeTarget;
    esc.writeMicroseconds(currentSpeed);
  } else {
    brakingActive = false; // done braking
  }
}

// ========== UTILITY FUNCTIONS ==========
bool every(unsigned long &last, unsigned long interval) {
  unsigned long now = millis();
  if (now - last >= interval) {
    last = now;
    return true;
  }
  return false;
}

void printSensorReadings(float d1, float d2, float d3, float d4, float minDist) {
  Serial.print("S1: ");
  if (d1 > 0) Serial.print(d1, 1); else Serial.print("--");
  Serial.print(" cm | S2: ");
  if (d2 > 0) Serial.print(d2, 1); else Serial.print("--");
  Serial.print(" cm | S3: ");
  if (d3 > 0) Serial.print(d3, 1); else Serial.print("--");
  Serial.print(" cm | S4: ");
  if (d4 > 0) Serial.print(d4, 1); else Serial.print("--");

  Serial.print(" cm | MIN: ");
  if (minDist < 9999) Serial.print(minDist, 1);
  else Serial.print("--");

  Serial.println(" cm");
}

bool isNumber(const char *s) {
  if (*s == 0) return false;
  if (*s == '+' || *s == '-') s++;
  if (*s == 0) return false;

  while (*s) {
    if (*s < '0' || *s > '9') return false;
    s++;
  }
  return true;
}
