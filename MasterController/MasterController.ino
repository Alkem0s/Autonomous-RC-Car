#include <Servo.h>

// ========== PIN DEFINITIONS ==========
// Sensors
const int TRIG1 = 2;
const int ECHO1 = 3;
const int TRIG2 = 4;
const int ECHO2 = 5;
const int TRIG3 = 6;
const int ECHO3 = 7;
const int TRIG4 = 8;
const int ECHO4 = 9;

// Motor and Servo
const byte ESC_PIN = 10;      // ESC pin for motor
const byte SERVO_PIN = 11;    // Servo pin for steering

// ========== ESC PULSE SETTINGS ==========
const int PULSE_MIN = 1000;      // Full Reverse
const int PULSE_NEUTRAL = 1500;  // Stop
const int PULSE_MAX = 2000;      // Full Forward

// ========== SENSOR SETTINGS ==========
const unsigned long READ_INTERVAL = 250;
const int SAMPLES = 9;
const unsigned long TIMEOUT = 30000UL;
const int INTER_SENSOR_DELAY = 100;

// Sensor Calibration
float SENSOR_OFFSET_CM = 0.0;
float SENSOR_SCALE = 1.0;

// ========== TIMING VARIABLES ==========
unsigned long lastSensorRead = 0;
unsigned long intervalSensors = 250;
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
  
  // Arm the ESC
  Serial.println("=== ARDUINO CAR MASTER CONTROL ===");
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(PULSE_NEUTRAL);
  delay(3000);
  Serial.println("Ready!");
  
  delay(50);
}

// ========== MAIN LOOP ==========
void loop() {
  // Read sensors periodically
  if (every(lastSensorRead, intervalSensors)) {
    readAndCheckSensors();
  }
  
  // Autonomous steering control
  if (autonomousMode && every(lastAutonomousUpdate, intervalAutonomous)) {
    autonomousSteering();
  }
  
  // Handle Bluetooth commands
  if (Serial1.available()) {
    handleBluetoothCommand();
  }
  
  // Handle Serial Monitor commands
  if (Serial.available()) {
    handleSerialCommand();
  }
}

// ========== AUTONOMOUS CONTROL ==========
void autonomousSteering() {
  // Placeholder: Do nothing for now
}

// ========== SENSOR FUNCTIONS ==========
void readAndCheckSensors() {
  if (!DEBUG_SENSORS) return;
  
  float d1 = readSensorMedianCm(TRIG1, ECHO1, SAMPLES);
  delay(INTER_SENSOR_DELAY);
  float d2 = readSensorMedianCm(TRIG2, ECHO2, SAMPLES);
  delay(INTER_SENSOR_DELAY);
  float d3 = readSensorMedianCm(TRIG3, ECHO3, SAMPLES);
  delay(INTER_SENSOR_DELAY);
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
  unsigned long arr[31];
  if (samples > 31) samples = 31;
  for (int i = 0; i < samples; i++) {
    arr[i] = singlePulseDuration(trigPin, echoPin);
    delay(20);
  }
  int n = samples;
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        unsigned long t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }
  unsigned long medianDur;
  if (n % 2 == 1) medianDur = arr[n / 2];
  else medianDur = (arr[n / 2 - 1] + arr[n / 2]) / 2;
  return (medianDur + SENSOR_OFFSET_CM) * SENSOR_SCALE;
}

void printDistance(float d) {
  if (d < 0) Serial.print("ERROR");
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
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  // Check for autonomous mode toggle
  if (input.equalsIgnoreCase("AUTO")) {
    autonomousMode = !autonomousMode;
    Serial.print("Autonomous Mode: ");
    Serial.println(autonomousMode ? "ON" : "OFF");
    return;
  }
  
  // Manual control disables autonomous mode
  autonomousMode = false;
  
  // Parse command format: "SPEED ANGLE" or single value
  int spaceIndex = input.indexOf(' ');
  
  if (spaceIndex > 0) {
    // Format: "SPEED ANGLE"
    int val1 = input.substring(0, spaceIndex).toInt();
    int val2 = input.substring(spaceIndex + 1).toInt();
    
    if (val1 >= PULSE_MIN && val1 <= PULSE_MAX) {
      setSpeed(val1);
      if (DEBUG_SERIAL) {
        Serial.print("Speed: ");
        Serial.print(val1);
      }
    }
    
    if (val2 > 0 && val2 < 180) {
      setSteering(val2);
      if (DEBUG_SERIAL) {
        Serial.print(" | Angle: ");
        Serial.print(val2);
      }
    }
  } else {
    // Single value
    int value = input.toInt();
    
    if (value > 0 && value < 180) {
      setSteering(value);
      if (DEBUG_SERIAL) {
        Serial.print("Angle: ");
        Serial.print(value);
      }
    } else if (value >= PULSE_MIN && value <= PULSE_MAX) {
      setSpeed(value);
      if (DEBUG_SERIAL) {
        Serial.print("Speed: ");
        Serial.print(value);
      }
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

// ========== UTILITY FUNCTIONS ==========
bool every(unsigned long &last, unsigned long interval) {
  unsigned long now = millis();
  if (now - last >= interval) {
    last = now;
    return true;
  }
  return false;
}