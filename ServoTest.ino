#include <Servo.h>

// These variables will be adjusted to ESC' pulse range
const int PULSE_MIN = 1000; // Reverse Max
const int PULSE_NEUTRAL = 1500; // Stop
const int PULSE_MAX = 2000; // Forward Max

Servo esc;
Servo servo;

const byte ESC_PIN = 9; // ESC to control the brushless motor
const byte SERVO_PIN = 10; // Servo to steer the wheels

// Parameters to use in every() function.
unsigned long intervalSensors = 20; // interval for sensors to check their surroundings
unsigned long lastSensors = 0; // last time the sensors process run

//Forward declaration for functions
bool every(unsigned long &last, const unsigned long interval);

void setup() {
  // put your setup code here, to run once:
  esc.attach(ESC_PIN);
  servo.attach(SERVO_PIN);
  esc.writeMicroseconds(PULSE_NEUTRAL); // start arming procedure
  delay(5000); // wait 5 seconds to wait for arming procedure to finish
}

void loop() {
  // put your main code here, to run repeatedly:

}

// Generic "every" helper
//
// This function grants easy multiprocessing for different tast that will
// be written loop().
//
// Declare an interval and a last value for every operation you write. Example:
//
//  unsigned int intervalSensors = 20; // interval for sensors to check their surroundings
//  unsigned long lastSensors = 0; // last time the sensor code ran
//
// -Usage in loop():
// if (every(lastServo, intervalServo)) updateServo();
// if (every(lastSensors, intervalSensors)) readSensors();
// if (every(lastLog, intervalLog)) logData();
//
bool every(unsigned long &last, unsigned long &interval) {
  unsigned long now = millis();
  if (now - last >= interval) { 
    last += interval; 
    return true; 
  }
  return false;
}