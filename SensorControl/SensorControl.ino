const int TRIG1 = 2;
const int ECHO1 = 3;
const int TRIG2 = 4;
const int ECHO2 = 5;
const int TRIG3 = 6;
const int ECHO3 = 7;
const int TRIG4 = 8;
const int ECHO4 = 9;

const unsigned long READ_INTERVAL = 250;
const int SAMPLES = 9;
const unsigned long TIMEOUT = 30000UL;
const int INTER_SENSOR_DELAY = 100;

float TEMPERATURE_C = 23.0;
float CALIB_OFFSET_CM = 0.0;
float CALIB_SCALE = 1.0;

unsigned long lastRead = 0;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG1, OUTPUT); digitalWrite(TRIG1, LOW);
  pinMode(TRIG2, OUTPUT); digitalWrite(TRIG2, LOW);
  pinMode(TRIG3, OUTPUT); digitalWrite(TRIG3, LOW);
  pinMode(TRIG4, OUTPUT); digitalWrite(TRIG4, LOW);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(ECHO4, INPUT);
  delay(50);
  Serial.println("HC-SR04 4-sensor median+temp cal test basladi");
}

void loop() {
  if (millis() - lastRead < READ_INTERVAL) return;
  lastRead = millis();

  float d1 = readSensorMedianCm(TRIG1, ECHO1, SAMPLES);
  delay(INTER_SENSOR_DELAY);
  float d2 = readSensorMedianCm(TRIG2, ECHO2, SAMPLES);
  delay(INTER_SENSOR_DELAY);
  float d3 = readSensorMedianCm(TRIG3, ECHO3, SAMPLES);
  delay(INTER_SENSOR_DELAY);
  float d4 = readSensorMedianCm(TRIG4, ECHO4, SAMPLES);

  Serial.print("S1: "); printDistance(d1);
  Serial.print("  |  S2: "); printDistance(d2);
  Serial.print("  |  S3: "); printDistance(d3);
  Serial.print("  |  S4: "); printDistance(d4);
  Serial.println();

  float dangerThreshold = 20.0;
  if ((d1>0 && d1<dangerThreshold) || (d2>0 && d2<dangerThreshold) || (d3>0 && d3<dangerThreshold) || (d4>0 && d4<dangerThreshold)) {
    Serial.println("!! OBSTACLE CLOSE !!");
  }
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

float durationToCmWithTemp(unsigned long duration) {
  if (duration == 0) return -1.0;
  float speed_m_s = 331.4 + 0.6 * TEMPERATURE_C;
  float speed_cm_per_us = speed_m_s * 0.0001;
  float dist = duration * speed_cm_per_us * 0.5;
  dist = dist * CALIB_SCALE + CALIB_OFFSET_CM;
  if (dist <= 0) return -1.0;
  return dist;
}

float readSensorMedianCm(int trigPin, int echoPin, int samples) {
  unsigned long arr[31];
  if (samples > 31) samples = 31;
  for (int i = 0; i < samples; i++) {
    arr[i] = singlePulseDuration(trigPin, echoPin);
    delay(20);
  }
  int n = samples;
  for (int i = 0; i < n-1; i++) {
    for (int j = i+1; j < n; j++) {
      if (arr[j] < arr[i]) {
        unsigned long t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }
  unsigned long medianDur;
  if (n % 2 == 1) medianDur = arr[n/2];
  else medianDur = (arr[n/2 - 1] + arr[n/2]) / 2;
  return durationToCmWithTemp(medianDur);
}

void printDistance(float d) {
  if (d < 0) Serial.print("HATA");
  else {
    Serial.print(d, 1);
    Serial.print(" cm");
  }
}
