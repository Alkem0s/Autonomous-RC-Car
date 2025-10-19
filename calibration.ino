#include <Servo.h>

Servo esc;
int escPin = 9;
Servo servo;
int servoPin = 10;

// HobbyKing ESC'ler genelde 1000–2000 µs aralığında çalışır.
const int minPulse = 1000;
const int maxPulse = 2000;

void calibrateESC() {
  // ESC kalibrasyon prosedürü
  Serial.begin(9600);
  esc.attach(escPin, minPulse, maxPulse);

  Serial.println("ESC Kalibrasyonu Basliyor!");
  Serial.println("Batarya TAKILI OLMASIN!");
  Serial.println("Hazir olunca Enter'a bas...");

  while (!Serial.available()); // kullanıcıdan input bekle

  Serial.read(); // buffer temizle

  // Kullanıcı hazırsa:
  Serial.println("Gaz maksimumda olmalı (2000 µs). ESC takılınca bip'leri dinle!");

  esc.writeMicroseconds(maxPulse);
  delay(2000);

  Serial.println("Simdi bataryayi tak ve bip seslerini dinle...");
  delay(5000); // ESC'nin tanıması için süre

  Serial.println("Simdi gazı minimuma çekiyoruz (1000 µs)...");
  esc.writeMicroseconds(minPulse);
  delay(4000);

  Serial.println("Kalibrasyon tamamlandı!");
  Serial.println("ESC artik normal calisma modunda.");
}

void setupServo() {
  // Servo'yu başlat
  servo.attach(servoPin);
}

void setupBluetooth() {
  // Bluetooth modülünü başlat
  // Serial1 ---> 19(RX1)/18(TX1) pinlerini kullanır
  Serial1.begin(9600);
}

void readBluetooth(int pulseValue) {
  // Bluetooth'tan gelen veriyi oku
  if (Serial1.available()){
    char command = Serial1.read();
    switch (command) {
      case 'F': // ileri
        esc.writeMicroseconds(pulseValue);
        break;
      case 'B': // geri
        esc.writeMicroseconds(pulseValue);
        break;
      case 'S': // dur
        esc.writeMicroseconds((minPulse + maxPulse) / 2);
        break;
      case 'L': // sola dön
        servo.write(45); // sola dönüş açısı
        break;
      case 'R': // sağa dön
        servo.write(135); // sağa dönüş açısı
        break;
      case 'C': // düzelt
        servo.write(90); // düz pozisyon
        break;
      default:
        Serial.println("Bilinmeyen komyut!");
        break;
    }

    // TODO: Buraya sonrası için hız kontrolü eklenecek

  }
}

void setup() {
  calibrateESC(); 
  setupServo();
  setupBluetooth();
}

void loop() {
  // ESC'yi test etmek için basit bir örnek:
  int val = map(analogRead(A0), 0, 1023, minPulse, maxPulse);
  esc.writeMicroseconds(val);
  //readBluetooth(val); ---> Bluetooth kontrolü için
}