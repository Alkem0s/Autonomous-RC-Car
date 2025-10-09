#include <Servo.h>

Servo esc;
int escPin = 9;

// HobbyKing ESC'ler genelde 1000–2000 µs aralığında çalışır.
const int minPulse = 1000;
const int maxPulse = 2000;

void setup() {
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

void loop() {
  // ESC'yi test etmek için basit bir örnek:
  int val = map(analogRead(A0), 0, 1023, minPulse, maxPulse);
  esc.writeMicroseconds(val);
}
