#include <Servo.h>


#define SERVO_PIN 3 

Servo direksiyonServo; 
int gelenAci = 90; 

void setup() {

  direksiyonServo.attach(SERVO_PIN);
  

  Serial.begin(9600);
  Serial.println("--- SERVO TEST MODU BASLADI ---");
  Serial.println("Lutfen 0 ile 180 arasinda bir aci (derece) girin.");
  

  direksiyonServo.write(90);
  Serial.println("Baslangic pozisyonu: 90 derece.");
}


void loop() {
  

  if (Serial.available() > 0) {
    

    gelenAci = Serial.parseInt(); 
    

    if (gelenAci >= 0 && gelenAci <= 180) {
      
  
      direksiyonServo.write(gelenAci); 
      
      Serial.print("Servo gonderilen aci: ");
      Serial.println(gelenAci);

    } else {
      Serial.println("Gecersiz Aci Degeri! Lutfen 0-180 araliginda bir sayi girin.");
    }
  }
}