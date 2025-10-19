#include <Servo.h>

#define MIN_PULSE_LENGTH 1000   // Full Reverse
#define MID_PULSE_LENGTH 1500   // Neutral / Stop
#define MAX_PULSE_LENGTH 2000   // Full Forward

Servo motA;
char data;

void setup() {
    Serial.begin(9600);
    motA.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    displayInstructions();
}

void loop() {
    if (Serial.available()) {
        data = Serial.read();
        switch (data) {
            case '0':  // Full reverse
                Serial.println("Sending FULL REVERSE (1000 µs)");
                motA.writeMicroseconds(MIN_PULSE_LENGTH);
                break;

            case '1':  // Neutral
                Serial.println("Sending NEUTRAL / STOP (1500 µs)");
                motA.writeMicroseconds(MID_PULSE_LENGTH);
                break;

            case '2':  // Full forward
                Serial.println("Sending FULL FORWARD (2000 µs)");
                motA.writeMicroseconds(MAX_PULSE_LENGTH);
                break;

            case '3':  // Test function
                Serial.print("Running bidirectional sweep in 3");
                delay(1000);
                Serial.print(" 2");
                delay(1000);
                Serial.println(" 1...");
                delay(1000);
                test();
                break;

            default:
                Serial.println("Invalid command. Use 0–3 as described below.");
                displayInstructions();
                break;
        }
    }
}

void test() {
    Serial.println("Starting full bidirectional test...");

    // Sweep forward
    for (int i = MID_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 10) {
        Serial.print("Forward pulse = ");
        Serial.println(i);
        motA.writeMicroseconds(i);
        delay(150);
    }

    // Pause briefly at max
    delay(1000);

    // Return to neutral
    Serial.println("Neutral (Stop)");
    motA.writeMicroseconds(MID_PULSE_LENGTH);
    delay(1000);

    // Sweep reverse
    for (int i = MID_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 10) {
        Serial.print("Reverse pulse = ");
        Serial.println(i);
        motA.writeMicroseconds(i);
        delay(150);
    }

    // Stop again at neutral
    Serial.println("Test complete. Motor stopped.");
    motA.writeMicroseconds(MID_PULSE_LENGTH);
}

void displayInstructions() {
    Serial.println("\nREADY - SEND COMMANDS AS FOLLOWS:");
    Serial.println("\t'0' : Send FULL REVERSE (1000 µs)");
    Serial.println("\t'1' : Send NEUTRAL / STOP (1500 µs)");
    Serial.println("\t'2' : Send FULL FORWARD (2000 µs)");
    Serial.println("\t'3' : Run full bidirectional sweep test\n");
}
