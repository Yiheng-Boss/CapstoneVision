#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX, TX

const int outputPin = 3; // Pin to control
const int outputPinLEDR = 4;
const int outputPinLEDG = 5;


void setup() {
    Serial.begin(9600);         // Communication with Serial Monitor
    BTSerial.begin(9600);      // Communication with HC-05
    pinMode(outputPin, OUTPUT); // Set pin 12 as output
    pinMode(outputPinLEDR, OUTPUT); 
    pinMode(outputPinLEDG, OUTPUT); 
    digitalWrite(outputPin, HIGH); // Ensure pin 12 starts LOW
    digitalWrite(outputPinLEDR, LOW);
    digitalWrite(outputPinLEDG, LOW);
    Serial.println("Slave ready to receive RF character");
}

void loop() {
    if (BTSerial.available()) {    // Check if data is received from Master
    Serial.println(BTSerial.available());
        char received = BTSerial.read();
        Serial.println("Received: " + String(received)); // Feedback to Serial Monitor
        
        if (received == 'H') {    // Check for the RF character (e.g., 'H')
            delay(3000);
            digitalWrite(outputPin, LOW);
            digitalWrite(outputPinLEDR, HIGH); // Set pin 12 HIGH
            Serial.println("Pin 3 set HIGH");
            Serial.println("LED RED set HIGH");
        } 
        else if (received == 'L') { // Optional: Reset pin 12 to LOW
            digitalWrite(outputPin, HIGH);
            Serial.println("Pin 3 set LOW");
        }
        else if (received == 'F') { // Optional: Reset pin 12 to LOW
            digitalWrite(outputPinLEDG, HIGH);
            Serial.println("LED GREEN set HIGH");
            digitalWrite(outputPinLEDR, LOW);
            Serial.println("LED RED set LOW");
            delay(5000);
            digitalWrite(outputPinLEDG, LOW);

        }
    }
}
