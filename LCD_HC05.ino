#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
const int rs = 7;
const int en = 8;
const int d4 = 9;
const int d5 = 10;
const int d6 = 11;
const int d7 = 12;

String newText;
String oldText;
String receivedText;

LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

SoftwareSerial BTSerial(3, 2); // RX, TX

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600);
    lcd.begin(16,2);
    lcd.clear();
}

void loop() {
  if (BTSerial.available()) {
    receivedText = BTSerial.readStringUntil('\n'); // Read incoming string
    receivedText.trim();
    Serial.println(receivedText);
    newText = receivedText;
  }

  if (newText != oldText) {
    // Split the received string into two parts
    int separatorIndex = receivedText.indexOf(';'); // Check for separator ';'
    if (separatorIndex != -1) {
      // If separator is found, split into two rows
      String firstRowText = receivedText.substring(0, separatorIndex);
      Serial.println(firstRowText);
      String secondRowText = receivedText.substring(separatorIndex + 1);
      Serial.println(secondRowText);

      // Update the LCD
      lcd.setCursor(0, 0); // First row
      lcd.print("                "); // Clear the row (16 spaces)
      lcd.setCursor(0, 0);
      lcd.print(firstRowText);

      lcd.setCursor(0, 1); // Second row
      lcd.print("                "); // Clear the row (16 spaces)
      lcd.setCursor(0, 1);
      lcd.print(secondRowText);
    } 
    else {
      // If no separator, update only the first row
      lcd.clear();
      lcd.setCursor(0, 0); // First row
      lcd.print("                "); // Clear the row (16 spaces)
      lcd.setCursor(0, 0);
      lcd.print(receivedText);
    }
    oldText = newText;
  }
  else{
    Serial.println("Same");
  }
}
