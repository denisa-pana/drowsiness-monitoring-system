#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000

PulseOximeter pox;

uint32_t tsLastReport = 0;

int buzzerPin = 13;
int green = 10;
int yellow = 11;
int red = 12;
String info = "";

int tiredLevel = 0;
bool tired = false;

LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD with the correct address

void setup() {
  Serial.begin(9600);

  Serial.print("Initializing pulse oximeter..");
  if (!pox.begin()) {
      Serial.println("FAILED");
      for(;;);
  } else {
      Serial.println("SUCCESS");
  }
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // Register a callback for the beat detection
  pox.setOnBeatDetectedCallback(onBeatDetected);

  lcd.init();                 //Init the LCD
  lcd.backlight();            //Activate backlight     
  lcd.home(); 
  lcd.clear();

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(red, OUTPUT);
}

void onBeatDetected() 
{
    Serial.println("Beat!");
}

void tiredAlarm() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("You are tired!");
  lcd.setCursor(0, 1);
  lcd.print("Take a break!");
  for (int i = 0; i <= 3; i++) {
    digitalWrite(buzzerPin, LOW);
    delay(1000);
    digitalWrite(buzzerPin, HIGH);
    delay(500);
  }
  tired = false;
}

void loop() {
  pox.update();
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    if(pox.getSpO2() > 0) {
      lcd.setCursor(0, 1); // Set cursor position
      info = String(pox.getHeartRate()) + "bpm - " + String(pox.getSpO2()) + "%";
      lcd.print(info); // Print to LCD
      Serial.println(info);

      if(pox.getHeartRate() < 30 || pox.getSpO2() < 80) {
        tiredLevel = 1;
      }

    } else {
      lcd.setCursor(0, 1); // Set cursor position
      lcd.print("Touch the contact"); // Print to LCD
    }
    tsLastReport = millis();
  }

  if (Serial.available() > 0) {
    int signal = Serial.parseInt();
    if (signal == 2) {
      tired = true;
      tiredLevel = 2;
    }
  }
  if (tired) {
    tiredAlarm();
  }
  if (tiredLevel == 0) {
    digitalWrite(green, HIGH);
    digitalWrite(yellow, LOW);
    digitalWrite(red, LOW);
  } else if (tiredLevel == 1) {
    digitalWrite(green, HIGH);
    digitalWrite(yellow, HIGH);
    digitalWrite(red, LOW);
  } else {
    digitalWrite(green, HIGH);
    digitalWrite(yellow, HIGH);
    digitalWrite(red, HIGH);
  }
}