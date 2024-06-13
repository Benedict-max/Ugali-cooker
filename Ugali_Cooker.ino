#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define motor pins
const int potMotorPin = 9;    // Pin for pot rotation motor
const int stirMotorPin = 8;   // Pin for stirring motor

// Define buzzer pin
const int buzzerPin = 6;      // Pin for buzzer

// Define temperature sensor pin
const int tempSensorPin = A0;

// Initialize LCD (I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27 for a 16x2 display

// Timer variables
unsigned long startTime = 0;
const unsigned long cookingTime = 30000;  // 1 minute in milliseconds

bool cookingActive = false;
bool cookingCompleted = false;

void setup() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize motor and buzzer pins
  pinMode(potMotorPin, OUTPUT);
  pinMode(stirMotorPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Display initial message
  lcd.setCursor(0, 0);
  lcd.print("Waiting for");
  lcd.setCursor(0, 1);
  lcd.print("Temp >= 35C");
}

void loop() {
  if (!cookingCompleted) {
    // Read temperature from sensor
    float temperature = readTemperature();

    // Check if temperature is above 35 degrees Celsius to start motors
    if (temperature >= 35.0 && !cookingActive) {
      cookingActive = true;
      startTime = millis();  // Start the timer when cooking begins

      // Display cooking start message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Cooking Starting");
      delay(2000);  // Wait for 2 seconds to show the message
    }

    // If cooking is active, control motors and display status
    if (cookingActive) {
      // Start rotating the pot
      rotatePotMotor();

      // Start stirring the ugali
      stirUgali();

      // Display cooking status
      displayStatus(temperature);

      // Check if the cooking time has elapsed
      if (millis() - startTime >= cookingTime) {
        stopMotors();
        activateBuzzer();
        cookingActive = false;  // Stop motor control
        cookingCompleted = true; // Mark the cooking process as completed

        // Display cooking end message
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Cooking Ended");
        lcd.setCursor(0, 1);
        lcd.print("Enjoy your meal");
      }
    } else {
      // Display current temperature while waiting to start
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature, 1);  // Temperature with 1 decimal place
      lcd.print(" C    ");
    }
  }

  // Add a small delay to avoid excessive updates
  delay(500);
}

float readTemperature() {
  // Read analog value from LM35 sensor
  int sensorValue = analogRead(tempSensorPin);

  // Convert analog value to temperature in Celsius
  float temperature = (sensorValue * 5.0 / 1024.0) / 0.01;

  return temperature;
}

void rotatePotMotor() {
  // Example: Rotate pot motor clockwise
  digitalWrite(potMotorPin, HIGH);
  // Implement motor speed control using PWM if needed
}

void stirUgali() {
  // Example: Start stirring motor
  digitalWrite(stirMotorPin, HIGH);
  // Implement stirring pattern or algorithm
}

void stopMotors() {
  // Stop all motors
  digitalWrite(potMotorPin, LOW);
  digitalWrite(stirMotorPin, LOW);
}

void activateBuzzer() {
  // Activate the buzzer to indicate cooking is complete
  digitalWrite(buzzerPin, HIGH);
  delay(2000); // Buzzer on for 2 seconds
  digitalWrite(buzzerPin, LOW);
}

void displayStatus(float temperature) {
  // Calculate time elapsed
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  
  // Calculate remaining time in seconds
  int remainingTime = (cookingTime - elapsedTime) / 1000;

  // Calculate percentage progress
  int progress = (elapsedTime * 100) / cookingTime;

  // Display titles and values
  lcd.setCursor(0, 0);
  lcd.print("Temp:"); 
  lcd.setCursor(6, 0);
  lcd.print("Time:"); 
  lcd.setCursor(12, 0);
  lcd.print("%:");

  lcd.setCursor(0, 1);
  lcd.print(temperature, 1);  // Temperature with 1 decimal place
  lcd.print("C ");

  lcd.setCursor(6, 1);
  lcd.print(remainingTime);
  lcd.print("s ");

  lcd.setCursor(12, 1);
  lcd.print(progress);
  lcd.print("% ");
}
