#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <DHT.h>
#include <LiquidCrystal.h>
#include <IRremote.hpp>

#define RECEIVER_PIN 10 // Pin the receiver is plugged into
// Emotion-based light control and PIR setup
#define RED_PIN 7
#define GREEN_PIN 6
#define BLUE_PIN 5
#define PIR_PIN 38
#define DHTPIN A3
#define DHTTYPE DHT11
// Motor driver setup
#define EN1 2
#define IN1 34
#define IN2 36

// Servo and MPU6050 setup
Servo sg90;
int servo_pin = 3;
MPU6050 sensor;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// LCD and DHT11 setup
LiquidCrystal lcd(32, 30, 28, 26, 24, 22);
DHT dht(DHTPIN, DHTTYPE);

// PID constants
float Kp = 40.0;
float Ki = 0.01;
float Kd = 1.0;

// PID variables
float setPointTemp = 25.0;
float setPointHumidity = 30.0;
float error, prevError = 0;
float integral = 0;
float derivative;
unsigned long lastTime = 0;
unsigned long sampleTime = 1000;  // Sample time in milliseconds
float integralLimit = 100;  // Reduced limit for integral wind-up

// User preference storage
struct Preference {
  float temperature;
  float humidity;
  int fanSpeed;
};

Preference preferences[10];  // Store up to 10 preferences
int preferenceCount = 0;

int speed = 0; // Declare speed as a global variable
unsigned long handDetectedTime = 0;
bool handControlMode = false; // State variable for hand control mode

int pirValue; // Place to store read PIR Value
unsigned long lastMotionTime = 0; // Time of the last detected motion
const unsigned long motionTimeout = 5000; // 5-second timeout
bool motionDetected = false;
char currentEmotion = '7'; // Default to 'Mismatch' or white color


void setup() {
  Serial.begin(9600);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  setColor(0, 0, 0); // Initialize with LED off
  Serial.println("Setup complete");

  // Setup for Servo and MPU6050
  sg90.attach(servo_pin);
  Wire.begin();
  Serial.println("Initializing the sensor");
  sensor.initialize();
  Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed");
  delay(1000);
  Serial.println("Taking Values from the sensor");
  delay(1000);

  // Setup for LCD and DHT11
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  dht.begin();

  // Setup for motor driver
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Setup for IR receiver
  IrReceiver.begin(RECEIVER_PIN, ENABLE_LED_FEEDBACK);

  // Wait for the sensor to stabilize
  delay(2000);
  lcd.clear();
}

void loop() {
  pirValue = digitalRead(PIR_PIN);

  if (pirValue == HIGH) {
    lastMotionTime = millis();
    if (!motionDetected) {
      Serial.println("Motion detected");
      setColor(255, 255, 255); // Set LED to white when motion is first detected
      Serial.println("MOTION"); // Send motion signal to Python
      motionDetected = true;
    }
  }


  if (Serial.available() > 0) {
    String receivedEmotion = Serial.readStringUntil('\n');
    receivedEmotion.trim(); // Trim any whitespace or newline characters
    Serial.print("Received: ");
    Serial.println(receivedEmotion);
    setColorBasedOnEmotion(receivedEmotion); // Set LED color based on the received emotion
  }

  if (millis() - lastMotionTime > motionTimeout) {
    setColor(0, 0, 0); // Turn off the LED if no motion detected for 5 seconds
    motionDetected = false;
    // Set the fan speed to 100 if no motion is detected
    speed = 100;
    analogWrite(EN1, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    lcd.setCursor(9, 1);  // Update LCD with the adjusted speed
    lcd.print("Fan:");
    lcd.print(speed);
  }

  // Check for hand detection message
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int delimiterIndex = data.indexOf(',');
    if (delimiterIndex != -1) {
      int distance = data.substring(0, delimiterIndex).toInt();
      int angle = data.substring(delimiterIndex + 1).toInt();

      if (distance < 0) {
        handControlMode = false;
        handDetectedTime = millis();  // Record the time when the hand was last detected
      } else {
        handControlMode = true;
        handDetectedTime = millis();  // Reset the hand detection time

        // Control fan speed with L293D directly from distance
        int fanSpeed = (distance > 255) ? 255 : distance;
        analogWrite(EN1, fanSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        // Control servo motor
        int servoAngle = map(angle, -180, 0, 0, 180); // Map -180 to 0 to 0 to 180
        sg90.write(servoAngle);

        // Update the LCD with the new speed
        lcd.setCursor(9, 1);  // Move the cursor to the appropriate position
        lcd.print("Fan:");
        lcd.print(fanSpeed);
      }
    }
  }

  // Switch to PID control if hand not detected for more than 5 seconds
  if (millis() - handDetectedTime > 5000) {
    handControlMode = false;
  }
    // Read humidity and temperature from DHT11
    float h = dht.readHumidity();
    float t = dht.readTemperature();
  // PID control mode
  if (!handControlMode && motionDetected) {
    // MPU6050 reading and servo control
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int servoPosition = map(ax, -17000, 17000, 0, 180); // Adjusted for Servo angle
    sg90.write(servoPosition);
    Serial.println(servoPosition);
    delay(200);

    // Read humidity and temperature from DHT11
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again)
    if (isnan(h) || isnan(t)) {
      lcd.setCursor(0, 0);
      lcd.print("Failed to read");
      lcd.setCursor(0, 1);
      lcd.print("from sensor!");
      return;
    }

    // Get the current time
    unsigned long now = millis();
    if (now - lastTime >= sampleTime) {
      // Calculate temperature and humidity errors
      float tempError = t - setPointTemp;
      float humidityError = h - setPointHumidity;
      error = tempError * 0.8 + humidityError * 0.2;

      // Calculate integral with wind-up guard
      integral += error * (now - lastTime);
      integral = constrain(integral, -integralLimit, integralLimit);

      // Calculate derivative
      derivative = (error - prevError) / (now - lastTime);

      // Compute PID output
      float output = Kp * error + Ki * integral + Kd * derivative;

      // Constrain the PID output to PWM range (0-255)
      speed = constrain(output, 0, 255);

      // Ensure minimum fan speed
      if (speed < 50 && (tempError > 0 || humidityError > 0)) {
        speed = 50;
      }

      // Debug output
      Serial.print("Temperature: ");
      Serial.print(t);
      Serial.print(" C, Humidity: ");
      Serial.print(h);
      Serial.print(" %, Error: ");
      Serial.print(error);
      Serial.print(", Integral: ");
      Serial.print(integral);
      Serial.print(", Derivative: ");
      Serial.print(derivative);
      Serial.print(", Output: ");
      Serial.print(output);
      Serial.print(", Speed: ");
      Serial.println(speed);

      // Update previous error and last time
      prevError = error;
      lastTime = now;

      // Control the motor
      analogWrite(EN1, speed);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      // Print temperature, humidity, and fan speed to the LCD
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(t);
      lcd.print(" C");

      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print((int)h);  // Cast humidity to an integer
      lcd.print("%");

      lcd.setCursor(9, 1);  // Move the cursor to a new position to avoid overwriting
      lcd.print("Fan:");
      lcd.print(speed);
    }

    // Check if current conditions match any stored preferences
    bool preferenceApplied = false;
    for (int i = 0; i < preferenceCount; i++) {
      if (abs(preferences[i].temperature - t) < 1 && abs(preferences[i].humidity - h) < 5) {
        speed = preferences[i].fanSpeed;
        analogWrite(EN1, speed);
        Serial.print("Applying saved preference: ");
        Serial.println(speed);
        lcd.setCursor(9, 1);  // Update LCD with the adjusted speed
        lcd.print("Fan:");
        lcd.print(speed);
        preferenceApplied = true;
        break;
      }
    }

    if (!preferenceApplied) {
      // If no preference is applied, continue with the PID control
      analogWrite(EN1, speed);
      lcd.setCursor(9, 1);  // Update LCD with the PID controlled speed
      lcd.print("Fan:");
      lcd.print(speed);
    }

    // IR remote handling
    if (IrReceiver.decode()) {
      int fanSpeedAdjustment = 0;
      uint16_t command = IrReceiver.decodedIRData.command;
      static uint16_t lastCommand = 0; // Store the last non-repeat command

      if (command == 0xFFFFFFFF) { // Repeat code
        command = lastCommand; // Use the last non-repeat command
      } else {
        lastCommand = command; // Update the last non-repeat command
      }

      switch (command) {
        case 69: // ON/OFF
          fanSpeedAdjustment = 0;
          break;
        case 70: // +VOL
          fanSpeedAdjustment = 10;
          break;
        case 21: // -VOL
          fanSpeedAdjustment = -10;
          break;
        case 9: // UP
          fanSpeedAdjustment = 20;
          break;
        case 7: // DOWN
          fanSpeedAdjustment = -20;
          break;
        case 13: // ST/REPT - Save current preferences
          preferenceCount = 0;
          Serial.println("User preferences reset.");
          break;
      }

      if (fanSpeedAdjustment != 0) {
        speed = constrain(speed + fanSpeedAdjustment, 0, 255);
        analogWrite(EN1, speed);
        lcd.setCursor(9, 1);  // Update LCD with the adjusted speed
        lcd.print("Fan:");
        lcd.print(speed);

        // Save the updated speed to preferences if within tolerance range
        bool preferenceFound = false;
        for (int i = 0; i < preferenceCount; i++) {
          if (abs(preferences[i].temperature - t) < 1 && abs(preferences[i].humidity - h) < 5) {
            preferences[i].fanSpeed = speed;
            preferenceFound = true;
            break;
          }
        }

        if (!preferenceFound && preferenceCount < 10) {
          preferences[preferenceCount].temperature = t;
          preferences[preferenceCount].humidity = h;
          preferences[preferenceCount].fanSpeed = speed;
          preferenceCount++;
        }
      }
      IrReceiver.resume();
    }
  }
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

void setColorBasedOnEmotion(String emotion) {
  if (emotion == "angry") {
    setColor(50, 50, 255); // Blue
  } else if (emotion == "disgusted") {
    setColor(64, 224, 208); // Turquoise
  } else if (emotion == "fear") {
    setColor(70, 255, 70); // Light green
  } else if (emotion == "happy") {
    setColor(255, 50, 50); // Pink
  } else if (emotion == "neutral") {
    setColor(192, 192, 192); // Light Gray
  } else if (emotion == "sad") {
    setColor(255, 255, 30); // Orange
  } else if (emotion == "surprised") {
    setColor(128, 40, 128); // Pink
  } else if (emotion == "Mismatch") {
    setColor(255, 255, 255); // White
  } else {
    setColor(255, 255, 255); // Default to White
  }
}




