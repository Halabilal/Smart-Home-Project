#define STEPPER_PIN_1 2
#define STEPPER_PIN_2 3
#define STEPPER_PIN_3 4
#define STEPPER_PIN_4 5
#define RAIN_SENSOR_PIN A0
#define THRESHOLD 120

int step_number = 0;
bool isRaining = false;

// Define the pins for the second code
#define trigpin 7  
#define echopin 6 
#define buzzer 8  // Define the buzzer pin

int led1 = A1; 
int led2 = A2; 
int led3 = A3; 
int led4 = A4; 
int led5 = A5; 

int solenoidValvePin = 9;  // Pin connected to the solenoid valve
int pumpPin = 10;          // Pin connected to the electric pump
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 12

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

 float Celcius=0;
 float Fahrenheit=0;

void setup() {
  // Setup for stepper motor and rain sensor
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
  
  // Setup for the second code
  Serial.begin(9600);
  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);
  pinMode(buzzer, OUTPUT);  // Set the buzzer pin as an output
  pinMode(led1, OUTPUT); 
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  pinMode(solenoidValvePin, OUTPUT);  // Set the solenoid valve pin as an output
  pinMode(pumpPin, OUTPUT);           // Set the pump pin as an output

  // Initialize all LEDs, the solenoid valve, the pump, and the buzzer to be off
  digitalWrite(led1, LOW); 
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
  digitalWrite(led5, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(solenoidValvePin, LOW);
  digitalWrite(pumpPin, LOW);
  sensors.begin();
}

void loop() {
  // Read the rain sensor value
  int rainValue = analogRead(RAIN_SENSOR_PIN);
  Serial.print("ADC value: ");
  Serial.println(rainValue);

  if (rainValue > THRESHOLD && !isRaining) {
    // Rain detected and motor is not yet moved
    moveStepper(1000, true); // Move forward 180 degrees
    isRaining = true;
    Serial.println("Rain detected, moving 180 degrees.");
    digitalWrite(solenoidValvePin, LOW);  // Turn off solenoid valve
    digitalWrite(pumpPin, LOW);           // Turn off pump
  } else if (rainValue <= THRESHOLD && isRaining) {
    // No rain detected and motor is in the moved position
    moveStepper(1000, false); // Move backward 180 degrees
    isRaining = false;
    Serial.println("No rain, returning 180 degrees.");
  }

  // Only run the second code if there is no rain
  if (!isRaining) {
    int duration, distance;
    Serial.println("Measuring distance.");
    // Send a pulse from the trigger pin
    digitalWrite(trigpin, HIGH);
    delayMicroseconds(10);  // Reduced to 10 microseconds for the pulse duration
    digitalWrite(trigpin, LOW);

    // Measure the duration of the echo pulse
    duration = pulseIn(echopin, HIGH);

    // Calculate the distance in centimeters
    distance = (duration / 2) / 29.1;

    // Print the distance to the serial monitor
    Serial.print("cm: "); 
    Serial.println(distance);

    // Control LEDs based on the measured distance
    controlLEDs(distance);

    // Control the buzzer based on the measured distance
    controlBuzzer(distance);

    // Control the solenoid valve and pump based on the measured distance
    controlSolenoidAndPump(distance);
  }
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  Fahrenheit=sensors.toFahrenheit(Celcius);
  Serial.print(" C  ");
  Serial.print(Celcius);
  Serial.print(" F  ");
  Serial.println(Fahrenheit);
  delay(1000); // Delay for a while before reading again
}

void moveStepper(int steps, bool dir) {
  for (int a = 0; a < steps; a++) {
    OneStep(dir);
    delay(2);
  }
}

void OneStep(bool dir) {
  if (dir) {
    switch (step_number) {
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    }
  } else {
    switch (step_number) {
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
    }
  }
  step_number++;
  if (step_number > 3) {
    step_number = 0;
  }
}

void controlLEDs(int distance) {
  if (distance <= 3) {
    digitalWrite(led1, HIGH); 
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
    digitalWrite(led5, HIGH); 
  } else if (distance > 3 && distance <= 6) {
    digitalWrite(led1, LOW); 
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
    digitalWrite(led5, HIGH); 
  } else if (distance > 6 && distance <= 9) {
    digitalWrite(led1, LOW); 
    digitalWrite(led2, LOW);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
    digitalWrite(led5, HIGH);
  } else if (distance > 9 && distance <= 12) {
    digitalWrite(led1, LOW); 
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, HIGH);
    digitalWrite(led5, HIGH);
  } else if (distance > 12 && distance <= 14) {
    digitalWrite(led1, LOW); 
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
    digitalWrite(led5, HIGH);
  } else if (distance >= 15) {
    digitalWrite(led1, LOW); 
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
    digitalWrite(led5, LOW);
  }
}

void controlBuzzer(int distance) {
  if (distance <= 2  || distance >= 13) {
    digitalWrite(buzzer, HIGH);  // Turn on the buzzer
  } else {
    digitalWrite(buzzer, LOW);   // Turn off the buzzer
  }
}

void controlSolenoidAndPump(int distance) {
  if (distance > 3 && distance <= 14) {  // Tank is low
    digitalWrite(pumpPin, HIGH);
    Serial.print("water pump is on  ");
    delay(5000);
    digitalWrite(solenoidValvePin, HIGH);  // Open solenoid valve
    Serial.print("solenoid Valve is on");
  } else {  // Tank is within acceptable range
    digitalWrite(solenoidValvePin, LOW);  // Close solenoid valve
    digitalWrite(pumpPin, LOW);           // Turn off the pump
    Serial.print("water pump is off  ");
    Serial.print("solenoid Valve is off");
  }
}