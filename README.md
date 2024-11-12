# Emotion and Gesture-Driven Smart Home System with Advanced Environmental Controls
## *This README provides an overview of the project. For more in-depth information, refer to the full project documentation available in this repository.*
## Overview
**This project presents a comprehensive smart home automation system that uses emotion and gesture recognition to control home environmental elements, enhancing user convenience, comfort, and well-being. The system integrates facial and speech emotion recognition, gesture-based controls, and environmental monitoring to manage lighting, ventilation, and water systems dynamically.**

## Project Description
**The system combines emotion detection (from facial and vocal cues), gesture recognition, and environmental sensors to create a responsive home environment. Key functionalities include:**
##  •	Emotion-driven LED lighting.
### •	Gesture-controlled fan speed and direction.
### •	Automated water tank management with rainwater collection.
### •	Energy-efficient operation with a passive infrared (PIR) motion sensor.

## Features
### 1.	Emotion-Based Light Control: Changes LED lighting based on user emotions detected through facial and speech analysis.
### 2.	Gesture-Controlled Fan: Adjusts fan speed and direction using hand gestures, leveraging a machine learning model.
### 3.	Environmental Monitoring: Temperature and humidity regulation using a PID-controlled fan.
### 4.	Automated Water Tank System: Monitors water levels, controls refilling, and collects rainwater.
### 5.	Motion Sensor Integration: Activates the system only when movement is detected, optimizing energy usage.
   
## Technologies Used
### •	Microcontroller: Arduino Mega for core system control.
### •	Sensors: DHT11, MPU6050, DS18B20, Ultrasonic, PIR.
### •	Machine Learning: TensorFlow and Keras for emotion and gesture recognition.
### •	Programming Languages: Python (for ML models) and Arduino C++.
### •	Development Environment: Spyder IDE, jupyter notebook, Arduino IDE.

## Hardware Components
### •	Arduino Mega: Primary control board.
### •	DHT11 Sensor: For temperature and humidity monitoring.
### •	MPU6050 Sensor: Controls fan direction based on user’s position.
### •	Servo Motor: Adjusts fan direction.
### •	L293D Motor Driver: Controls fan speed.
### •	IR Receiver: Enables fan speed control via remote.
### •	Ultrasonic Sensor: Measures water levels.
### •	DS18B20 Sensor: Measures water temperature.
### •	Rain Sensor: Detects rain and opens tank cover.
### •	RGB LED, Buzzer, LCD: Provides visual/auditory feedback on system status.

## Software Components
### •	Emotion Detection Algorithms: Facial and vocal emotion detection models trained using FER-2013, RAVDESS, and other datasets.
### •	Gesture Recognition Model: Processes hand gestures based on distance and angle between fingers.
### •	PID Control Algorithm: Regulates fan speed based on real-time temperature and humidity.
### •	User Interface: Integrates remote control, LCD, and LEDs for user feedback.

## System Architecture
### The system consists of:
#### •	Arduino Mega as the central controller.
#### •	Spyder-based machine learning models for emotion and gesture recognition, interacting with Arduino via serial communication.
#### •	Environmental sensors and actuators (fan, lights, water tank) interconnected for optimal environmental control.

## Methodology
### **Data Collection**
#### 1.	Facial Emotion Recognition: Used the FER-2013 dataset, containing 35,887 grayscale images across seven emotion categories (angry, disgusted, fearful, happy, neutral, sad, surprised).
#### 2.	Speech Emotion Recognition: Audio datasets RAVDESS, CREMA-D, TESS, and SAVEE were used to train models to recognize emotions such as happiness, sadness, anger, fear, disgust, and neutrality.
#### 3.	Combined Modal Data: For combined emotion recognition, both real-time and pre-recorded datasets (e.g., RAVDESS) were used to provide inputs for training and testing.
#### 4.	Gesture Detection: Real-time hand gesture data was collected using a webcam feed and processed using MediaPipe for hand landmark tracking.
### **Data Preprocessing**
#### 1.	Image Preprocessing: Images from FER-2013 were rescaled, converted to grayscale, and augmented to increase diversity, reducing the risk of overfitting.
#### 2.	Audio Preprocessing: Audio data was processed with techniques such as noise addition, time-stretching, and feature extraction using Mel-frequency cepstral coefficients (MFCCs) and other audio features.
#### 3.	Combined Modal Data: Feature extraction from audio data and face detection from video frames were applied, followed by encoding and normalization.
#### 4.	Gesture Data: Key landmarks were tracked, and distance and angle calculations between thumb and index fingers were performed for gesture-based fan control.
### **System Implementation**
#### 1.	Facial Emotion Recognition: Implemented a convolutional neural network (CNN) using Keras, trained on the FER-2013 dataset, and tested for real-time performance via webcam.
#### 2.	Speech Emotion Recognition: Developed a CNN for emotion recognition in audio using TensorFlow and Keras, with augmentation techniques applied for robustness.
#### 3.	Combined Modal Model: Combined predictions from both speech and facial emotion models, allowing the system to use matched predictions for increased accuracy.
#### 4.	Gesture Detection System: Real-time gesture detection was implemented using MediaPipe, calculating distances and angles between finger landmarks to control fan settings.
   
## Usage
### 1.	Emotion Detection:
#### o	Runs via webcam and microphone to detect user emotions, adjusting LED lighting accordingly.
### 2.	Gesture Control:
#### o	Uses webcam to capture gestures, adjusting fan speed and direction based on detected hand movements.
### 3.	Environmental Monitoring:
#### o	Automatically adjusts fan settings based on real-time temperature and humidity data.
### 4.	Water Tank System:
#### o	Automatically refills and manages water levels, displaying status on the LCD and alerting via buzzer if levels are critical.

## Results
### 1.	PID Control of Fan Speed: Achieved stable control using constants Kp = 40.0, Ki = 0.01, and Kd = 1.0 to maintain optimal temperature and humidity levels.
### 2.	Servo and MPU6050 Integration: Resolved library conflicts, enabling smooth directional control of the fan based on sensor inputs.
### 3.	IR Receiver and Remote Integration: Addressed timer conflicts, enabling seamless control of fan settings via remote.
### 4.	LED Emotion Detection: Successfully adjusted LED colors based on emotion, combining data from facial and speech recognition models.
### 5.	Water Tank System: Efficiently managed water levels with automated refilling and rainwater collection, with visual/auditory alerts.
### 6.	Gesture-Based Fan Control: Enabled smooth and responsive fan control via gestures, with unique signal generation to prevent interference with emotion detection.
   
## Limitations and Challenges
### •	Inter-device Conflicts: Conflicts were encountered with library dependencies when integrating multiple components (Servo, IR).
### •	Environmental Factors: Emotion and gesture detection accuracy may vary with lighting and noise levels.
### •	Real-time Constraints: The system experiences minor delays in response to motion and remote control signals.

