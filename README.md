🏙️ Smart Home IoT System (CPC357 – Assignment 2)
📌 Project Overview

This project presents a Smart Home IoT System designed and implemented as part of CPC357: IoT Architecture & Smart Applications. The system integrates edge devices, cloud computing, and real-time data visualization to address smart city challenges, contributing to UN Sustainable Development Goal (SDG) 11 – Sustainable Cities and Communities.

The system consists of two main subsystems:

Smart Door Security System

AI Window & Smart Drying Rack System

Both subsystems communicate with a Google Cloud Platform (GCP) Virtual Machine using the MQTT protocol, with Node-RED providing automation logic and a web-based dashboard.

🎯 Project Objectives

Design and implement a functional IoT system with real-world relevance

Demonstrate understanding of IoT layered architecture (5-layer model)

Integrate cloud computing using GCP

Enable real-time monitoring and control via a web dashboard

Apply edge AI (voice recognition) using Edge Impulse

🧱 IoT Architecture (5-Layer Model)
IoT Layer	Description
Perception Layer	Sensors and actuators such as NFC, PIR, Ultrasonic, Rain sensor, DHT11/DHT22, INMP441 microphone, servos, relays
Transport Layer	Wi-Fi connectivity using MQTT publish/subscribe protocol
Processing Layer	Node-RED flows running on a GCP Compute Engine VM
Application Layer	Node-RED Dashboard (web-based UI)
Business Layer	Automation rules, access control, alerting, logging, and system logic
🧠 System Components
🔐 Smart Door System

Hardware: Arduino UNO R4 WiFi

Features:

NFC-based access control

Motion detection (PIR)

Outside presence detection (Ultrasonic)

Smart alarm with buzzer and LED indicators

Smart light and fan automation

Functionality:

Unlock/lock via NFC or dashboard

Alarm trigger on unauthorized motion

Friendly notification for outside presence

Event logging via MQTT

🪟 AI Window & Smart Drying Rack

Hardware: ESP32-S3 + INMP441 microphone

Features:

Voice-controlled window using Edge Impulse keyword spotting

Rain-based automatic window and rack control

Temperature-based automation

Button control (single press = window, double press = drying rack)

Voice Commands:

open window

close window

noise

unknown

AI Processing:

Model trained and deployed on-device (edge inference) using Edge Impulse

☁️ Cloud Platform

Platform: Google Cloud Platform (GCP)

Services Used:

Compute Engine (Virtual Machine)

Mosquitto MQTT Broker

Node-RED Runtime

Role:

Central message broker

Automation logic processing

Web-based dashboard hosting

📊 Data Visualization

Tool: Node-RED Dashboard

Visualized Data:

Door and window states

Alarm status

Access logs

Temperature readings

Rain status

Control Features:

Open/Close door

Open/Close window

Toggle automation modes (voice, rain, temperature)

🧪 Development Process

Cloud setup on GCP VM (MQTT + Node-RED)

Firmware development for Arduino UNO R4 WiFi (Smart Door)

Firmware development for ESP32-S3 (AI Window & Drying Rack)

Dataset collection and training using Edge Impulse

MQTT integration between edge devices and cloud

Node-RED automation and dashboard design

System integration and end-to-end testing

📚 Edge Impulse – Dataset & Training

Audio Classes:

open window

close window

noise

unknown

Process:

Audio samples recorded using INMP441 microphone

Dataset uploaded to Edge Impulse

Feature extraction using MFCC

Model trained and validated

Arduino inference library generated and deployed

Benefit:

Low latency

Reduced cloud dependency

Improved privacy

🔐 Security Considerations

MQTT topic separation for command, state, and events

Cooldown and debounce logic to prevent repeated triggers

NFC UID verification for door access

Edge inference reduces sensitive audio data transmission

Controlled automation overrides during rain or alarm states

🌍 SDG 11 – Smart Cities Impact

This project supports SDG 11 by:

Enhancing home security and safety

Improving energy efficiency through automation

Demonstrating smart infrastructure integration

Enabling scalable cloud-connected IoT systems suitable for urban environments

🛠️ Hardware & Software Requirements
Hardware

Arduino UNO R4 WiFi

ESP32-S3 (Cytron Maker Feather AIoT S3)

NFC PN532

PIR Sensor

Ultrasonic Sensor (HC-SR04)

Rain Sensor

DHT11 / DHT22

INMP441 Microphone

Servo Motors (continuous + standard)

Relays, LEDs, buzzer

Software

Arduino IDE

Edge Impulse

Node-RED

Mosquitto MQTT

Google Cloud Platform

▶️ Demo Video

📺 YouTube Link: (Add your video link here)

📁 Repository Structure
├── Smart_Door/
│   └── Security_Door.ino
├── AI_Window/
│   └── AI_Window.ino
├── Node-RED/
│   └── flows.json
├── README.md

👨‍🎓 Course Information

Course: CPC357 – IoT Architecture & Smart Applications
Institution: Universiti Sains Malaysia
Assignment: Assignment 2 – IoT Smart Application
