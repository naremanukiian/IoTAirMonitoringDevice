# IoT Firmware & AWS Pipeline

![GitHub Pages](https://img.shields.io/badge/GitHub-Pages-blue)
![Arduino](https://img.shields.io/badge/Arduino-00979D?logo=arduino&logoColor=white)
![AWS](https://img.shields.io/badge/AWS-FF9900?logo=amazonaws&logoColor=white)

This repository contains the ESP32 firmware and AWS Lambda backend used in the ClimateNet environmental monitoring project developed at TUMO Labs × UFAR. The system collects air-quality data from multiple sensors and uploads it to AWS for storage and analysis.

## Features
- **Multi-sensor data acquisition** using BME280, SCD30, SGP40, and SPS30  
- **Data packaging in JSON** format on the ESP32  
- **Wi-Fi connectivity** and secure API endpoint communication  
- **AWS Lambda processing** for incoming data  
- **Automatic timestamping (UTC+4)**  
- **Append-only S3 storage** for all device records  

## Repository Structure
The repository is organized as follows:
- **/IoT_AirMonitoring.ino/** → ESP32 firmware for reading sensors and sending data to AWS
- **/lambda_s3_upload.py/** → Python Lambda function for processing incoming sensor data and storing it in S3
  
Each folder contains all necessary code, libraries, and configuration files for its respective component.

## Workflow
1. ESP32 wakes, powers sensors, collects measurements  
2. Device sends JSON payload to AWS API  
3. Lambda parses data, adds timestamp, appends to S3 JSON file  
4. ESP32 returns to deep sleep to save power  

## Technologies Used
- Arduino (ESP32)
- AWS (Lambda, API Gateway, S3)
- Adafruit & SparkFun sensor libraries
- Python (for backend Lambda logic)

## Notes
This repo includes only **ESP32 firmware** and the **AWS backend**; dashboard and analysis tools are stored separately.



