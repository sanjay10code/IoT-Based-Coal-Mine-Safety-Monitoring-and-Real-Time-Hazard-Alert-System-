<div align="center">

# ⛏️ CoalGuard

### IoT-Based Coal Mine Safety Monitoring & Real-Time Hazard Alert System

![Status](https://img.shields.io/badge/Status-Active-1D9E75?style=flat-square)
![Platform](https://img.shields.io/badge/Platform-ESP32-085041?style=flat-square&logo=espressif&logoColor=white)
![ML](https://img.shields.io/badge/ML-Hazard%20Detection-3C3489?style=flat-square)
![Dashboard](https://img.shields.io/badge/Dashboard-CoalGuard-378ADD?style=flat-square)

</div>

---

## 📸 Screenshots

<div align="center">

![Dashboard](Screenshot%202026-06-06%20170000.png)

</div>

<div align="center">

| | |
|---|---|
| ![Sensor View 1](Screenshot%202026-04-18%20003457.png) | ![Sensor View 2](Screenshot%202026-04-18%20003614.png) |
| ![Sensor View 3](Screenshot%202026-04-18%20003714.png) | ![Sensor View 4](Screenshot%202026-04-18%20012433.png) |

</div>

<div align="center">

![Circuit Design](cirkit%20designer.jpeg)

![Hardware](Screenshot%202026-03-14%20180853.png)

</div>

---

## 🧠 Overview

An intelligent IoT-based coal mine safety monitoring system using **ESP32** and **9 environmental sensors** to protect underground mining workers in real time. Sensor data streams via Wi-Fi to the **CoalGuard web dashboard** with live graphs, alerts, and historical data logging.

---

## ⚙️ Sensors

| Sensor | Detects |
|--------|---------|
| MQ-4 | Methane (CH4) |
| MQ-7 | Carbon Monoxide (CO) |
| MQ-2 | Smoke & LPG Gas |
| MQ-135 | Air Quality |
| Ozone Sensor | Ozone (O3) |
| PM2.5 | Dust Particles |
| DHT11 | Temperature & Humidity |
| Infrared Sensor | Surface Temperature |
| ESP32-CAM | Live Video Surveillance |

---

## 🖥️ Features

- **Real-time dashboard** — live sensor readings, graphical trends, system status
- **Threshold-based alerts** — buzzer, LED, and voice alerts on hazard detection
- **CSV data logging** — SD card storage for historical analysis
- **Video surveillance** — ESP32-CAM for remote visual monitoring
- **ML hazard prediction** — anomaly detection on collected sensor datasets
- **Portable** — lithium-ion battery powered, scalable and cost-effective

---

## 🛠️ Tech Stack

![ESP32](https://img.shields.io/badge/ESP32-085041?style=for-the-badge&logo=espressif&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![JavaScript](https://img.shields.io/badge/JavaScript-0C447C?style=for-the-badge&logo=javascript&logoColor=white)
![PHP](https://img.shields.io/badge/PHP-0C447C?style=for-the-badge&logo=php&logoColor=white)
![MySQL](https://img.shields.io/badge/MySQL-633806?style=for-the-badge&logo=mysql&logoColor=white)
![Python](https://img.shields.io/badge/Python-3C3489?style=for-the-badge&logo=python&logoColor=white)

---

## 🏗️ System Architecture

```
ESP32 + Sensors
      │
      ▼ Wi-Fi (HTTP/JSON)
CoalGuard Dashboard (HTML/CSS/JS/PHP/MySQL)
      │
      ├── Live sensor graphs
      ├── Alert management
      ├── Historical data
      └── ML anomaly detection
```

---

## 👨‍💻 Author

**Sanjay S** — VIT Vellore, M.Tech Software Engineering

[![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/s-sanjay-contactsanjay)
[![Email](https://img.shields.io/badge/Email-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:sanjaysureshbabu1@gmail.com)
[![GitHub](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/sanjay10code)
