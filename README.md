# Car Accident Detection and Prevention System with Alcohol & Smoke Detector  

## Overview  
This project is a **safety mechanism for vehicles** built using the **ESP32 microcontroller**, integrating multiple sensors and actuators. The system detects:  
- **Accidents** (using MPU6050 & SW-420 vibration sensor)  
- **Alcohol consumption** (using MQ-3 sensor)  
- **Smoke/fire hazards** (using MQ-2 sensor)  

When any incident is detected, the system:  
- Stops the motor & disables ignition  
- Activates warning indicators (LED & buzzer)  
- Sends an **SOS alert with real-time Google Maps location** to a **Telegram bot**  

This project demonstrates how **IoT, embedded systems, and robotics** can work together to enhance **road safety**.  

---

## 🛠 Features  
- 🚦 **Accident Detection** using MPU6050 accelerometer & SW-420 vibration sensor  
- 🍺 **Alcohol Detection** using MQ-3 with ignition lock  
- 💨 **Smoke Detection** for fire hazard prevention  
- 📡 **GPS-enabled SOS Alert** sent via Telegram bot with Google Maps link  
- 🎮 **Joystick-based motor control** (forward/backward movement)  
- 🔑 **Ignition & Reset mechanism** for safe recovery after incidents  
- 🔔 Real-time alerts with LEDs, buzzer, and LCD display  

---

## System Architecture  

The system is built around the ESP32 microcontroller.  
<div align="center">
  <img src="https://images2.imgbox.com/7e/f1/FNL4jt8P_o.png" alt="System Flowchart" />
</div>

---

## Complete Wiring Guide

The hardware connections of sensors, motor driver, GPS, and ESP32 are shown below:  

### Power System

| Component | Pin | Connection | Notes |
|-----------|-----|------------|-------|
| **Buck Converter (LM2596)** | IN+ | Battery+ (7.4V) | Power input |
| | IN- | Battery- (GND) | Ground input |
| | OUT+ | ESP32 VIN | 5V regulated output |
| | OUT- | ESP32 GND | Common ground |
| **Battery Pack** | + | Buck Converter IN+ | 2x 3.7V Li-ion = 7.4V |
| | - | Buck Converter IN- | Common ground |

---

### ESP32 Main Controller

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **ESP32 Power** | VIN | Buck OUT+ | Power | 5V from buck converter |
| | GND | Buck OUT- | Ground | Common ground |
| | 3.3V | - | Output | 3.3V for sensors |

---

### Gas & Smoke Sensors

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **MQ-2 Smoke Sensor** | A0 (Analog) | GPIO 35 | Analog Input | Continuous smoke level |
| | D0 (Digital) | GPIO 23 | Digital Input | Threshold trigger |
| | VCC | 5V | Power | Sensor power |
| | GND | GND | Ground | Common ground |
| **MQ-3 Alcohol Sensor** | A0 (Analog) | GPIO 34 | Analog Input | Alcohol level reading |
| | D0 (Digital) | GPIO 33 | Digital Input | Threshold trigger |
| | VCC | 5V | Power | Sensor power |
| | GND | GND | Ground | Common ground |

---

### Motion & Location Sensors

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **MPU6050 Accelerometer** | SDA | GPIO 21 | I2C Data | Accelerometer data |
| | SCL | GPIO 22 | I2C Clock | I2C clock line |
| | VCC | 3.3V | Power | Sensor power |
| | GND | GND | Ground | Common ground |
| **NEO-6M GPS Module** | TX | GPIO 16 | UART RX | GPS data transmission |
| | RX | GPIO 17 | UART TX | GPS data reception |
| | VCC | 3.3V | Power | GPS module power |
| | GND | GND | Ground | Common ground |

---

### Motor Control System

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **L298N Motor Driver** | VCC (Logic) | 5V | Power | Logic power supply |
| | +12V (Motor) | Battery+ | Power | Motor power (7.4V) |
| | GND | GND | Ground | Common ground |
| | ENA | 5V or GPIO 18 | PWM/Digital | Motor enable (optional) |
| | INA | GPIO 13 | Digital Output | Motor direction A |
| | INB | GPIO 12 | Digital Output | Motor direction B |
| **DC Motor** | OUTA | L298N OUTA | Motor Output | Motor terminal 1 |
| | OUTB | L298N OUTB | Motor Output | Motor terminal 2 |

---

### Display & User Interface

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **16x2 LCD (I2C)** | SDA | GPIO 21 | I2C Data | Display data (shared) |
| | SCL | GPIO 22 | I2C Clock | I2C clock (shared) |
| | VCC | 5V | Power | LCD power |
| | GND | GND | Ground | Common ground |
| **Joystick Module** | VRX | GPIO 32 | Analog Input | Horizontal axis |
| | VCC | 3.3V | Power | Joystick power |
| | GND | GND | Ground | Common ground |

---

### Alert & Indicator System

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **Buzzer** | + (Positive) | GPIO 5 | Digital Output | Sound alerts |
| | - (Negative) | GND | Ground | Common ground |
| **LED1 (Warning)** | Anode (Long leg) | GPIO 2 | Digital Output | Via 220Ω resistor |
| | Cathode (Short leg) | GND | Ground | Common ground |
| **LED2 (Ignition)** | Anode (Long leg) | GPIO 19 | Digital Output | Via 220Ω resistor |
| | Cathode (Short leg) | GND | Ground | Common ground |

---

### Control Buttons

| Component | Component Pin | ESP32 GPIO | Connection Type | Description |
|-----------|---------------|-------------|-----------------|-------------|
| **Reset Button** | Pin 1 | GND | Ground | Button ground |
| | Pin 2 | GPIO 27 | Digital Input | System reset (pull-up) |
| **Ignition Button** | Pin 1 | GND | Ground | Button ground |
| | Pin 2 | GPIO 14 | Digital Input | Ignition enable (pull-up) |

---

## GPIO Pin Summary

| GPIO Pin | Component | Function | Type |
|----------|-----------|----------|------|
| GPIO 2 | LED1 (Warning) | Warning indicator | Digital Output |
| GPIO 5 | Buzzer | Sound alerts | Digital Output |
| GPIO 12 | L298N INB | Motor direction B | Digital Output |
| GPIO 13 | L298N INA | Motor direction A | Digital Output |
| GPIO 14 | Ignition Button | Enable ignition | Digital Input |
| GPIO 15 | Servo Motor | Position control | PWM Output |
| GPIO 16 | GPS TX | GPS data receive | UART RX |
| GPIO 17 | GPS RX | GPS data transmit | UART TX |
| GPIO 18 | L298N ENA | Motor enable (optional) | PWM Output |
| GPIO 19 | LED2 (Ignition) | Ignition indicator | Digital Output |
| GPIO 21 | MPU6050 SDA, LCD SDA | I2C data line | I2C |
| GPIO 22 | MPU6050 SCL, LCD SCL | I2C clock line | I2C |
| GPIO 23 | MQ-2 Digital | Smoke threshold | Digital Input |
| GPIO 26 | Joystick VRY | Vertical axis | Analog Input |
| GPIO 27 | Reset Button | System reset | Digital Input |
| GPIO 32 | Joystick VRX, SW | Horizontal axis & button | Analog/Digital |
| GPIO 33 | MQ-3 Digital | Alcohol threshold | Digital Input |
| GPIO 34 | MQ-3 Analog | Alcohol level | Analog Input |
| GPIO 35 | MQ-2 Analog | Smoke level | Analog Input |

---

## Power Distribution

| Voltage Level | Components | Source |
|---------------|------------|---------|
| **7.4V** | L298N Motor Power, Battery Input | 2x 3.7V Li-ion batteries |
| **5V** | ESP32 VIN, LCD, Servo, MQ Sensors | Buck converter output |
| **3.3V** | MPU6050, GPS, Joystick | ESP32 internal regulator |
| **GND** | All components | Common ground connection |

---

## Important Notes

- **All GND pins** must be connected to common ground
- **MQ sensors** require 5V for proper operation  
- **MPU6050 and GPS** use 3.3V to prevent damage
- **I2C devices** (LCD, MPU6050) share the same SDA/SCL lines
- **LEDs** require 220Ω current-limiting resistors
- **Buttons** use internal pull-up resistors in ESP32
- **Joystick SW** shares GPIO 32 with VRX (button press detection)

---

## 🛠 Prototype Hardware  

Real hardware setup of the system:  

<div align="center">
  <img src="https://images2.imgbox.com/f7/c9/FglbuKU0_o.jpg" alt="Hardware Pic" width="800" />
</div>

---

## Hardware Components  
| Component | Quantity | Purpose |
|-----------|----------|---------|
| ESP32 (38 pins) | 1 | Central controller |
| MQ-3 Sensor | 1 | Alcohol detection |
| MQ-2 Sensor | 1 | Smoke detection |
| SW-420 Sensor | 1 | Accident/vibration detection |
| MPU6050 | 1 | Accelerometer for accident detection |
| NEO-6M GPS Module | 1 | Location tracking |
| L298N Motor Driver + DC Motor + Wheel | 1 | Vehicle movement simulation |
| Joystick Module (5-pin) | 1 | Motor control input |
| 16x2 I2C LCD | 1 | System status display |
| Buzzer & LEDs | Multiple | Alerts & indicators |
| Push Buttons | 2 | Ignition & system reset |
| Buck Converter (LM2596) | 1 | Voltage regulation |
| Batteries (3.7V) | 2 | Power supply |
| Battery Holder (2-cell) | 1 | Holds 3.7V batteries |

---

## Software & Tools  
- **Arduino IDE** (for coding & uploading to ESP32)  
- **Telegram Bot API** (for SOS alert messaging)  
- **C/C++ (Arduino language)**  

---

## Telegram Bot Integration  
The system uses a **Telegram Bot** to send **real-time SOS alerts**.  
👉 [Click here to view the bot](https://t.me/carsos461_bot)  
👉 [Setup Guide: Create your own Telegram Bot](/telegram_bot.py)  

Whenever alcohol, smoke, or an accident is detected, the bot sends:  
- 🚨 **Incident type** (Accident / Alcohol / Smoke)  
- 📍 **Google Maps location link** (from NEO-6M GPS)  

Example SOS Alert:  
```
ALERT: Collision detected!
Location: https://www.google.com/maps?q=23.780573,90.279239
```
<div align="center">
  <img src="https://images2.imgbox.com/4a/57/9qSDx0qG_o.jpg" alt="Telegram SOS alert" width="350" />
</div>

## 🔧 How It Works  
1. **Ignition ON** → Driver presses ignition button.  
2. **Normal Drive** → Joystick controls motor direction.  
3. **Incident Detected** → Motor stops, ignition disabled, alerts triggered.  
4. **SOS Sent** → Telegram bot sends Google Maps link to contacts..  
5. **Reset System** → User presses reset button to clear alerts.  

---

## Results  
✅ Motor control worked smoothly during normal conditions.  
✅ Alcohol & smoke detection disabled ignition and triggered warnings.  
✅ Accident detection reliably stopped the motor and sent SOS alerts.  
✅ Telegram bot successfully sent **real-time Google Maps location links**.  

---

## Sustainability & Impact  
- **Sustainable Design** → Powered by rechargeable batteries, modular, and reusable components.  
- **Impact** → Prevents drunk driving, detects accidents early, improves emergency response.  
- **Applications** → Can be adapted for school buses, taxis, delivery fleets, and private vehicles.  

---

## Challenges Faced  
- Sensor calibration (MQ-2 & MQ-3 highly sensitive to environment)  
- Joystick drifting & pin conflicts with Wi-Fi functions  
- Debugging GPS signal acquisition indoors  
- Power distribution issues between ESP32, motor, and sensors  

---

## Future Improvements  
- Sensor calibration for higher accuracy  
- Machine Learning to reduce false positives  
- Mobile app/cloud dashboard for real-time monitoring  
- Solar/kinetic power integration for sustainability  
- Additional sensors (heart-rate, eye-tracking) for driver fatigue detection  

---

## Getting Started  

### Clone Repository  
```bash
git clone https://github.com/fah-ayon/Car-Accident-Detection-and-Prevention-System-with-Alcohol-Smoke-Detector.git
```


### Open in Arduino IDE

1. Install ESP32 board support (see below).
2. Install required libraries:

   * `LiquidCrystal_I2C`
   * `MPU6050`
   * `WiFi`
   * `HTTPClient`

### Upload Code

1. Connect ESP32 via USB.
2. Open `final_code_fixed_V2.ino`.
3. Select the correct board & COM port.
4. Upload the sketch.

### Run & Test

1. Power with batteries.
2. Test alcohol, smoke, and collision events.
3. Check Telegram bot for real-time Google Maps alerts.

---

## ESP32 Setup Instructions

### Install ESP32 Board Support

1. Go to `File → Preferences → Additional Boards URL` and paste:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

2. Go to `Tools → Board → Boards Manager → Search esp32 → Install`.

### Select the Correct Board

`Tools → Board → ESP32 → Choose your ESP32 model.`

### Install Libraries

`Sketch → Include Library → Manage Libraries → Install:`

* `LiquidCrystal_I2C` (Frank de Brabander)
* `MPU6050` (Electronic Cats)

### Find the Correct Port

`Tools → Port → Select your ESP32 COM port.`
Use Device Manager (Win + R → `devmgmt.msc`) if unsure.

### Fix "Connecting…" Upload Error

* Hold **BOOT** while uploading, release when `Connecting...` appears.
* If still failing: Hold **BOOT**, press **RESET** once, then release **BOOT**.

### Serial Monitor

Set baud rate to `115200`.

✅ Your ESP32 is now ready!

---

✨ **Project Summary:**
This system integrates IoT and embedded hardware to monitor vehicle conditions and alert users in real-time, improving road safety and emergency response.

Best of luck
— Abdullah Al Fahad  
LinkedIn: https://www.linkedin.com/in/abdullahalfahadayon/