# 🤖 ESP32 4WD WiFi Rc Controller

Complete ESP32-based WiFi Rc car with joystick control, MPU6050 balancing, OTA update, LED brightness control, and buzzer startup sound.

---

## 📦 Features
- Web joystick control
- Self-stabilizing mode
- OTA firmware upload
- LED toggle + brightness slider
- Startup melody buzzer
- Motor auto-stop safety

---

## 🧰 Hardware List

| Component | Qty |
|----------|-----|
ESP32 Board | 1 |
L298N Motor Driver | 1 |
DC Motors | 4 |
MPU6050 | 1 |
LED | 1 |
Resistor 220Ω | 1 |
Buzzer | 1 |
Battery Pack | 1 |

---

## 🔌 Wiring Guide

### 🔹 Motor Driver → ESP32

**Left Motors**

| Driver | ESP32 |
|--------|------|
IN1 | GPIO27 |
IN2 | GPIO26 |
ENA | GPIO14 |

**Right Motors**

| Driver | ESP32 |
|--------|------|
IN3 | GPIO33 |
IN4 | GPIO25 |
ENB | GPIO32 |

---

### 🔹 Power Wiring

| Device | Connection |
|------|-------------|
Motor Driver 12V | Battery + |
Motor Driver GND | Battery – |
ESP32 GND | SAME GND |

⚠ **Important:** All grounds must be connected together.

---

### 🔹 MPU6050 → ESP32

| MPU6050 | ESP32 |
|---------|------|
VCC | 3.3V |
GND | GND |
SDA | GPIO21 |
SCL | GPIO22 |

---

### 🔹 LED → ESP32

| LED | ESP32 |
|-----|------|
+ | GPIO12 via resistor |
– | GND |

---

### 🔹 Buzzer → ESP32

| Buzzer | ESP32 |
|--------|------|
+ | GPIO13 |
– | GND |

---

## 📡 WiFi Connection

After boot ESP32 creates hotspot:

```
SSID: Rc4v
Password: 12345678
```

Open browser:

```
http://192.168.4.1
```

---

## 🎮 Controls

| Action | Result |
|------|--------|
Move joystick | Drive robot |
Tap LED button | Toggle LED |
Hold LED button | Brightness slider |
Settings | Open menu |

---

## ⚖ Stabilization Mode

Uses MPU6050 + PID balancing.

Tune values in code:

```cpp
float Kp = 25.0;
float Ki = 0.5;
float Kd = 1.0;
```

---

## 🔊 Startup Sound

Robot plays 5-tone melody when powered on.

---

## 🛑 Safety

If signal stops for **3 seconds**  
→ Motors auto stop

---

## ⬆ OTA Update

Open:

```
Settings → OTA Upload
```

Upload firmware directly from browser.

---

## 🧠 Movement Logic

Joystick sends values:

```
x = turning
y = forward/back
```

ESP32 converts → PWM → Motor driver → Motors move

---

## 📚 Libraries Required

Install from Arduino Library Manager:

- ArduinoJson  
- MPU6050 (I2Cdev)  
- Wire  

---

## 🛠 Troubleshooting

| Issue | Fix |
|------|-----|
Robot not moving | Check motor battery |
Sensor not detected | Check SDA/SCL |
WiFi missing | Restart board |
Motors reversed | Swap wires |

---

## 📜 License
Free to use and modify.
