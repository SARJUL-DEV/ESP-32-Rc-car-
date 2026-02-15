# 🚗 ESP32 WiFi Smart Car (Normal + Stabilized Modes)

ESP32 controlled smart car with web joystick control, stabilization system, OTA update, LED control, and buzzer startup tone.

---

## 📦 Main Features

- Web control from phone browser
- Two drive modes
  - Normal drive mode
  - Stabilized mode (sensor assisted)
- LED toggle + brightness
- OTA firmware update
- Motor safety timeout
- Startup sound

---

## 🎮 Drive Modes

| Mode | Function |
|------|----------|
Normal Mode | Standard driving |
Stabilized Mode | Uses sensor to maintain balance |

---

## 🔌 Wiring Connections

### Motor Driver → ESP32

#### Left Wheels
| Driver | ESP32 |
|--------|------|
IN1 | GPIO27 |
IN2 | GPIO26 |
ENA | GPIO14 |

#### Right Wheels
| Driver | ESP32 |
|--------|------|
IN3 | GPIO33 |
IN4 | GPIO25 |
ENB | GPIO32 |

---

### Power

| Device | Connect |
|--------|--------|
Motor Driver + | Battery + |
Motor Driver GND | Battery – |
ESP32 GND | SAME GND |

**All grounds must be common**

---

### MPU6050 Sensor

| MPU6050 | ESP32 |
|---------|------|
VCC | 3.3V |
GND | GND |
SDA | GPIO21 |
SCL | GPIO22 |

---

### LED

| LED | ESP32 |
|-----|------|
+ | GPIO12 |
– | GND |

---

### Buzzer

| Buzzer | ESP32 |
|--------|------|
+ | GPIO13 |
– | GND |

---

## 📡 WiFi Access Point

```
SSID: Rc4v
Password: 12345678
```

Open browser:

```
192.168.4.1
```

---

## ⚙ Controls

| Action | Result |
|------|--------|
Move joystick | Drive car |
Tap LED button | Toggle light |
Hold LED | Brightness adjust |
Settings | Open options |

---

## 🛑 Safety System

If signal lost for **3 seconds**

→ Motors stop automatically

---

## ⬆ OTA Update

Menu → OTA Upload  
Upload new firmware wirelessly.

---

## 📚 Libraries Required

Install via Arduino Library Manager:

- ArduinoJson
- MPU6050 (I2Cdev)
- Wire

---

## 🛠 Troubleshooting

| Problem | Fix |
|--------|-----|
Car not moving | Check motor battery |
Sensor error | Check SDA/SCL |
WiFi missing | Restart ESP32 |
Wrong direction | Swap motor wires |

---

## 📜 License

Open source and free to modify.
