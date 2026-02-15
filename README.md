ESP32 4WD WiFi Robot Controller with MPU6050 + OTA + LED + Buzzer
This project is a complete ESP32-based WiFi controlled 4WD robot system with:
Web joystick control UI
Self-stabilizing mode using MPU6050
OTA firmware upload via browser
LED control with brightness slider
Startup melody buzzer
Motor watchdog safety stop
📦 Hardware Required
Component
Quantity
ESP32 Dev Board
1
L298N Motor Driver
1
DC Motors
4
MPU6050 Gyroscope
1
LED
1
Resistor (220Ω recommended)
1
Buzzer (Active or Passive)
1
Battery Pack
1
🔌 Pin Connections (Step-by-Step)
1️⃣ Motor Driver → ESP32
Left Motors (Front Left + Rear Left)
L298N Pin
ESP32 Pin
IN1
GPIO 27
IN2
GPIO 26
ENA
GPIO 14
Right Motors (Front Right + Rear Right)
L298N Pin
ESP32 Pin
IN3
GPIO 33
IN4
GPIO 25
ENB
GPIO 32
Power
L298N
Connection
12V
Battery +
GND
Battery – AND ESP32 GND
⚠️ Important:
ESP32 GND must be connected to motor driver GND
2️⃣ MPU6050 Sensor → ESP32
MPU6050
ESP32
VCC
3.3V
GND
GND
SDA
GPIO 21
SCL
GPIO 22
3️⃣ LED → ESP32
LED Pin
ESP32
Anode (+)
GPIO 12 through resistor
Cathode (–)
GND
4️⃣ Buzzer → ESP32
Buzzer Pin
ESP32
Positive
GPIO 13
Negative
GND
📡 WiFi Access Point
When ESP32 boots, it creates a WiFi hotspot:
Copy code

SSID: Rc4v
Password: 12345678
Connect your phone to this network, then open browser:
Copy code

http://192.168.4.1
🎮 Controls
Joystick
Controls movement direction and speed.
LED Button
Tap → ON/OFF
Long Press → Brightness Slider
Settings Menu
Self Stabilize mode
Guide page
Theme switch
OTA Upload
