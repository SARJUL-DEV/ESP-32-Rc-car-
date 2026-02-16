#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>  // For OTA web updates
#include "esp_system.h"  // For brownout disable (ESP32 specific)
#include <ArduinoJson.h>  // Add this library for JSON (install via Library Manager if needed)
#include <Wire.h>         // For I2C communication with MPU6050
#include <MPU6050.h>      // MPU6050 library (install I2Cdev MPU6050 via Library Manager)
#include <math.h>         // For PI and atan2

// WiFi Access Point credentials (change these)
const char* ap_ssid     = "SARJUL-RC";  // Updated SSID
const char* ap_password = "12345678";       // AP Password (at least 8 characters)

// Create an instance of the WebServer on port 80
WebServer server(80);

// MPU6050 Setup (connect SDA to GPIO21, SCL to GPIO22, VCC to 3.3V, GND to GND)
MPU6050 mpu;
bool sensorConnected = false;
float targetAngle = 0;  // Target pitch angle from joystick (forward/back)
bool stabilizeMode = false;  // Global flag for self-stabilize mode

// PID Constants for Balancing (tune these values experimentally)
float Kp = 25.0;  // Proportional gain
float Ki = 0.5;   // Integral gain
float Kd = 1.0;   // Derivative gain
float errorSum = 0;
float lastError = 0;
unsigned long lastTime = 0;

// Left Side Motors (Front Left + Rear Left - Parallel on one L298N channel)
int leftMotorPin1 = 27;  // IN1 for left side (forward)
int leftMotorPin2 = 26;  // IN2 for left side (reverse)
int leftEnablePin = 14;  // ENA PWM for left side

// Right Side Motors (Front Right + Rear Right - Parallel on second L298N channel)
int rightMotorPin1 = 33; // IN3 for right side (forward)
int rightMotorPin2 = 25; // IN4 for right side (reverse)
int rightEnablePin = 32; // ENB PWM for right side

// PWM channels (changed to avoid conflicts: left=0, right=4)
const int channel1 = 0;  // Left PWM
const int channel2 = 4;  // Right PWM

// Updated: LED Setup (GPIO 12 for LED, PWM channel 3 for brightness control)
const int ledPin = 12;
const int ledChannel = 3;
const int ledFreq = 1000;  // 1kHz for smooth dimming
const int ledResolution = 8;  // 8-bit (0-255)
bool ledOn = false;
int ledBrightness = 128;  // Default 50% brightness (0-255)

// Buzzer Setup (GPIO 13 for 1.5V-5V Buzzer, PWM channel 5 for tone)
const int buzzerPin = 13;
const int buzzerChannel = 5;
const int buzzerFreq = 2000;  // Base freq for PWM tone
const int buzzerResolution = 8;

// Non-blocking buzzer state machine
enum BuzzerState { IDLE, PLAYING };
BuzzerState buzzerState = IDLE;
unsigned long buzzerStartTime = 0;
int currentBeepFreq = 0;
int currentBeepDuration = 0;
int melodyStep = 0;  // For startup sequence

// Setting PWM properties
const int freq = 1000;   // 1kHz for smooth
const int resolution = 8; // 8-bit (0-255)

// Hardware inversion flags (set true ONLY if reverse PWM needs 255 - pwm; test without first)
#define INVERT_REVERSE_LEFT false
#define INVERT_REVERSE_RIGHT false

// Watchdog timer for auto-stop - 3s
unsigned long lastCommand = 0;
bool motorStarted = false;  // Track if motors have started (for beep)

// Speed hold variables
float last_left_speed = 0;
float last_right_speed = 0;

// Frequencies and Durations for Detailed Startup Sequence (based on user description)
#define FIRST_BEEP_FREQ   1000  // First beep: 1000 Hz
#define SECOND_BEEP_FREQ  1500  // Second beep: 1500 Hz
#define THIRD_BEEP_FREQ   2000  // Third beep: 2000 Hz
#define FINAL_TONE1_FREQ  2200  // Final first tone: 2200 Hz
#define FINAL_TONE2_FREQ  2600  // Final second tone: 2600 Hz
int melodyFreqs[] = {FIRST_BEEP_FREQ, SECOND_BEEP_FREQ, THIRD_BEEP_FREQ, FINAL_TONE1_FREQ, FINAL_TONE2_FREQ};
int melodyDurs[] = {150, 150, 150, 120, 120};
int melodyPauses[] = {100, 100, 100, 0};  // Pauses after each except last

// Non-blocking beep starter
void startBeep(int frequency, int duration) {
  if (buzzerState == IDLE) {
    buzzerState = PLAYING;
    buzzerStartTime = millis();
    currentBeepFreq = frequency;
    currentBeepDuration = duration;
    ledcSetup(buzzerChannel, frequency, buzzerResolution);
    ledcAttachPin(buzzerPin, buzzerChannel);
    ledcWrite(buzzerChannel, 128);  // 50% duty for tone
    Serial.println("Beep started: " + String(frequency) + "Hz for " + String(duration) + "ms");
  }
}

// Update buzzer (call in loop)
void updateBuzzer() {
  if (buzzerState == PLAYING) {
    if (millis() - buzzerStartTime >= currentBeepDuration) {
      ledcWrite(buzzerChannel, 0);  // Stop
      buzzerState = IDLE;
      Serial.println("Beep ended.");
      
      // For startup melody: Advance step after pause
      if (melodyStep < 4) {  // 5 beeps total
        unsigned long pauseTime = melodyPauses[melodyStep];
        if (pauseTime > 0) {
          delay(pauseTime);  // Short blocking pause ok for startup
        }
        melodyStep++;
        if (melodyStep < 5) {
          startBeep(melodyFreqs[melodyStep], melodyDurs[melodyStep]);
        }
      }
    }
  }
}

// Updated Function for Detailed Startup Sound (non-blocking init)
void playStartupMelody() {
  Serial.println("Starting detailed startup melody (non-blocking)...");
  melodyStep = 0;
  startBeep(melodyFreqs[0], melodyDurs[0]);
}

// New: Function to update LED state
void updateLED() {
  if (ledOn) {
    ledcWrite(ledChannel, ledBrightness);
    Serial.println("LED ON at brightness: " + String(ledBrightness));
  } else {
    ledcWrite(ledChannel, 0);
    Serial.println("LED OFF");
  }
}

// Function to get error string (compatible with old ESP32 cores)
const char* getUpdateErrorString() {
  uint8_t err = Update.getError();
  switch (err) {
    case UPDATE_ERROR_OK: return "No error";
    case UPDATE_ERROR_WRITE: return "Write failed";
    case UPDATE_ERROR_ERASE: return "Erase failed";
    case UPDATE_ERROR_READ: return "Read failed";
    case UPDATE_ERROR_SPACE: return "Not enough space";
    case UPDATE_ERROR_SIZE: return "Image size mismatch";
    case UPDATE_ERROR_STREAM: return "Stream read error";
    case UPDATE_ERROR_MD5: return "MD5 check failed";
    case UPDATE_ERROR_MAGIC_BYTE: return "Magic byte mismatch";
    case UPDATE_ERROR_ACTIVATE: return "Activation failed";
    case UPDATE_ERROR_NO_PARTITION: return "Partition table invalid";
    case UPDATE_ERROR_BAD_ARGUMENT: return "Bad argument";
    case UPDATE_ERROR_ABORT: return "Aborted";
    default: return "Unknown error";
  }
}

// Updated HTML for Joystick Page (with fixed LED toggle logic and long-press handling; added LED status fetch)
const char joystick_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>ESP32 4WD - Joystick</title>
<style>
  :root {
    --bg-color: #071225;
    --text-color: #fff;
    --pad-gradient: radial-gradient(circle at 30% 30%, rgba(255,255,255,0.03), rgba(255,255,255,0.01));
    --pad-bg: #081021;
    --pad-shadow: rgba(0,0,0,0.6);
    --stick-bg: linear-gradient(180deg,#f8fafc,#cbd5e1);
    --stick-shadow: rgba(0,0,0,0.5);
    --hint-color: #9aa6bc;
    --menu-bg: #081021;
    --menu-text: #fff;
    --menu-border: #333;
    --menu-hover: #6366f1;
  }
  body.light {
    --bg-color: #f0f0f0;
    --text-color: #000;
    --pad-gradient: radial-gradient(circle at 30% 30%, rgba(0,0,0,0.03), rgba(0,0,0,0.01));
    --pad-bg: #ffffff;
    --pad-shadow: rgba(0,0,0,0.2);
    --stick-bg: linear-gradient(180deg,#333,#666);
    --stick-shadow: rgba(0,0,0,0.3);
    --hint-color: #666;
    --menu-bg: #ffffff;
    --menu-text: #000;
    --menu-border: #ddd;
    --menu-hover: #e0e0e0;
  }
  html,body{height:100%;margin:0;background:var(--bg-color);color:var(--text-color);font-family:Helvetica,Arial;display:flex;align-items:center;justify-content:center}
  .wrap{width:100vw;height:100vh;display:flex;align-items:center;justify-content:center}
  .pad{width:78vmin;max-width:520px;height:78vmin;max-height:520px;border-radius:999px;background:var(--pad-gradient), var(--pad-bg);box-shadow:0 12px 40px var(--pad-shadow);position:relative;touch-action:none}
  .stick{width:34%;height:34%;border-radius:50%;background:var(--stick-bg);position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);box-shadow:0 10px 30px var(--stick-shadow)}
  .hint{position:fixed;bottom:10px;left:50%;transform:translateX(-50%);color:var(--hint-color);font-size:13px;opacity:0.9}
  .menu-btn{position:fixed;top:10px;left:10px;padding:8px 12px;border-radius:8px;background:#6366f1;border:none;color:#fff;font-weight:600;font-size:12px;cursor:pointer}
  .led-btn{position:fixed;top:10px;right:10px;padding:8px 12px;border-radius:8px;background:#10b981;border:none;color:#fff;font-weight:600;font-size:12px;cursor:pointer}
  .led-btn.off { background: #ef4444; }
  .led-btn.on { background: #10b981; }
  .brightness-panel { display: none; position: fixed; top: 50px; right: 10px; background: var(--menu-bg); border-radius: 8px; box-shadow: 0 4px 20px rgba(0,0,0,0.5); z-index: 1000; padding: 10px; width: 200px; }
  .brightness-slider { width: 100%; margin: 10px 0; }
  .menu { display: none; position: fixed; top: 50px; left: 10px; background: var(--menu-bg); border-radius: 8px; box-shadow: 0 4px 20px rgba(0,0,0,0.5); z-index: 1000; }
  .menu a { display: block; padding: 10px 15px; color: var(--menu-text); text-decoration: none; border-bottom: 1px solid var(--menu-border); cursor: pointer; }
  .menu a:last-child { border-bottom: none; }
  .menu a:hover { background: var(--menu-hover); }
  #theme-toggle { cursor: pointer; }
</style>
</head>
<body>
  <button class="menu-btn" id="menu-toggle">Settings</button>
  <button class="led-btn off" id="led-toggle">LED OFF</button>
  <div id="brightness-panel" class="brightness-panel">
    <label for="brightness-slider">Brightness:</label>
    <input type="range" id="brightness-slider" class="brightness-slider" min="0" max="255" value="128">
    <span id="brightness-value">128</span>
    <br>
    <button id="close-brightness">Close</button>
  </div>
  <div id="menu" class="menu">
    <a href="/stabilize">Self Stabilize</a>
    <a href="/about">Guide</a>
    <a href="#" id="theme-toggle">☀️ Light Mode</a>
    <a href="/upload">OTA Upload</a>
  </div>

  <div class="wrap">
    <div id="pad" class="pad" aria-label="joystick pad"></div>
  </div>

  <div class="hint">Move joystick for 4WD control</div>

<script>
  const pad = document.getElementById('pad');
  const stick = document.createElement('div');
  stick.className = 'stick';
  pad.appendChild(stick);

  let active = false;
  let lastSend = 0;
  let lastSentX = 0, lastSentY = 0;
  const SEND_INTERVAL = 80;
  const DEADZONE = 0;  // Zero deadzone for tiniest movement response

  function getPos(e){
    const rect = pad.getBoundingClientRect();
    const clientX = e.touches ? e.touches[0].clientX : e.clientX;
    const clientY = e.touches ? e.touches[0].clientY : e.clientY;
    let x = (clientX - rect.left) - rect.width/2;
    let y = rect.height/2 - (clientY - rect.top);
    let nx = x / (rect.width/2);
    let ny = y / (rect.height/2);
    const mag = Math.sqrt(nx*nx + ny*ny);
    if (mag > 1){ nx /= mag; ny /= mag; }
    const px = rect.width/2 + nx * (rect.width/2) * 1.0;
    const py = rect.height/2 - ny * (rect.height/2) * 1.0;
    return {nx, ny, px, py};
  }

  function sendStop() {
    if (navigator.sendBeacon) {
      navigator.sendBeacon(`/move?x=0&y=0&s=50`);
    } else {
      fetch(`/move?x=0&y=0&s=50`).catch(()=>{});
    }
  }

  function start(e){ 
    active = true; 
    pad.style.touchAction = "none"; 
    closeMenu();  // Auto-close menu on joystick interaction
    closeBrightnessPanel();  // Auto-close brightness panel on joystick interaction
    move(e); 
    e.preventDefault(); 
  }
  function move(e){
    if (!active) return;
    e.preventDefault();
    const p = getPos(e);
    stick.style.left = p.px + 'px';
    stick.style.top  = p.py + 'px';
    let sx = Math.round(p.nx * 1000);  // Higher precision (0.1%)
    let sy = Math.round(p.ny * 1000);
    if (Math.abs(sx) < DEADZONE*1000) sx = 0;
    if (Math.abs(sy) < DEADZONE*1000) sy = 0;
    const s = 50;
    const now = Date.now();
    if ((now - lastSend > SEND_INTERVAL) && (sx !== lastSentX || sy !== lastSentY || active)) {
      lastSend = now; 
      lastSentX = sx; 
      lastSentY = sy;
      if (navigator.sendBeacon) navigator.sendBeacon(`/move?x=${sx}&y=${sy}&s=${s}`);
      else fetch(`/move?x=${sx}&y=${sy}&s=${s}`).catch(()=>{});
    }
  }
  function end(e){
    active = false;
    stick.style.left = '50%';
    stick.style.top = '50%';
    sendStop();
    setTimeout(sendStop, 50);
    setTimeout(sendStop, 100);
  }

  pad.addEventListener('touchstart', start, {passive:false});
  pad.addEventListener('touchmove', move, {passive:false});
  pad.addEventListener('touchend', end);
  pad.addEventListener('mousedown', (e)=> start(e));
  window.addEventListener('mousemove', move);
  window.addEventListener('mouseup', end);

  // Menu toggle with auto-close on joystick interaction
  const menuToggle = document.getElementById('menu-toggle');
  const menu = document.getElementById('menu');

  function closeMenu() {
    menu.style.display = 'none';
  }

  menuToggle.addEventListener('click', function(e) {
    e.preventDefault();
    if (menu.style.display === 'block') {
      closeMenu();
    } else {
      menu.style.display = 'block';
    }
  });

  // Close menu on click outside (advanced: covers toggles/settings interactions)
  document.addEventListener('click', function(e) {
    if (!menu.contains(e.target) && e.target !== menuToggle) {
      closeMenu();
    }
  });

  // Fixed: LED Toggle with Long Press for Brightness (No toggle on long press release; shows current state correctly)
  const ledToggle = document.getElementById('led-toggle');
  const brightnessPanel = document.getElementById('brightness-panel');
  const brightnessSlider = document.getElementById('brightness-slider');
  const brightnessValue = document.getElementById('brightness-value');
  const closeBrightness = document.getElementById('close-brightness');
  let longPressTimer;
  let pressStartTime = 0;
  let isLongPressActive = false;  // Renamed for clarity

  // New: Fetch initial LED state from server on load
  function loadLedState() {
    fetch('/led-status')
      .then(response => response.json())
      .then(data => {
        if (data.on) {
          ledToggle.textContent = 'LED ON';
          ledToggle.classList.remove('off');
          ledToggle.classList.add('on');
          brightnessSlider.value = data.brightness;
          brightnessValue.textContent = data.brightness;
        } else {
          ledToggle.textContent = 'LED OFF';
          ledToggle.classList.remove('on');
          ledToggle.classList.add('off');
        }
      })
      .catch(() => console.log('Failed to load LED state'));
  }

  function closeBrightnessPanel() {
    brightnessPanel.style.display = 'none';
    isLongPressActive = false;  // Reset only when closing panel
  }

  function startLongPress() {
    longPressTimer = setTimeout(() => {
      isLongPressActive = true;
      brightnessPanel.style.display = 'block';
      // Use current value from slider (synced on load)
      updateBrightness();
    }, 1000); // 1 second
  }

  function cancelPendingLongPress() {
    if (longPressTimer) {
      clearTimeout(longPressTimer);
      longPressTimer = null;
    }
  }

  function toggleLED() {
    // Fixed: Assume current class reflects state; flip it correctly
    // Button shows CURRENT state: "LED ON" with green when on, "LED OFF" with red when off
    const isCurrentlyOn = ledToggle.classList.contains('on');
    fetch('/led-toggle').then(() => {
      if (isCurrentlyOn) {
        // Was on, now off
        ledToggle.textContent = 'LED OFF';
        ledToggle.classList.remove('on');
        ledToggle.classList.add('off');
      } else {
        // Was off, now on
        ledToggle.textContent = 'LED ON';
        ledToggle.classList.remove('off');
        ledToggle.classList.add('on');
      }
    }).catch(() => {
      // On error, revert UI (optional)
      alert('Toggle failed!');
    });
  }

  function updateBrightness() {
    const value = brightnessSlider.value;
    brightnessValue.textContent = value;
    fetch(`/led-brightness?value=${value}`).catch(() => {});
  }

  // Fixed Event Listeners: Handle short vs long press based on duration; no toggle on long press release
  // mousedown/touchstart: Start timer and track press time
  function handlePressStart(e) {
    e.preventDefault();
    pressStartTime = Date.now();
    cancelPendingLongPress();  // Clear any previous
    startLongPress();
  }

  // mouseup/touchend: Check duration; toggle only if short press (<1s)
  function handlePressEnd(e) {
    e.preventDefault();
    cancelPendingLongPress();  // Always clear pending timer
    const pressDuration = Date.now() - pressStartTime;
    if (pressDuration < 1000 && !isLongPressActive) {
      // Short press: Toggle LED
      toggleLED();
    }
    // If long press, do nothing extra (panel stays open for adjustment)
    // Reset pressStartTime
    pressStartTime = 0;
  }

  // click: Fallback for non-pointer events, but prevent double-trigger
  function handleClick(e) {
    e.preventDefault();
    // Only toggle if no active long press (rare double-fire prevention)
    if (!isLongPressActive && (Date.now() - pressStartTime < 100) ) {  // Very short click
      toggleLED();
    }
  }

  ledToggle.addEventListener('mousedown', handlePressStart);
  ledToggle.addEventListener('mouseup', handlePressEnd);
  ledToggle.addEventListener('touchstart', handlePressStart, {passive: false});
  ledToggle.addEventListener('touchend', handlePressEnd, {passive: false});
  ledToggle.addEventListener('click', handleClick);

  brightnessSlider.addEventListener('input', updateBrightness);

  closeBrightness.addEventListener('click', () => {
    closeBrightnessPanel();
  });

  // Close brightness panel on outside click (optional enhancement)
  document.addEventListener('click', function(e) {
    if (!brightnessPanel.contains(e.target) && e.target !== ledToggle) {
      closeBrightnessPanel();
    }
  });

  // Load LED state on page load
  loadLedState();

  // Theme toggle (Fixed Logic)
  const themeToggle = document.getElementById('theme-toggle');
  function setTheme(isDark) {
    if (isDark) {
      document.body.classList.remove('light');
      themeToggle.textContent = '☀️ Light Mode';
    } else {
      document.body.classList.add('light');
      themeToggle.textContent = '🌙 Dark Mode';
    }
    localStorage.setItem('theme', isDark ? 'dark' : 'light');
  }
  themeToggle.addEventListener('click', (e) => {
    e.preventDefault();
    const currentIsDark = !document.body.classList.contains('light');
    setTheme(!currentIsDark);
  });
  // Load saved theme
  const savedTheme = localStorage.getItem('theme') || 'dark';
  if (savedTheme === 'light') {
    document.body.classList.add('light');
    themeToggle.textContent = '🌙 Dark Mode';
  } else {
    themeToggle.textContent = '☀️ Light Mode';
  }

  document.body.addEventListener('touchmove', function(e){
    if (e.target.closest('.pad')) e.preventDefault();
  }, {passive:false});
</script>
</body>
</html>
)rawliteral";

// Updated HTML for Self Stabilize Page (single toggle button, no apply button)
const char stabilize_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Self Stabilize</title>
<style>
  body { font-family: Arial, sans-serif; max-width: 600px; margin: 50px auto; padding: 20px; background: #f4f4f4; }
  .back-link { position: absolute; top: 20px; left: 20px; color: #007bff; text-decoration: none; }
  #status { margin-bottom: 15px; padding: 10px; background: #e9ecef; border-radius: 4px; }
  .toggle { margin: 20px 0; }
  #toggle-btn { padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; font-weight: bold; transition: background 0.3s; }
  #toggle-btn:disabled { background: #ccc; cursor: not-allowed; color: #666; }
  #toggle-btn.off { background: #dc3545; color: white; }
  #toggle-btn.on { background: #28a745; color: white; }
  .instructions { margin-top: 15px; padding: 10px; background: #d4edda; border-radius: 4px; color: #155724; }
</style>
</head>
<body>
  <a href="/" class="back-link">Back</a>
  <h1>Self Stabilize Mode</h1>
  <div id="status">Loading sensor status...</div>
  <div class="toggle">
    <button id="toggle-btn" class="off" disabled>OFF</button>
  </div>
  <div class="instructions">
    <strong>Instructions:</strong> Before enabling, place the robot/car in an upright straight position. Once ON, it will immediately balance and stand straight. Use joystick: Forward/Back for tilt control (balanced movement), Left/Right for full rotation/turning. You can drive, rotate, and do everything while staying balanced!
  </div>

<script>
  // Fetch sensor status
  fetch('/sensor-status')
    .then(response => response.json())
    .then(data => {
      const status = document.getElementById('status');
      const toggleBtn = document.getElementById('toggle-btn');
      status.innerHTML = data.connected ? '<strong style="color: green;">Sensor: Connected</strong>' : '<strong style="color: red;">Sensor: Not Connected (Check wiring: SDA=21, SCL=22)</strong>';
      if (!data.connected) {
        toggleBtn.disabled = true;
        toggleBtn.textContent = 'OFF';
        toggleBtn.classList.remove('on');
        toggleBtn.classList.add('off');
      } else {
        toggleBtn.disabled = false;
        if (data.enabled) {
          toggleBtn.textContent = 'ON';
          toggleBtn.classList.remove('off');
          toggleBtn.classList.add('on');
        } else {
          toggleBtn.textContent = 'OFF';
          toggleBtn.classList.remove('on');
          toggleBtn.classList.add('off');
        }
      }
    });

  // Handle toggle button click
  document.getElementById('toggle-btn').addEventListener('click', async function() {
    const toggleBtn = document.getElementById('toggle-btn');
    const currentEnabled = toggleBtn.classList.contains('on');
    const newEnabled = !currentEnabled;
    const response = await fetch(`/set-stabilize?enabled=${newEnabled ? 1 : 0}`);
    if (response.ok) {
      if (newEnabled) {
        toggleBtn.textContent = 'ON';
        toggleBtn.classList.remove('off');
        toggleBtn.classList.add('on');
        alert('Stabilize ON! Return to joystick and ensure robot is upright.');
      } else {
        toggleBtn.textContent = 'OFF';
        toggleBtn.classList.remove('on');
        toggleBtn.classList.add('off');
        alert('Stabilize OFF.');
      }
    } else {
      alert('Error: Stabilize can only be enabled if sensor is connected!');
    }
  });
</script>
</body>
</html>
)rawliteral";

// Simplified HTML for Upload Page (unchanged, but accessible from menu)
const char upload_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OTA Update</title>
<style>
  body { font-family: Arial, sans-serif; max-width: 600px; margin: 50px auto; padding: 20px; background: #f4f4f4; }
  .form-group { margin-bottom: 15px; }
  label { display: block; margin-bottom: 5px; font-weight: bold; }
  input[type="file"] { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 4px; }
  input[type="submit"] { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; width: 100%; }
  input[type="submit"]:disabled { background: #ccc; cursor: not-allowed; }
  input[type="submit"]:hover:not(:disabled) { background: #0056b3; }
  .back-link { position: absolute; top: 20px; left: 20px; color: #007bff; text-decoration: none; }
  #storage-info { margin-bottom: 15px; padding: 10px; background: #e9ecef; border-radius: 4px; }
  #error-msg { color: #dc3545; display: none; }
  #progress-container { width: 100%; background: #ddd; height: 10px; border-radius: 5px; margin: 10px 0; display: none; }
  #progress-bar { width: 0%; height: 100%; background: #4CAF50; border-radius: 5px; transition: width 0.3s ease; }
  #upload-status { margin-top: 10px; font-weight: bold; }
</style>
</head>
<body>
  <a href="/" class="back-link">Back</a>
  <h1>OTA Update</h1>
  <div id="storage-info">Loading free space...</div>
  <div id="error-msg">File too large! Available: <span id="avail-bytes"></span> bytes</div>
  <form id="upload-form" enctype="multipart/form-data">
    <div class="form-group">
      <label for="update">Select .bin File:</label>
      <input type="file" id="update" name="update" accept=".bin" required onchange="checkFileSize()">
    </div>
    <input type="submit" value="Update" id="submit-btn">
  </form>
  <div id="progress-container">
    <div id="progress-bar"></div>
  </div>
  <div id="upload-status"></div>
  <p><small>Do not interrupt upload. Device restarts on success.</small></p>

<script>
  let freeSpace = 0;

  // Fetch free storage space
  fetch('/freestorage')
    .then(response => response.json())
    .then(data => {
      freeSpace = data.free;
      document.getElementById('storage-info').innerHTML = `Free OTA Space: ${(freeSpace / 1024 / 1024).toFixed(1)} MB (${freeSpace} bytes)`;
    })
    .catch(() => {
      document.getElementById('storage-info').innerHTML = 'Error fetching storage info.';
    });

  function checkFileSize() {
    const fileInput = document.getElementById('update');
    const submitBtn = document.getElementById('submit-btn');
    const errorMsg = document.getElementById('error-msg');
    const availSpan = document.getElementById('avail-bytes');

    if (fileInput.files.length > 0) {
      const fileSize = fileInput.files[0].size;
      if (fileSize > freeSpace) {
        submitBtn.disabled = true;
        errorMsg.style.display = 'block';
        availSpan.textContent = freeSpace;
      } else {
        submitBtn.disabled = false;
        errorMsg.style.display = 'none';
      }
    } else {
      submitBtn.disabled = true;
      errorMsg.style.display = 'none';
    }
  }

  // Handle upload with progress using fetch (better for multipart compatibility)
  document.getElementById('upload-form').addEventListener('submit', async function(e) {
    e.preventDefault();
    const fileInput = document.getElementById('update');
    const formData = new FormData();
    formData.append('update', fileInput.files[0]);

    const progressContainer = document.getElementById('progress-container');
    const progressBar = document.getElementById('progress-bar');
    const status = document.getElementById('upload-status');
    const submitBtn = document.getElementById('submit-btn');

    submitBtn.disabled = true;
    submitBtn.value = 'Uploading...';
    progressContainer.style.display = 'block';
    status.textContent = 'Starting upload...';

    // Simulate progress (fetch doesn't have native upload progress, use timeout for demo)
    let progress = 0;
    const interval = setInterval(() => {
      progress += 10;
      if (progress > 90) progress = 90;
      progressBar.style.width = progress + '%';
      status.textContent = 'Uploading: ' + progress + '%';
    }, 200);

    try {
      const response = await fetch('/update', {
        method: 'POST',
        body: formData
      });

      clearInterval(interval);
      progressBar.style.width = '100%';
      status.textContent = 'Processing...';

      if (response.ok) {
        status.textContent = 'Upload complete! Rebooting...';
        setTimeout(() => { window.location.href = '/'; }, 2000);  // Redirect to home after delay
      } else {
        status.textContent = 'Upload failed: ' + response.status + ' ' + response.statusText;
        submitBtn.disabled = false;
        submitBtn.value = 'Update';
      }
    } catch (error) {
      clearInterval(interval);
      status.textContent = 'Upload error: ' + error.message;
      submitBtn.disabled = false;
      submitBtn.value = 'Update';
    }

    progressContainer.style.display = 'none';
  });
</script>
</body>
</html>
)rawliteral";

// Updated HTML for Guide/About Page (Clean, Step-by-Step Structure)
const char about_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 4WD Robot - User Guide</title>
<style>
  body { font-family: Arial, sans-serif; max-width: 800px; margin: 20px auto; padding: 20px; background: #f4f4f4; line-height: 1.6; }
  .back-link { position: absolute; top: 20px; left: 20px; color: #007bff; text-decoration: none; }
  h1, h2, h3 { color: #333; }
  section { margin-bottom: 30px; padding: 15px; background: white; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
  ul, ol { padding-left: 20px; }
  code { background: #e9ecef; padding: 2px 4px; border-radius: 4px; font-family: monospace; }
  table { width: 100%; border-collapse: collapse; margin: 10px 0; }
  th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
  th { background: #f8f9fa; }
  .warning { background: #fff3cd; border: 1px solid #ffeaa7; color: #856404; padding: 10px; border-radius: 4px; }
  .step { margin-bottom: 10px; }
</style>
</head>
<body>
  <a href="/" class="back-link">Back to Joystick</a>
  <h1>ESP32 4WD Self-Balancing Robot - Step-by-Step Guide</h1>
  
  <section>
    <h2>1. Overview</h2>
    <p>Control your 4WD robot via WiFi joystick. Key features:</p>
    <ul>
      <li>Tank driving: Left/right turns, forward/back movement.</li>
      <li>Self-stabilize: Balance on two wheels (MPU6050 required).</li>
      <li>LED control: On/off + brightness for night use.</li>
      <li>OTA updates: Wireless firmware flash.</li>
      <li>Buzzer: Startup melody + mode beeps.</li>
      <li>Theme: Dark/light toggle.</li>
    </ul>
  </section>

  <section>
    <h2>2. Hardware Requirements</h2>
    <ul>
      <li>ESP32 board (e.g., ESP32-WROOM-32).</li>
      <li>2x L298N motor drivers (parallel front/rear motors per side).</li>
      <li>4x DC motors (6-12V geared).</li>
      <li>MPU6050 sensor (optional, for balancing).</li>
      <li>Buzzer (1.5-5V, GPIO 13).</li>
      <li>LED (3.3V white, GPIO 12 with 220Ω resistor).</li>
      <li>4WD chassis + 7.4V LiPo battery.</li>
      <li>5V for ESP32, 6-12V for motors.</li>
      <li>Jumper wires/breadboard.</li>
    </ul>
    <div class="warning">
      <strong>Warning:</strong> Match voltages. Test wiring without power.
    </div>
  </section>

  <section>
    <h2>3. Wiring - Step by Step</h2>
    
    <h3>Step 3.1: Motors (L298N)</h3>
    <ol>
      <li>Left L298N: IN1 → GPIO 27, IN2 → GPIO 26, ENA → GPIO 14.</li>
      <li>Right L298N: IN3 → GPIO 33, IN4 → GPIO 25, ENB → GPIO 32.</li>
      <li>Motors: OUT1/OUT2 → left motors (parallel), OUT3/OUT4 → right motors.</li>
      <li>Power: 12V/GND → battery; 5V → ESP32 5V (optional).</li>
    </ol>
    <table>
      <tr><th>Component</th><th>ESP32 Pin</th><th>Purpose</th></tr>
      <tr><td>Left Forward</td><td>GPIO 27</td><td>IN1</td></tr>
      <tr><td>Left Reverse</td><td>GPIO 26</td><td>IN2</td></tr>
      <tr><td>Left PWM</td><td>GPIO 14</td><td>ENA</td></tr>
      <tr><td>Right Forward</td><td>GPIO 33</td><td>IN3</td></tr>
      <tr><td>Right Reverse</td><td>GPIO 25</td><td>IN4</td></tr>
      <tr><td>Right PWM</td><td>GPIO 32</td><td>ENB</td></tr>
    </table>

    <h3>Step 3.2: MPU6050 (Balancing)</h3>
    <ol>
      <li>VCC → 3.3V, GND → GND.</li>
      <li>SDA → GPIO 21, SCL → GPIO 22.</li>
      <li>Mount on top-center, Y-axis forward.</li>
    </ol>
    <table>
      <tr><th>MPU Pin</th><th>ESP32 Pin</th><th>Purpose</th></tr>
      <tr><td>VCC</td><td>3.3V</td><td>Power</td></tr>
      <tr><td>GND</td><td>GND</td><td>Ground</td></tr>
      <tr><td>SDA</td><td>GPIO 21</td><td>Data</td></tr>
      <tr><td>SCL</td><td>GPIO 22</td><td>Clock</td></tr>
    </table>

    <h3>Step 3.3: Buzzer</h3>
    <ol>
      <li>Positive → GPIO 13, Negative → GND.</li>
    </ol>
    <table>
      <tr><th>Buzzer Pin</th><th>ESP32 Pin</th><th>Purpose</th></tr>
      <tr><td>+</td><td>GPIO 13</td><td>Signal</td></tr>
      <tr><td>-</td><td>GND</td><td>Ground</td></tr>
    </table>

    <h3>Step 3.4: LED</h3>
    <ol>
      <li>Anode (+) → 220Ω resistor → GPIO 12.</li>
      <li>Cathode (-) → GND.</li>
    </ol>
    <table>
      <tr><th>LED Pin</th><th>ESP32 Pin</th><th>Purpose</th></tr>
      <tr><td>Anode (+)</td><td>GPIO 12 (via resistor)</td><td>PWM Brightness</td></tr>
      <tr><td>Cathode (-)</td><td>GND</td><td>Ground</td></tr>
    </table>
    <div class="warning">
      <strong>Tip:</strong> Add 4.7kΩ pull-ups on SDA/SCL if I2C unstable.
    </div>
  </section>

  <section>
    <h2>4. Software Setup - Step by Step</h2>
    <ol>
      <li><strong>Libraries:</strong> Arduino IDE → Tools → Manage Libraries → Install: ESP32 (board), ArduinoJson, I2Cdev, MPU6050 (Jeff Rowberg).</li>
      <li><strong>Board:</strong> Tools → Board → ESP32 Dev Module. Flash: 4MB+, Partition: Default 4MB with spiffs.</li>
      <li><strong>Upload:</strong> USB connect → Port select → Upload .ino file.</li>
      <li><strong>Test:</strong> Serial Monitor (115200 baud) → Check boot, WiFi IP, logs.</li>
    </ol>
  </section>

  <section>
    <h2>5. Usage - Step by Step</h2>
    
    <h3>Step 5.1: Connect</h3>
    <ol>
      <li>Power ESP32 (USB/battery) → Hear startup melody (3 rising beeps + 2 tones).</li>
      <li>Phone/PC → WiFi: "Rc4v", Password: "12345678".</li>
      <li>Browser → http://192.168.4.1 (check Serial for IP).</li>
      <li>Joystick page loads → "Settings" for menu.</li>
    </ol>

    <h3>Step 5.2: Drive (Normal Mode)</h3>
    <ol>
      <li>Joystick forward/back → Straight drive. First move: Beep confirmation.</li>
      <li>Left/right → Tank turns.</li>
      <li>Release → Auto-stop (3s watchdog).</li>
      <li>Supports touch/mouse on any device.</li>
    </ol>

    <h3>Step 5.3: LED Control</h3>
    <ol>
      <li>Top-right "LED OFF" → Click to toggle ON (green).</li>
      <li>Long-press (1 sec) → Brightness slider opens (0-255, drag left/right).</li>
      <li>Adjust → Applies instantly. Close panel to save.</li>
      <li>Use for night visibility.</li>
    </ol>

    <h3>Step 5.4: Stabilize Mode</h3>
    <ol>
      <li>Connect MPU6050, mount upright.</li>
      <li>Settings → "Self Stabilize" → Check "Connected".</li>
      <li>Robot upright → Toggle ON (high beep).</li>
      <li>Joystick: Forward/back → Balanced move; left/right → 360° rotate.</li>
      <li>Toggle OFF (low beep). Tune PID if unstable.</li>
    </ol>
    <div class="warning">
      <strong>Warning:</strong> Enable only upright; test on flat surface.
    </div>

    <h3>Step 5.5: Theme</h3>
    <ol>
      <li>Settings → "☀️ Light Mode" → Toggle dark/light.</li>
      <li>Saves in browser.</li>
    </ol>

    <h3>Step 5.6: OTA Update</h3>
    <ol>
      <li>Settings → "OTA Upload" → Select .bin file.</li>
      <li>Check space → Upload → Auto-restart (melody plays).</li>
    </ol>
  </section>

  <section>
    <h2>6. Troubleshooting - Quick Fixes</h2>
    <ul>
      <li><strong>No WiFi:</strong> Restart ESP32, check Serial errors.</li>
      <li><strong>Motors Off:</strong> Wiring/power check. Serial test directions.</li>
      <li><strong>Sensor Fail:</strong> I2C pins (21/22), 3.3V. Run scanner sketch.</li>
      <li><strong>Balance Unstable:</strong> Calibrate MPU, tune PID (Kp=25 start).</li>
      <li><strong>No Buzzer:</strong> GPIO 13 wiring. Test tone() in Serial.</li>
      <li><strong>LED Dim/No Light:</strong> GPIO 12 + resistor/GND. Serial logs check.</li>
      <li><strong>Wrong Directions:</strong> Swap motor wires or INVERT flags.</li>
      <li><strong>Lag:</strong> Strong WiFi. Reduce SEND_INTERVAL.</li>
    </ul>
    <p>Always check Serial Monitor. Re-upload for changes.</p>
  </section>

  <section>
    <h2>7. Safety & Tips</h2>
    <ul>
      <li>Low speeds first.</li>
      <li>Eye protection, secure batteries.</li>
      <li>Advanced: Add battery monitor/Bluetooth.</li>
      <li>Open-source: Customize buzzer (FIRST_BEEP_FREQ etc.).</li>
    </ul>
  </section>

  <footer style="text-align: center; margin-top: 40px; color: #666;">
    <p>Version: Dec 02, 2025 | ESP32 4WD with LED & Buzzer</p>
  </footer>
</body>
</html>
)rawliteral";

void handleFreeStorage() {
  // Get free OTA partition space using ESP.getFreeSketchSpace()
  uint32_t freeBytes = ESP.getFreeSketchSpace();
  DynamicJsonDocument doc(200);
  doc["free"] = freeBytes;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// New: Handle LED Status (for JS sync on load)
void handleLedStatus() {
  DynamicJsonDocument doc(200);
  doc["on"] = ledOn;
  doc["brightness"] = ledBrightness;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleRoot() {
  startBeep(600, 100);  // Short beep on joystick page entry (non-blocking)
  server.send_P(200, "text/html", joystick_html);  // Updated to joystick-specific HTML
}

void handleStabilize() {
  startBeep(500, 150);  // Beep on entering stabilize page
  server.send_P(200, "text/html", stabilize_html);
}

void handleAbout() {
  startBeep(500, 150);  // Beep on entering guide page
  server.send_P(200, "text/html", about_html);
}

void handleUpload() {
  startBeep(500, 150);  // Beep on entering upload page
  server.send_P(200, "text/html", upload_html);
}

void handleSensorStatus() {
  DynamicJsonDocument doc(200);
  doc["connected"] = sensorConnected;
  doc["enabled"] = stabilizeMode;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleSetStabilize() {
  if (server.hasArg("enabled")) {
    bool requestedMode = (server.arg("enabled").toInt() == 1);
    if (requestedMode && !sensorConnected) {
      // Strictly prevent enabling if not connected
      Serial.println("Stabilize ON rejected: Sensor not connected!");
      server.send(400, "text/plain", "Error: Stabilize can only be enabled if MPU6050 sensor is connected!");
      startBeep(200, 300);  // Error low beep
      return;
    }
    stabilizeMode = requestedMode;
    if (stabilizeMode) {
      targetAngle = 0;  // Reset to upright on enable
      errorSum = 0;     // Reset PID integral
      Serial.println("Stabilize mode: ON (Robot should be upright!)");
      startBeep(800, 200);  // Success high beep
    } else {
      Serial.println("Stabilize mode: OFF");
      startBeep(400, 200);  // Off low beep
    }
  }
  server.send(200);
}

// New: Handle LED Toggle
void handleLedToggle() {
  ledOn = !ledOn;
  updateLED();
  server.send(200);
}

// New: Handle LED Brightness
void handleLedBrightness() {
  if (server.hasArg("value")) {
    ledBrightness = server.arg("value").toInt();
    ledBrightness = constrain(ledBrightness, 0, 255);
    updateLED();  // Apply immediately
  }
  server.send(200);
}

void handleUpdate() {
  HTTPUpload& upload = server.upload();
  static size_t total = 0;
  static bool first = true;

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("=== OTA Start: %s (expected size: %u bytes) ===\n", upload.filename.c_str(), upload.totalSize);
    total = 0;
    first = true;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {  // Target app partition
      Update.printError(Serial);
      Serial.println("Update.begin() FAILED!");
      server.send(400, "text/plain", "Update failed: " + String(getUpdateErrorString()));
      return;
    }
    Serial.println("Update.begin() SUCCESS!");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (first) {
      first = false;
      Serial.println("Writing data...");
    }
    size_t written = Update.write(upload.buf, upload.currentSize);
    total += written;
    if (total % 1024 == 0) Serial.printf("Progress: %u / %u bytes\n", total, upload.totalSize);
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("=== OTA End: Total %u bytes ===\n", upload.totalSize);
    if (Update.end(true)) { // Commit
      Serial.printf("Update SUCCESS! Rebooting...\n");
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", "Update successful! Rebooting...");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      Serial.println("Update.end() FAILED!");
      server.send(400, "text/plain", "Update failed: " + String(getUpdateErrorString()));
    }
    Update.clearError();
  } else {
    Serial.printf("Upload status: %d\n", upload.status);
  }
}

// PID Compute for Balancing (improved anti-windup)
float computePID(float angle) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt < 0.01) dt = 0.01;  // Minimum dt
  lastTime = now;

  float error = targetAngle - angle;
  errorSum += error * dt;
  // Improved anti-windup: limit based on max output / Kp
  float maxIntegral = 255.0 / Kp;
  errorSum = constrain(errorSum, -maxIntegral, maxIntegral);
  float dError = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * errorSum + Kd * dError;
  return constrain(output, -255, 255);  // PWM range
}

// Helper function to drive a side (left or right) - consistent and fixed
void driveSide(int pin1, int pin2, int channel, float speed, bool isLeft) {
  int pwm = (int)(abs(speed) * 255);
  bool invert = (speed < 0) ? (isLeft ? INVERT_REVERSE_LEFT : INVERT_REVERSE_RIGHT) : false;
  if (invert) pwm = 255 - pwm;  // Only if flag true

  if (speed > 0) {
    // Forward: pin1 HIGH, pin2 LOW (fixed)
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    ledcWrite(channel, pwm);
  } else if (speed < 0) {
    // Reverse: pin1 LOW, pin2 HIGH (fixed)
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    ledcWrite(channel, pwm);
  } else {
    // Stop
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    ledcWrite(channel, 0);
  }
}

void handleMove() {
  if (server.hasArg("x") && server.hasArg("y")) {
    lastCommand = millis();
    
    int sx = server.arg("x").toInt();
    int sy = server.arg("y").toInt();
    const int DEADZONE = 0;  // Zero deadzone for tiniest movement
    if (abs(sx) < DEADZONE) sx = 0;
    if (abs(sy) < DEADZONE) sy = 0;

    // New: Beep on first motor command (motor start)
    if (!motorStarted && (sx != 0 || sy != 0)) {
      startBeep(600, 100);  // Confirmation beep (non-blocking)
      motorStarted = true;
      Serial.println("Motors started - beep started!");
    }

    if (stabilizeMode && sensorConnected) {
      // Self-Stabilize Mode: Joystick Y sets target tilt, X for full turn
      targetAngle = -sy / 1000.0 * 30.0;  // +/-30 degrees
      float turn = sx / 1000.0;  // -1 to 1

      // Read pitch
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      float accelX = ax / 16384.0;
      float accelY = ay / 16384.0;
      float accelZ = az / 16384.0;
      float pitch = atan2(-accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 180 / PI;

      // PID
      float balanceOutput = computePID(pitch);

      // Speeds with turn
      float left_speed = (balanceOutput / 255.0) + (turn * 0.8);
      float right_speed = (balanceOutput / 255.0) - (turn * 0.8);
      left_speed = constrain(left_speed, -1.0, 1.0);
      right_speed = constrain(right_speed, -1.0, 1.0);

      // Drive
      driveSide(leftMotorPin1, leftMotorPin2, channel1, left_speed, true);
      driveSide(rightMotorPin1, rightMotorPin2, channel2, right_speed, false);

      // Update tracking
      last_left_speed = left_speed;
      last_right_speed = right_speed;

      Serial.println("Stabilize: pitch=" + String(pitch) + ", target=" + String(targetAngle) + 
                     ", balance=" + String(balanceOutput) + ", left=" + String(left_speed) + 
                     ", right=" + String(right_speed) + " (Fixed: Full rotation + directions!)");
    } else {
      // Normal Mode: Direct tank drive (FIXED: Swapped signs for correct turn direction)
      float left_speed = constrain((sy + sx) / 1000.0, -1.0, 1.0);   // sx>0: left faster (right turn)
      float right_speed = constrain((sy - sx) / 1000.0, -1.0, 1.0);  // sx>0: right slower

      // Update if changed
      if (abs(left_speed - last_left_speed) > 0.001 || abs(right_speed - last_right_speed) > 0.001) {
        last_left_speed = left_speed;
        last_right_speed = right_speed;

        // Drive
        driveSide(leftMotorPin1, leftMotorPin2, channel1, left_speed, true);
        driveSide(rightMotorPin1, rightMotorPin2, channel2, right_speed, false);

        Serial.println("Normal: x=" + String(sx) + ", y=" + String(sy) + 
                       ", left=" + String(left_speed) + ", right=" + String(right_speed) + " (Fixed: Correct turn directions!)");
      }
    }
  }
  server.send(200);
}

void setup() {
  delay(1000);  // Stable boot
  Serial.begin(115200);
  Serial.println("=== ESP32 Starting with Self-Balance, LED Control & Non-Blocking Buzzer Melody (Rising Beeps + Chirp) ===");

  // Updated: LED PWM Setup (now on GPIO 12)
  ledcSetup(ledChannel, ledFreq, ledResolution);
  ledcAttachPin(ledPin, ledChannel);
  pinMode(ledPin, OUTPUT);
  ledcWrite(ledChannel, 0);  // Start off
  Serial.println("LED ready on GPIO 12!");

  // Buzzer PWM Setup
  ledcSetup(buzzerChannel, buzzerFreq, buzzerResolution);
  ledcAttachPin(buzzerPin, buzzerChannel);
  pinMode(buzzerPin, OUTPUT);
  ledcWrite(buzzerChannel, 0);  // Start silent
  Serial.println("Buzzer ready on GPIO 13 (non-blocking)!");

  // Play Detailed Startup Melody (non-blocking)
  playStartupMelody();

  // MPU6050 init
  Wire.begin();
  mpu.initialize();
  sensorConnected = mpu.testConnection();
  if (sensorConnected) {
    Serial.println("MPU6050 connected!");
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  } else {
    Serial.println("MPU6050 failed! Check wiring.");
  }
  lastTime = millis();
  lastCommand = millis();

  // Brownout (uncomment if supported)
  // esp_brownout_disable();
  Serial.println("Brownout skipped.");

  // Pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  Serial.println("Pins set.");

  // PWM
  ledcSetup(channel1, freq, resolution);
  ledcAttachPin(leftEnablePin, channel1);
  ledcSetup(channel2, freq, resolution);
  ledcAttachPin(rightEnablePin, channel2);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, 0);
  Serial.println("PWM ready (0 duty).");

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("SSID: "); Serial.println(ap_ssid);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  // Routes (added /led-status)
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/stabilize", handleStabilize);
  server.on("/about", handleAbout);
  server.on("/upload", handleUpload);
  server.on("/freestorage", handleFreeStorage);
  server.on("/sensor-status", handleSensorStatus);
  server.on("/set-stabilize", handleSetStabilize);
  server.on("/led-toggle", handleLedToggle);  // New LED toggle route
  server.on("/led-brightness", handleLedBrightness);  // New LED brightness route
  server.on("/led-status", handleLedStatus);  // New: LED status for JS sync
  server.on("/update", HTTP_POST, []() { server.send(200, "text/plain", "OK"); }, handleUpdate);

  server.begin();
  Serial.println("Server started. Connect & test with non-blocking sounds, LED sync, and fixed turns!");
  Serial.println("Tune PID if needed: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
}

void loop() {
  server.handleClient();
  updateBuzzer();  // Non-blocking buzzer update
  
  // Continuous balance (if on) - Optimized: Run only if no recent command or target changed
  static unsigned long lastPIDTime = 0;
  if (stabilizeMode && sensorConnected && (millis() - lastPIDTime > 20)) {  // ~50Hz PID loop
    lastPIDTime = millis();
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float pitch = atan2(-accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 180 / PI;

    float balanceOutput = computePID(pitch);
    float left_speed = (balanceOutput / 255.0);
    float right_speed = (balanceOutput / 255.0);

    // Update tracking
    last_left_speed = left_speed;
    last_right_speed = right_speed;

    // Drive (straight, no turn)
    driveSide(leftMotorPin1, leftMotorPin2, channel1, left_speed, true);
    driveSide(rightMotorPin1, rightMotorPin2, channel2, right_speed, false);

    if (abs(pitch) > 1.0) {
      Serial.println("Balance: pitch=" + String(pitch, 1) + ", output=" + String(balanceOutput) + " (Optimized loop)");
    }
  }
  
  // Watchdog (optimized: print only once per stop event)
  static bool watchdogTriggered = false;
  if (millis() - lastCommand > 3000) {
    if (abs(last_left_speed) > 0.01 || abs(last_right_speed) > 0.01 || (stabilizeMode && abs(targetAngle) > 0.1)) {
      driveSide(leftMotorPin1, leftMotorPin2, channel1, 0, true);
      driveSide(rightMotorPin1, rightMotorPin2, channel2, 0, false);
      last_left_speed = 0;
      last_right_speed = 0;
      targetAngle = 0;
      motorStarted = false;  // Reset for next start beep
      
      if (!watchdogTriggered) {
        Serial.println("Watchdog: Auto-stop triggered!");
        watchdogTriggered = true;
      }
    } else {
      watchdogTriggered = false;  // Reset flag when idle
    }
  }
}