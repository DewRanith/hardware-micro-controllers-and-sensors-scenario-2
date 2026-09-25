#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// =====================================================
// PIN DEFINITIONS
// =====================================================

#define DHT_PIN         26   ///< Indoor DHT22 (primary climate sensor)
#define DHT_OUT_PIN     33   ///< Outdoor DHT22 (added for the design challenge)
#define DHT_TYPE        DHT22

#define LDR_PIN         34   ///< ADC1 input
#define POT_PIN         35   ///< ADC1 input (manual vent control)

#define SERVO1_PIN      18   ///< Vent window 1
#define SERVO2_PIN      19   ///< Vent window 2
#define SERVO3_PIN      23   ///< Vent window 3

#define LED1_PIN        25   ///< Grow LED 1
#define LED2_PIN        27   ///< Grow LED 2
#define MANUAL_LED_PIN  14   ///< Lit while in MANUAL mode
#define FAULT_LED_PIN   16   ///< Lit while in SAFETY mode

#define BUZZER_PIN      32   ///< Passive buzzer (tone made by hardware timer)

#define MODE_BTN_PIN    4    ///< Toggles AUTONOMOUS <-> MANUAL
#define RESET_BTN_PIN   13   ///< Clears a latched SAFETY fault
#define LIGHT_BTN_PIN   17   ///< Toggles grow lights in MANUAL mode
#define VENT_BTN_PIN    5    ///< Cycles vent control in MANUAL mode

#define I2C_SDA         21
#define I2C_SCL         22

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDRESS    0x3C

// =====================================================
// THRESHOLDS / CONFIGURATION
// =====================================================

/// 1 = outdoor DHT22 fitted on DHT_OUT_PIN.
/// 0 = no outdoor sensor, outdoor temp is typed in over UART ("SET OUT 5").
#define HAS_OUTDOOR_SENSOR 1

// ---- Adjustable at run time over UART:  SET <name> <value> ----
float highTemp      = 30.0;  ///< SET HIGH  - start heat venting (C)
float critTemp      = 35.0;  ///< SET CRIT  - critical heat, vents may open fully (C)
float minIndoorTemp = 18.0;  ///< SET MIN   - room too cold, never vent (C)
float coldOutside   = 10.0;  ///< SET COLD  - outdoor "dangerously cold" (C)
float highHumidity  = 85.0;  ///< SET HUM   - start humidity venting (%)
float outdoorSim    = 15.0;  ///< SET OUT   - used when HAS_OUTDOOR_SENSOR = 0
unsigned long purgeInterval = 120000; ///< SET PURGE (s) - demo 2 min, real ~30 min

// ---- Fixed ----
const float TEMP_HYST      = 1.0;   ///< Heat venting stops at highTemp - 1 C
const float HUM_HYST       = 10.0;  ///< Humidity venting stops at highHumidity - 10 %

const float TEMP_MIN_VALID = -40.0; ///< DHT22 valid range
const float TEMP_MAX_VALID = 80.0;
const float HUM_MIN_VALID  = 0.0;
const float HUM_MAX_VALID  = 100.0;

const int  LIGHT_ON_PCT  = 30;      ///< Grow LEDs ON below this light %
const int  LIGHT_OFF_PCT = 40;      ///< Grow LEDs OFF above this light %
/// true if the LDR gives HIGH readings in the dark (Wokwi module does).
/// Test: cover the LDR and watch "raw" in Serial. If it goes DOWN, set false.
const bool LDR_HIGH_MEANS_DARK = true;
/// Treat an LDR stuck at exactly 0 or 4095 as a failed sensor.
const bool CHECK_LDR_FAULT = true;

// ---- Vent positions (servo degrees) ----
const int VENT_CLOSED   = 0;
const int VENT_COLD_CAP = 15;  ///< Max opening when cold outside (mild heat)
const int VENT_HEAT_MIN = 20;  ///< First opening when heat venting starts
const int VENT_PURGE    = 25;  ///< Scheduled air-exchange purge
const int VENT_SAFE     = 30;  ///< SAFETY posture: cracked open
const int VENT_HUMID    = 35;  ///< High humidity air exchange
const int VENT_OPEN     = 90;  ///< Fully open

// =====================================================
// TIMING (all non-blocking, hardware-timer based)
// =====================================================

const unsigned long SENSOR_INTERVAL  = 2000;  ///< DHT22 minimum is 2 s
const unsigned long LDR_INTERVAL     = 200;   ///< LDR sampled 5x per second
const unsigned long MANUAL_INTERVAL  = 100;   ///< Pot is read fast in MANUAL
const unsigned long SERVO_STEP       = 20;    ///< Servo moves 1 deg per 20 ms
const unsigned long DISPLAY_INTERVAL = 250;
const unsigned long SIREN_INTERVAL   = 300;   ///< Safety siren on/off period
const unsigned long SIREN_DURATION   = 10000; ///< Full siren, then chirps
const unsigned long PURGE_DURATION   = 20000; ///< Purge lasts 20 s
const unsigned long DEBOUNCE_MS      = 250;

const int FAULT_CONFIRM_READS = 2;   ///< 2 bad DHT reads in a row (4 s) -> SAFETY
const int LDR_FAULT_READS     = 10;  ///< 10 x 200 ms = 2 s stuck at 0/4095

// =====================================================
// TYPES
// =====================================================

enum SystemMode { AUTONOMOUS, MANUAL, SAFETY };

enum ManualSource { LOCKED_OPEN, LOCKED_CLOSED, POT_CONTROL };

enum SensorFault { FAULT_NONE, FAULT_DHT_DISCONNECTED, FAULT_TEMP_RANGE,
                   FAULT_HUM_RANGE, FAULT_LDR };

/// Result of the priority logic: where the vent should go and why
struct VentDecision
{
  int angle;
  const char *reason;
};

// =====================================================
// OBJECTS
// =====================================================

DHT dht(DHT_PIN, DHT_TYPE);
DHT dhtOut(DHT_OUT_PIN, DHT_TYPE);
Servo servo1, servo2, servo3;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
hw_timer_t *hwTimer = nullptr;

// =====================================================
// STATE
// =====================================================

float temperature  = NAN;     ///< Indoor
float humidity     = NAN;     ///< Indoor
float outdoorTemp  = NAN;
bool  outdoorOk    = false;
int   lightValue   = 0;       ///< Raw LDR 0..4095
float lightPercent = 50;      ///< Filtered, 0 = dark, 100 = bright
int   ldrRailCount = 0;
int   potValue     = 0;

int   ventAngle    = 0;       ///< Where the servos are now
int   ventTarget   = 0;       ///< Where the servos are going
bool  growLightsOn = false;
const char *ventReason = "CLOSED";

bool  heatVenting  = false;   ///< Hysteresis memory
bool  humidVenting = false;   ///< Hysteresis memory
bool  criticalHeat = false;
bool  purgeActive  = false;

SystemMode   currentMode   = AUTONOMOUS;
ManualSource manualSource  = LOCKED_OPEN;
bool         manualLights  = false;
SensorFault  activeFault   = FAULT_NONE;
int          badReadCount  = 0;
bool         oledOk        = false;

unsigned long lastSensorRead    = 0;
unsigned long lastLdrRead       = 0;
unsigned long lastManualUpdate  = 0;
unsigned long lastServoStep     = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long nextPurgeTime     = 0;
unsigned long purgeEndTime      = 0;
unsigned long safetyStartTime   = 0;
unsigned long beepEndTime       = 0;

// Shared with ISRs
volatile unsigned long tickMs       = 0;     ///< 1 ms count from hardware timer
volatile bool          buzzerToneOn = false; ///< Timer ISR makes a tone while true
volatile bool          modeButtonPressed  = false;
volatile bool          resetButtonPressed = false;
volatile bool          lightButtonPressed = false;
volatile bool          ventButtonPressed  = false;
volatile unsigned long lastModeIsr        = 0;
volatile unsigned long lastResetIsr       = 0;
volatile unsigned long lastLightIsr       = 0;
volatile unsigned long lastVentIsr        = 0;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

unsigned long timerMillis();
void startHardwareTimer();
void fastWrite(uint8_t pin, bool level);
bool fastRead(uint8_t pin);
void setVentPosition(int angle);
void updateServos();
void setGrowLights(bool on);
bool isLowLight();
void readSensors();
void readLDR();
SensorFault evaluateSensors();
const char *faultSensor(SensorFault f);
const char *faultText(SensorFault f);
void checkSafety();
void tryResetSafety(const char *source);
void changeMode(SystemMode newMode, const char *reason);
const char *modeName(SystemMode m);
void handleButtons();
float heatSeverity();
void updatePurge(unsigned long now);
VentDecision decideVent();
void autonomousControl(unsigned long now);
void manualControl();
void checkSerialCommands(unsigned long now);
void handleCommand(char *cmd);
void applySetting(const char *key, float value);
void printHelp();
void printConfig();
void printStatus();
void updateOLED();
void drawHeader(const char *title, bool inverted);
void fmtValue(char *buf, float v, const char *unit);
void beep(unsigned long durationMs);
void updateBuzzer(unsigned long now);

// =====================================================
// INTERRUPT SERVICE ROUTINES & HARDWARE TIMER
// =====================================================

/**
 * Hardware timer ISR - fires every 1 ms.
 * 1) Counts time for the whole program (tickMs replaces millis()).
 * 2) Toggles the buzzer pin each tick -> 500 Hz tone on a passive buzzer.
 */
void IRAM_ATTR onTimerTick()
{
  tickMs++;

  static bool level = false;
  if (buzzerToneOn)
  {
    level = !level;
    fastWrite(BUZZER_PIN, level);
  }
  else if (level)
  {
    level = false;
    fastWrite(BUZZER_PIN, LOW);
  }
}

/** MODE button (falling edge). Debounced, and confirmed LOW by register read. */
void IRAM_ATTR onModeButton()
{
  unsigned long now = tickMs;
  if (now - lastModeIsr > DEBOUNCE_MS && !fastRead(MODE_BTN_PIN))
  {
    lastModeIsr = now;
    modeButtonPressed = true;
  }
}

/** RESET button (falling edge). */
void IRAM_ATTR onResetButton()
{
  unsigned long now = tickMs;
  if (now - lastResetIsr > DEBOUNCE_MS && !fastRead(RESET_BTN_PIN))
  {
    lastResetIsr = now;
    resetButtonPressed = true;
  }
}

/** LIGHT button (falling edge). */
void IRAM_ATTR onLightButton()
{
  unsigned long now = tickMs;
  if (now - lastLightIsr > DEBOUNCE_MS && !fastRead(LIGHT_BTN_PIN))
  {
    lastLightIsr = now;
    lightButtonPressed = true;
  }
}

/** VENT button (falling edge). */
void IRAM_ATTR onVentButton()
{
  unsigned long now = tickMs;
  if (now - lastVentIsr > DEBOUNCE_MS && !fastRead(VENT_BTN_PIN))
  {
    lastVentIsr = now;
    ventButtonPressed = true;
  }
}

/** Starts a 1 ms periodic hardware timer (works on core 2.x and 3.x). */
void startHardwareTimer()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  hwTimer = timerBegin(1000000);             // 1 MHz timer clock = 1 us per count
  timerAttachInterrupt(hwTimer, &onTimerTick);
  timerAlarm(hwTimer, 1000, true, 0);        // alarm every 1000 us, auto-reload
#else
  hwTimer = timerBegin(0, 80, true);         // timer 0, 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(hwTimer, &onTimerTick, true);
  timerAlarmWrite(hwTimer, 1000, true);      // alarm every 1000 us, auto-reload
  timerAlarmEnable(hwTimer);
#endif
}

/** System time in ms, counted by the hardware timer ISR. */
unsigned long timerMillis()
{
  return tickMs;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // Digital outputs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(MANUAL_LED_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  fastWrite(LED1_PIN, LOW);
  fastWrite(LED2_PIN, LOW);
  fastWrite(MANUAL_LED_PIN, LOW);
  fastWrite(FAULT_LED_PIN, LOW);
  fastWrite(BUZZER_PIN, LOW);

  // Buttons: wired between the pin and GND, pressed = LOW
  pinMode(MODE_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(LIGHT_BTN_PIN, INPUT_PULLUP);
  pinMode(VENT_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_BTN_PIN), onModeButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET_BTN_PIN), onResetButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIGHT_BTN_PIN), onLightButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(VENT_BTN_PIN), onVentButton, FALLING);

  analogReadResolution(12);   // ADC 0..4095

  dht.begin();
#if HAS_OUTDOOR_SENSOR
  dhtOut.begin();
#endif

  Wire.begin(I2C_SDA, I2C_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledOk)
  {
    Serial.println("OLED ERROR! Check I2C wiring. Continuing without display.");
  }

  // Servos: 50 Hz PWM, 500-2400 us pulse
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo3.attach(SERVO3_PIN, 500, 2400);
  servo1.write(VENT_CLOSED);
  servo2.write(VENT_CLOSED);
  servo3.write(VENT_CLOSED);

  startHardwareTimer();

  // Splash screen
  if (oledOk)
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawHeader("MICRO-CLIMATE", false);
    display.setCursor(0, 20);
    display.println("NURSERY SYSTEM");
    display.println();
    display.println("Starting...");
    display.display();
  }
  delay(1500); // one-off splash (also lets DHT22 power up); loop() never uses delay()

  readLDR();
  lightPercent = LDR_HIGH_MEANS_DARK ? 100.0 - lightValue * 100.0 / 4095
                                     : lightValue * 100.0 / 4095;

  Serial.println();
  Serial.println("================================================");
  Serial.println("   MICRO-CLIMATE NURSERY SYSTEM STARTED");
  Serial.println("================================================");
  printConfig();
  printHelp();

  beep(150);
  unsigned long now = timerMillis();
  lastSensorRead = now - SENSOR_INTERVAL;  // read immediately
  nextPurgeTime  = now + purgeInterval;
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  unsigned long now = timerMillis();

  checkSerialCommands(now);
  handleButtons();

  // LDR sampled fast and filtered
  if (now - lastLdrRead >= LDR_INTERVAL)
  {
    lastLdrRead = now;
    readLDR();
  }

  // Sensor cycle (every 2 s in every mode, so faults are always detected)
  if (now - lastSensorRead >= SENSOR_INTERVAL)
  {
    lastSensorRead = now;

    readSensors();
    checkSafety();

    if (currentMode == AUTONOMOUS)
    {
      autonomousControl(now);
    }

    printStatus();
  }

  // Manual override runs faster so the pot feels responsive
  if (currentMode == MANUAL && now - lastManualUpdate >= MANUAL_INTERVAL)
  {
    lastManualUpdate = now;
    manualControl();
  }

  // Servos glide 1 deg per step towards the target
  if (now - lastServoStep >= SERVO_STEP)
  {
    lastServoStep = now;
    updateServos();
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL)
  {
    lastDisplayUpdate = now;
    updateOLED();
  }

  updateBuzzer(now);
}

// =====================================================
// LOW-LEVEL OUTPUT HELPERS
// =====================================================

/**
 * Register-level GPIO write.
 * Writing a bit to W1TS sets that pin HIGH, to W1TC sets it LOW.
 * Pins 0-31 are in bank 0, pins 32-39 in bank 1 (GPIO_OUT1_...).
 * In IRAM so the timer ISR can call it.
 */
void IRAM_ATTR fastWrite(uint8_t pin, bool level)
{
  if (pin < 32)
  {
    uint32_t mask = 1UL << pin;
    REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, mask);
  }
  else
  {
    uint32_t mask = 1UL << (pin - 32);
    REG_WRITE(level ? GPIO_OUT1_W1TS_REG : GPIO_OUT1_W1TC_REG, mask);
  }
}

/** Register-level GPIO read (input register). */
bool IRAM_ATTR fastRead(uint8_t pin)
{
  if (pin < 32) return (REG_READ(GPIO_IN_REG) >> pin) & 1UL;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1UL;
}

/** Sets where the vents should go. updateServos() moves them there smoothly. */
void setVentPosition(int angle)
{
  ventTarget = constrain(angle, VENT_CLOSED, VENT_OPEN);
}

/** Moves all 3 vent servos 1 deg towards ventTarget (PWM). */
void updateServos()
{
  if (ventAngle == ventTarget) return;

  ventAngle += (ventTarget > ventAngle) ? 1 : -1;

  servo1.write(ventAngle);
  servo2.write(ventAngle);
  servo3.write(ventAngle);
}

void setGrowLights(bool on)
{
  fastWrite(LED1_PIN, on);
  fastWrite(LED2_PIN, on);
  growLightsOn = on;
}

/** Low light with hysteresis: ON below 30 %, OFF above 40 %, else keep state. */
bool isLowLight()
{
  if (lightPercent < LIGHT_ON_PCT)  return true;
  if (lightPercent > LIGHT_OFF_PCT) return false;
  return growLightsOn;
}

// =====================================================
// SENSORS & SAFETY
// =====================================================

void readSensors()
{
  temperature = dht.readTemperature();
  humidity    = dht.readHumidity();

#if HAS_OUTDOOR_SENSOR
  outdoorTemp = dhtOut.readTemperature();
  outdoorOk   = !isnan(outdoorTemp) && outdoorTemp >= TEMP_MIN_VALID
                                    && outdoorTemp <= TEMP_MAX_VALID;
#else
  outdoorTemp = outdoorSim;
  outdoorOk   = true;
#endif

  potValue = analogRead(POT_PIN);
}

/** Reads the LDR, converts to light %, and smooths it (low-pass filter). */
void readLDR()
{
  lightValue = analogRead(LDR_PIN);

  float pct = lightValue * 100.0 / 4095.0;
  if (LDR_HIGH_MEANS_DARK) pct = 100.0 - pct;
  lightPercent = 0.7 * lightPercent + 0.3 * pct;

  // A reading stuck at the very top or bottom = broken / shorted wire
  if (lightValue <= 0 || lightValue >= 4095)
  {
    if (ldrRailCount < 1000) ldrRailCount++;
  }
  else
  {
    ldrRailCount = 0;
  }
}

SensorFault evaluateSensors()
{
  if (isnan(temperature) || isnan(humidity))
    return FAULT_DHT_DISCONNECTED;

  if (temperature < TEMP_MIN_VALID || temperature > TEMP_MAX_VALID)
    return FAULT_TEMP_RANGE;

  if (humidity < HUM_MIN_VALID || humidity > HUM_MAX_VALID)
    return FAULT_HUM_RANGE;

  if (CHECK_LDR_FAULT && ldrRailCount >= LDR_FAULT_READS)
    return FAULT_LDR;

  return FAULT_NONE;
}

/** Which sensor failed (short, fits on the OLED). */
const char *faultSensor(SensorFault f)
{
  switch (f)
  {
    case FAULT_DHT_DISCONNECTED:
    case FAULT_TEMP_RANGE:
    case FAULT_HUM_RANGE:        return "INDOOR DHT22";
    case FAULT_LDR:              return "LDR SENSOR";
    default:                     return "NONE";
  }
}

/** What went wrong. */
const char *faultText(SensorFault f)
{
  switch (f)
  {
    case FAULT_DHT_DISCONNECTED: return "NO DATA (NaN)";
    case FAULT_TEMP_RANGE:       return "TEMP INVALID";
    case FAULT_HUM_RANGE:        return "HUMID INVALID";
    case FAULT_LDR:              return "STUCK 0/4095";
    default:                     return "NONE";
  }
}

/**
 * Runs every sensor cycle in EVERY mode.
 * Needs FAULT_CONFIRM_READS bad reads in a row, so one glitch
 * doesn't trip the alarm (2 reads = 4 s, still immediate for a greenhouse).
 */
void checkSafety()
{
  SensorFault f = evaluateSensors();

  if (f == FAULT_NONE)
  {
    badReadCount = 0;
    return;
  }

  badReadCount++;

  if (badReadCount >= FAULT_CONFIRM_READS && currentMode != SAFETY)
  {
    activeFault = f;
    Serial.print("!!! SENSOR FAULT: ");
    Serial.print(faultSensor(f));
    Serial.print(" - ");
    Serial.print(faultText(f));
    Serial.println(" !!!");
    changeMode(SAFETY, faultText(f));
  }
}

void tryResetSafety(const char *source)
{
  Serial.print(">>> RESET requested via ");
  Serial.println(source);

  if (currentMode != SAFETY)
  {
    Serial.println("    System is not in SAFETY mode - nothing to reset.");
    return;
  }

  // Take a fresh reading before deciding
  readSensors();
  SensorFault f = evaluateSensors();

  if (f == FAULT_NONE)
  {
    activeFault  = FAULT_NONE;
    badReadCount = 0;
    changeMode(AUTONOMOUS, "Operator reset, sensor OK");
  }
  else
  {
    activeFault = f;
    Serial.print("    RESET REFUSED - fault still present: ");
    Serial.print(faultSensor(f));
    Serial.print(" - ");
    Serial.println(faultText(f));
    Serial.println("    (DHT22 needs 2 s between reads - wait and retry)");
  }
}

// =====================================================
// MODE MANAGEMENT
// =====================================================

const char *modeName(SystemMode m)
{
  switch (m)
  {
    case AUTONOMOUS: return "AUTONOMOUS";
    case MANUAL:     return "MANUAL OVERRIDE";
    default:         return "SAFETY";
  }
}

void changeMode(SystemMode newMode, const char *reason)
{
  if (newMode == currentMode) return;

  SystemMode oldMode = currentMode;
  currentMode = newMode;

  // Leaving SAFETY: silence the siren, fault LED off
  if (oldMode == SAFETY)
  {
    buzzerToneOn = false;
    fastWrite(FAULT_LED_PIN, LOW);
  }

  Serial.println();
  Serial.println("################################################");
  Serial.print  ("  MODE CHANGE: ");
  Serial.print  (modeName(oldMode));
  Serial.print  (" -> ");
  Serial.println(modeName(newMode));
  Serial.print  ("  Reason: ");
  Serial.println(reason);
  Serial.println("################################################");

  switch (newMode)
  {
    case MANUAL:
      fastWrite(MANUAL_LED_PIN, HIGH);
      manualSource = LOCKED_OPEN;    // brief: workers lock vents OPEN
      manualLights = growLightsOn;   // keep current lights state
      Serial.println("==============================================");
      Serial.println("        *** MANUAL OVERRIDE MODE ***");
      Serial.println("  Automation SUSPENDED - operator in control");
      Serial.println("  Vents LOCKED OPEN");
      Serial.println("  VENT button  = OPEN -> CLOSED -> POT -> OPEN");
      Serial.println("  LIGHT button = toggle grow lights");
      Serial.println("  MODE button  = return to AUTO");
      Serial.println("==============================================");
      beep(120);
      manualControl();
      break;

    case AUTONOMOUS:
      fastWrite(MANUAL_LED_PIN, LOW);
      heatVenting  = false;
      humidVenting = false;
      purgeActive  = false;
      nextPurgeTime = timerMillis() + purgeInterval;
      Serial.println("==============================================");
      Serial.println("        *** AUTONOMOUS MODE ***");
      Serial.println("  LDR controls grow lights, DHT22s control vents");
      Serial.println("==============================================");
      beep(120);
      autonomousControl(timerMillis());   // act on current readings straight away
      break;

    case SAFETY:
      fastWrite(MANUAL_LED_PIN, LOW);
      fastWrite(FAULT_LED_PIN, HIGH);
      // Safe posture: vents cracked open 30 deg, grow lights off.
      // Closed could overheat the crop in a midday spike we can't see;
      // fully open could chill it. 30 deg keeps air moving with limited
      // exposure. LEDs off = no extra heat while temperature is unknown.
      purgeActive = false;
      setVentPosition(VENT_SAFE);
      setGrowLights(false);
      ventReason = "SAFE POSTURE";
      safetyStartTime = timerMillis();
      Serial.println("==============================================");
      Serial.println("        !!! SAFETY MODE !!!");
      Serial.print  ("  Failed sensor: ");
      Serial.print  (faultSensor(activeFault));
      Serial.print  (" - ");
      Serial.println(faultText(activeFault));
      Serial.println("  Vents 30 deg (safe), grow lights OFF");
      Serial.println("  Fix the sensor, then press RESET");
      Serial.println("==============================================");
      break;
  }
}

void handleButtons()
{
  if (modeButtonPressed)
  {
    modeButtonPressed = false;
    Serial.println("[BUTTON] MODE pressed");

    if (currentMode == AUTONOMOUS)
      changeMode(MANUAL, "MODE button");
    else if (currentMode == MANUAL)
      changeMode(AUTONOMOUS, "MODE button");
    else
      Serial.println("    Ignored - system is in SAFETY. Press RESET first.");
  }

  if (resetButtonPressed)
  {
    resetButtonPressed = false;
    Serial.println("[BUTTON] RESET pressed");
    tryResetSafety("RESET button");
  }

  if (ventButtonPressed)
  {
    ventButtonPressed = false;
    Serial.println("[BUTTON] VENT pressed");

    if (currentMode == MANUAL)
    {
      // Cycle: LOCKED OPEN -> LOCKED CLOSED -> POT CONTROL -> LOCKED OPEN
      if      (manualSource == LOCKED_OPEN)   manualSource = LOCKED_CLOSED;
      else if (manualSource == LOCKED_CLOSED) manualSource = POT_CONTROL;
      else                                    manualSource = LOCKED_OPEN;

      manualControl();
      Serial.print("    Vents now: ");
      Serial.println(ventReason);
    }
    else
    {
      Serial.println("    Ignored - VENT button only works in MANUAL mode.");
    }
  }

  if (lightButtonPressed)
  {
    lightButtonPressed = false;
    Serial.println("[BUTTON] LIGHT pressed");

    if (currentMode == MANUAL)
    {
      manualLights = !manualLights;
      Serial.printf("    Grow lights %s\n", manualLights ? "ON" : "OFF");
      manualControl();
    }
    else
    {
      Serial.println("    Ignored - LIGHT button only works in MANUAL mode.");
    }
  }
}

// =====================================================
// AUTONOMOUS MODE  -  PRIORITY LOGIC HIERARCHY
// =====================================================
//
//  Highest priority first. A higher level always wins.
//
//   1 SAFETY     sensor fault            -> fixed safe posture   (checkSafety)
//   2 MANUAL     operator override       -> operator decides     (manualControl)
//   3 TOO COLD   indoor <= minIndoorTemp -> vents CLOSED, no matter what
//   4 COLD LIMIT outdoor cold OR outdoor sensor failed
//                -> vent capped at 15 + severity^2 * 75 deg
//   5 HEAT       indoor >= highTemp      -> proportional 20..90 deg
//   6 HUMIDITY   RH >= highHumidity      -> 35 deg
//   7 PURGE      timed schedule          -> 25 deg for 20 s
//   8 DEFAULT                            -> closed (keep warmth in)
//
//  Levels 5-7 REQUEST an angle: the biggest request wins.
//  Levels 3-4 then LIMIT that angle.
//  severity = 0 at highTemp, 1 at critTemp.
// =====================================================

/** 0.0 at highTemp ... 1.0 at critTemp */
float heatSeverity()
{
  return constrain((temperature - highTemp) / (critTemp - highTemp), 0.0f, 1.0f);
}

/** Scheduled behaviour: air-exchange purge against stagnant air pockets. */
void updatePurge(unsigned long now)
{
  if (!purgeActive && (long)(now - nextPurgeTime) >= 0)
  {
    purgeActive  = true;
    purgeEndTime = now + PURGE_DURATION;
    Serial.println("[SCHEDULE] Air-exchange purge STARTED");
  }

  if (purgeActive && (long)(now - purgeEndTime) >= 0)
  {
    purgeActive   = false;
    nextPurgeTime = now + purgeInterval;
    Serial.println("[SCHEDULE] Air-exchange purge FINISHED");
  }
}

VentDecision decideVent()
{
  // ---- Level 3: room already too cold -> never let cold air in ----
  if (temperature <= minIndoorTemp)
  {
    return {VENT_CLOSED, "TOO COLD"};
  }

  int angle = VENT_CLOSED;
  const char *why = "CLOSED";

  // ---- Level 5: heat (hysteresis stops the vent fluttering at 30.0 C) ----
  if (temperature >= highTemp)                   heatVenting = true;
  else if (temperature <= highTemp - TEMP_HYST)  heatVenting = false;

  // Venting only cools if outside is cooler than inside
  bool outsideCooler = !outdoorOk || outdoorTemp < temperature - 1.0;

  if (heatVenting && outsideCooler)
  {
    angle = VENT_HEAT_MIN + heatSeverity() * (VENT_OPEN - VENT_HEAT_MIN);
    why   = criticalHeat ? "CRIT HEAT" : "HEAT";
  }
  else if (heatVenting)
  {
    why = "OUT HOTTER";   // opening would bring heat IN
  }

  // ---- Level 6: humidity / stagnant air ----
  if (humidity >= highHumidity)                      humidVenting = true;
  else if (humidity <= highHumidity - HUM_HYST)      humidVenting = false;

  if (humidVenting && VENT_HUMID > angle)
  {
    angle = VENT_HUMID;
    why   = "HUMIDITY";
  }

  // ---- Level 7: scheduled purge ----
  if (purgeActive && VENT_PURGE > angle)
  {
    angle = VENT_PURGE;
    why   = "PURGE";
  }

  // ---- Level 4: cold outside (unknown outdoor = assume the worst) ----
  bool coldOut = !outdoorOk || outdoorTemp <= coldOutside;

  if (coldOut && angle > VENT_CLOSED)
  {
    float s = heatSeverity();
    int cap = VENT_COLD_CAP + s * s * (VENT_OPEN - VENT_COLD_CAP);
    if (angle > cap)
    {
      angle = cap;
      why   = "COLD LIMIT";
    }
  }

  return {angle, why};
}

void autonomousControl(unsigned long now)
{
  criticalHeat = temperature >= critTemp;

  updatePurge(now);

  // ---- Vents ----
  VentDecision d = decideVent();

  if (d.angle != ventTarget)
  {
    Serial.printf("[VENT] %d -> %d deg (%s)\n", ventTarget, d.angle, d.reason);
    if (d.angle == VENT_OPEN) beep(200);   // audible heat alert
  }
  setVentPosition(d.angle);
  ventReason = d.reason;

  // ---- Lights ----
  bool lights = isLowLight();
  if (criticalHeat) lights = false;         // shed LED heat in a heat emergency
  setGrowLights(lights);

  if (criticalHeat)
  {
    Serial.println("[ALERT] CRITICAL HEAT - vents opening, grow LEDs off");
    beep(150);
  }
}

// =====================================================
// MANUAL MODE
// =====================================================

void manualControl()
{
  int target;
  switch (manualSource)
  {
    case LOCKED_OPEN:   target = VENT_OPEN;   ventReason = "LOCKED OPEN";   break;
    case LOCKED_CLOSED: target = VENT_CLOSED; ventReason = "LOCKED CLOSED"; break;
    default:
      potValue = analogRead(POT_PIN);
      target = map(potValue, 0, 4095, VENT_CLOSED, VENT_OPEN);
      ventReason = "POT";
      break;
  }

  // Ignore 1-degree jitter from the ADC
  if (abs(target - ventTarget) > 1 || target == VENT_CLOSED || target == VENT_OPEN)
  {
    setVentPosition(target);
  }

  if (growLightsOn != manualLights) setGrowLights(manualLights);
}

// =====================================================
// SERIAL (UART)
// =====================================================

void printHelp()
{
  Serial.println("Buttons:");
  Serial.println("  MODE  (GPIO4)  = Autonomous <-> Manual");
  Serial.println("  RESET (GPIO13) = clear sensor fault");
  Serial.println("  LIGHT (GPIO17) = grow lights on/off        (manual)");
  Serial.println("  VENT  (GPIO5)  = OPEN -> CLOSED -> POT     (manual)");
  Serial.println("Config (type in Serial Monitor):");
  Serial.println("  SET HIGH 30 | SET CRIT 35 | SET MIN 18 | SET COLD 10");
  Serial.println("  SET HUM 85  | SET PURGE 120 (s) | SET OUT 5 (sim outdoor C)");
}

void printConfig()
{
  Serial.printf("CONFIG: high=%.1fC crit=%.1fC min=%.1fC cold=%.1fC hum=%.0f%% purge=%lus\n",
                highTemp, critTemp, minIndoorTemp, coldOutside, highHumidity,
                purgeInterval / 1000);
}

void printStatus()
{
  Serial.println();
  if (currentMode == MANUAL)
    Serial.println("======== MANUAL OVERRIDE - AUTOMATION OFF ========");
  else if (currentMode == SAFETY)
    Serial.println("!!!!!!!!!!!!!!!! SAFETY MODE ACTIVE !!!!!!!!!!!!!!!!");
  else
    Serial.println("---------------- AUTONOMOUS STATUS ----------------");

  Serial.printf("Indoor      : %.1f C  %.1f %%\n", temperature, humidity);
  if (outdoorOk)
    Serial.printf("Outdoor     : %.1f C\n", outdoorTemp);
  else
    Serial.println("Outdoor     : SENSOR FAULT (assuming cold)");
  Serial.printf("Light (LDR) : %d%%  raw %d  (%s)\n", (int)lightPercent, lightValue,
                isLowLight() ? "LOW" : "OK");
  Serial.printf("Pot         : %d\n", potValue);
  Serial.printf("Vents       : %d deg -> %d deg  [%s]\n", ventAngle, ventTarget, ventReason);
  Serial.printf("Grow LEDs   : %s\n", growLightsOn ? "ON" : "OFF");
  Serial.printf("Mode        : %s\n", modeName(currentMode));
  if (currentMode == AUTONOMOUS)
  {
    unsigned long now = timerMillis();
    if (purgeActive) Serial.printf("Purge       : ACTIVE (%lus left)\n", (purgeEndTime - now) / 1000);
    else             Serial.printf("Purge       : next in %lus\n", (nextPurgeTime - now) / 1000);
  }
  if (currentMode == SAFETY)
    Serial.printf("FAULT       : %s - %s -> press RESET\n",
                  faultSensor(activeFault), faultText(activeFault));
}

/**
 * Collects characters into a line without blocking.
 * A command runs on Enter, or after 300 ms of no typing
 * (so it works even with "No line ending" in the Serial Monitor).
 */
void checkSerialCommands(unsigned long now)
{
  static char buffer[32];
  static uint8_t length = 0;
  static unsigned long lastCharTime = 0;

  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (length > 0)
      {
        buffer[length] = '\0';
        handleCommand(buffer);
        length = 0;
      }
    }
    else if (length < sizeof(buffer) - 1)
    {
      buffer[length++] = c;
      lastCharTime = now;
    }
  }

  if (length > 0 && now - lastCharTime > 300)
  {
    buffer[length] = '\0';
    handleCommand(buffer);
    length = 0;
  }
}

void handleCommand(char *cmd)
{
  for (char *p = cmd; *p; p++) *p = toupper(*p);

  Serial.print("[SERIAL] Command: ");
  Serial.println(cmd);

  // ---- Configuration: SET <name> <value> ----
  char key[8];
  float value;
  if (sscanf(cmd, "SET %7s %f", key, &value) == 2)
  {
    applySetting(key, value);
    return;
  }

  Serial.println("    Unknown command. Use: SET <name> <value>  (e.g. SET HIGH 28)");
}

/** Changes a threshold at run time. Values are range-checked. */
void applySetting(const char *key, float value)
{
  bool ok = true;

  if      (!strcmp(key, "HIGH"))  { if (value > minIndoorTemp && value < critTemp) highTemp = value; else ok = false; }
  else if (!strcmp(key, "CRIT"))  { if (value > highTemp && value <= 60)           critTemp = value; else ok = false; }
  else if (!strcmp(key, "MIN"))   { if (value >= 0 && value < highTemp)            minIndoorTemp = value; else ok = false; }
  else if (!strcmp(key, "COLD"))  { if (value >= -20 && value <= 25)               coldOutside = value; else ok = false; }
  else if (!strcmp(key, "HUM"))   { if (value >= 50 && value <= 99)                highHumidity = value; else ok = false; }
  else if (!strcmp(key, "PURGE")) { if (value >= 30 && value <= 7200)              purgeInterval = (unsigned long)value * 1000UL; else ok = false; }
  else if (!strcmp(key, "OUT"))   { outdoorSim = value; }
  else
  {
    Serial.println("    Unknown setting. Use HIGH, CRIT, MIN, COLD, HUM, PURGE or OUT.");
    return;
  }

  if (ok)
  {
    Serial.print("    OK. ");
    printConfig();
  }
  else
  {
    Serial.println("    REJECTED - value out of allowed range.");
  }
}

// =====================================================
// OLED (I2C)
// =====================================================

void drawHeader(const char *title, bool inverted)
{
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;

  if (inverted)
  {
    display.fillRect(0, 0, SCREEN_WIDTH, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  else
  {
    display.drawRect(0, 0, SCREEN_WIDTH, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }

  display.setCursor(x, 2);
  display.print(title);
  display.setTextColor(SSD1306_WHITE);
}

/** Formats a reading, or "--" if it is NaN (failed sensor). */
void fmtValue(char *buf, float v, const char *unit)
{
  if (isnan(v)) sprintf(buf, "--%s", unit);
  else          sprintf(buf, "%.1f%s", v, unit);
}

void updateOLED()
{
  if (!oledOk) return;

  char t[10], h[10], o[10];
  fmtValue(t, temperature, "C");
  fmtValue(h, humidity, "%");
  fmtValue(o, outdoorTemp, "C");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (currentMode == MANUAL)
  {
    // ---------- MANUAL SCREEN ----------
    drawHeader("MANUAL OVERRIDE", true);

    display.setCursor(0, 14);
    display.print("AUTOMATION SUSPENDED");

    display.setCursor(0, 25);
    display.printf("T:%s  H:%s", t, h);

    display.setCursor(0, 36);
    display.printf("Light:%d%% LEDs:%s", (int)lightPercent, growLightsOn ? "ON" : "OFF");

    display.setCursor(0, 46);
    display.printf("Vent:%d %s", ventAngle, ventReason);

    display.setCursor(0, 56);
    display.print("LIGHT btn=LEDs on/off");
  }
  else if (currentMode == SAFETY)
  {
    // ---------- SAFETY SCREEN (header flashes) ----------
    bool flash = (timerMillis() / 500) % 2;
    drawHeader("!! SENSOR FAULT !!", flash);

    display.setCursor(0, 14);
    display.printf("Failed:%s", faultSensor(activeFault));

    display.setCursor(0, 24);
    display.print(faultText(activeFault));

    display.setCursor(0, 35);
    display.printf("T:%s H:%s L:%d%%", t, h, (int)lightPercent);

    display.setCursor(0, 45);
    display.printf("Vent:%d SAFE LED:OFF", ventAngle);

    display.setCursor(0, 56);
    display.print("Fix sensor > RESET");
  }
  else
  {
    // ---------- AUTONOMOUS SCREEN ----------
    drawHeader("AUTO MODE", false);

    display.setCursor(0, 14);
    display.printf("T:%s  H:%s", t, h);

    display.setCursor(0, 24);
    if (outdoorOk) display.printf("Out:%s", o);
    else           display.print("Out:SENSOR FAULT");

    display.setCursor(0, 34);
    display.printf("Light:%d%% LEDs:%s", (int)lightPercent, growLightsOn ? "ON" : "OFF");

    display.setCursor(0, 44);
    display.printf("Vent:%d %s", ventAngle, ventReason);

    display.setCursor(0, 55);
    unsigned long now = timerMillis();
    if (criticalHeat)     display.print("!! CRITICAL HEAT !!");
    else if (purgeActive) display.printf("Purging: %lus", (purgeEndTime - now) / 1000);
    else                  display.printf("Next purge: %lus", (nextPurgeTime - now) / 1000);
  }

  display.display();
}

// =====================================================
// BUZZER (non-blocking, tone made by the timer ISR)
// =====================================================

void beep(unsigned long durationMs)
{
  if (currentMode == SAFETY) return;
  beepEndTime = timerMillis() + durationMs;
}

void updateBuzzer(unsigned long now)
{
  if (currentMode == SAFETY)
  {
    // Siren for 10 s, then a short chirp every 5 s until RESET
    unsigned long t = now - safetyStartTime;
    if (t < SIREN_DURATION) buzzerToneOn = ((now / SIREN_INTERVAL) % 2) == 0;
    else                    buzzerToneOn = (t % 5000) < 150;
    return;
  }

  buzzerToneOn = (long)(beepEndTime - now) > 0;
}

