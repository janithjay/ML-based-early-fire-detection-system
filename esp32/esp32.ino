/*
  ESP32 Fire Detection - TFLite INT8 Model with Firebase Integration
  Using Firebase Database Secret (Legacy Token) for authentication
  WITH BUZZER ALERT SYSTEM
  
  Libraries needed:
  - Firebase ESP Client by Mobizt
  - DFRobot_ENS160
  - Adafruit AHTX0
  - Adafruit BMP085 Unified
  - LiquidCrystal I2C
  - TensorFlow Lite Micro
  - Time library (built-in)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP085_U.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <time.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tiny_model_int8.h"

// ==================== WiFi Configuration ====================
#define WIFI_SSID "Lord of the pings"
#define WIFI_PASSWORD "15448192"

// ==================== Firebase Configuration ====================
#define DATABASE_URL "https://firely-50a30-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define DATABASE_SECRET "h6iRd4cv0jcYsaYs3k8Im2u85N3nrgsfMkXQsOTE"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

// ==================== TFLite Configuration ====================
constexpr int kTensorArenaSize = 12 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* model = nullptr;
tflite::MicroMutableOpResolver<4> resolver;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// Quantization parameters
const float INPUT_SCALE = 0.02470421977341175f;
const int INPUT_ZERO_POINT = -23;
const float OUTPUT_SCALE = 0.00390625f;
const int OUTPUT_ZERO_POINT = -128;

// Standardization parameters
const float FEATURE_MEANS[6] = {
    1290.20526961f, 53.63631331f, 1006.08489930f, 2993.15093954f, 115.52450980f, 41.12098688f
};

const float FEATURE_STDS[6] = {
    2689.46756645f, 15.91762949f, 0.21825710f, 275.28715973f, 100.50365919f, 9.70326020f
};

// ====== SENSOR OBJECTS ======
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
Adafruit_AHTX0 aht;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
LiquidCrystal_I2C lcd(0x27, 16, 4);

// ====== PIN DEFINITIONS ======
#define MQ3_PIN 34
#define MQ8_PIN 35
#define BUZZER_PIN 25  // Buzzer connected to GPIO 25
#define TEST_BUTTON_PIN 32  // Push button for demo mode
#define CALIBRATE_BUTTON_PIN 33  // Push button for calibration mode

// ====== BUZZER VARIABLES ======
bool buzzerState = false;
unsigned long lastBuzzerToggle = 0;
const unsigned long buzzerBeepInterval = 500;  // 500ms on, 500ms off for beep pattern

// ====== SENSOR VARIABLES ======
sensors_event_t humidity_event, temp_event, pressure_event;
int mq8_raw = 0;
int mq3_raw = 0;

// ====== TIMING VARIABLES ======
unsigned long lastInferenceTime = 0;
const unsigned long inferenceInterval = 2000;  // 2 seconds

// ====== NTP TIME CONFIGURATION ======
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 5*3600;  // Set to your timezone offset (e.g., 5*3600 for UTC+5:30)
const int daylightOffset_sec = 0;

// ====== FIRE DETECTION STATE ======
bool fireDetected = false;

// ====== DEMO MODE VARIABLES ======
bool demoMode = false;
int demoDataIndex = 0;
unsigned long lastDemoUpdate = 0;
const unsigned long demoUpdateInterval = 3000;  // Update every 3 seconds in demo mode

// Dummy fire detection data (8 samples from your dataset)
const float DEMO_DATA[8][7] = {
  // eco2,  humidity, pressure, raw_ethanol, raw_h2, temperature, fire_alarm
  {3936, 61.81192, 1006.01, 3032, 293, 39.37054, 1},
  {5753, 61.47594, 1006.00, 3036, 292, 39.87541, 1},
  {4392, 60.59647, 1005.95, 3056, 316, 39.98394, 1},
  {2954, 59.82027, 1005.99, 3072, 324, 40.03391, 1},
  {2192, 59.21974, 1005.99, 3091, 321, 40.08026, 1},
  {1802, 58.63543, 1005.93, 3091, 318, 40.05241, 1},
  {1746, 58.03804, 1005.94, 3098, 306, 40.02628, 1},
  {1513, 57.38468, 1005.94, 3109, 302, 40.02762, 1}
};

const int DEMO_DATA_COUNT = 8;

// ====== CALIBRATION VARIABLES ======
bool isCalibrated = false;
bool calibrationMode = false;
int calibrationSampleCount = 0;
const int CALIBRATION_SAMPLES = 20;  // Collect 20 samples over 40 seconds
float calibrationSums[6] = {0, 0, 0, 0, 0, 0};  // Sum of samples for averaging
float calibrationOffsets[6] = {0, 0, 0, 0, 0, 0};  // Final offset values

// Original baseline from training data (fire_alarm = 0 conditions)
const float BASELINE_MEANS[6] = {
    594.73,      // eco2 (average of non-fire samples)
    65.17,       // humidity
    1006.09,     // pressure
    2824.09,     // raw_ethanol
    58.25,      // raw_h2
    34.23        // temperature
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===========================================");
  Serial.println("ESP32 Fire Detection System");
  Serial.println("TFLite + Firebase Integration");
  Serial.println("===========================================\n");

  // Initialize Buzzer Pin (OFF by default)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Buzzer initialized (OFF)");
  
  // Initialize Test Button Pin with internal pull-up
  pinMode(TEST_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Test button initialized (GPIO 32)");
  Serial.println("Press button to start DEMO MODE with fire detection data");
  
  // Initialize Calibration Button Pin with internal pull-up
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Calibration button initialized (GPIO 33)");
  Serial.println("Press and HOLD for 3 seconds to start CALIBRATION");

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fire Detection");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1000);

  // ====== WiFi Connection ======
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected!");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);

  // ====== Sync Time with NTP ======
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Syncing Time...");
  
  Serial.println("Syncing time with NTP server...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // Wait for time to be set
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 24 * 3600 && attempts < 20) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    attempts++;
  }
  Serial.println();
  
  struct tm timeinfo = *localtime(&now);
  Serial.print("Current time: ");
  Serial.println(asctime(&timeinfo));
  
  lcd.setCursor(0, 1);
  lcd.print("Time Synced!");
  delay(1000);

  // ====== Initialize I2C and GPIO ======
  Wire.begin(21, 22);
  Wire.setClock(100000);
  pinMode(MQ3_PIN, INPUT);
  pinMode(MQ8_PIN, INPUT);
  analogSetPinAttenuation(MQ3_PIN, ADC_11db);
  analogSetPinAttenuation(MQ8_PIN, ADC_11db);
  Serial.println("I2C and GPIO initialized\n");

  // ====== Initialize Sensors ======
  Serial.println("Initializing sensors...");
  
  if (!aht.begin()) {
    Serial.println("AHT Error!");
    lcd.clear();
    lcd.print("AHT Error!");
    while (1) delay(1000);
  }
  Serial.println("- AHT OK");

  if (!bmp.begin()) {
    Serial.println("BMP Error!");
    lcd.clear();
    lcd.print("BMP Error!");
    while (1) delay(1000);
  }
  Serial.println("- BMP OK");

  if (ens160.begin() != NO_ERR) {
    Serial.println("ENS160 Error!");
    lcd.clear();
    lcd.print("ENS160 Error!");
    while (1) delay(1000);
  }
  Serial.println("- ENS160 OK");
  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(25.0, 50.0);
  Serial.println("All sensors OK\n");

  // ====== Initialize TFLite Model ======
  Serial.println("Loading TFLite model...");
  
  model = tflite::GetModel(tiny_model_int8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model version mismatch!");
    lcd.clear();
    lcd.print("Model Error!");
    while (1) delay(1000);
  }

  resolver.AddFullyConnected();
  resolver.AddLogistic();
  resolver.AddQuantize();
  resolver.AddDequantize();

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Tensor allocation failed!");
    lcd.clear();
    lcd.print("Tensor Error!");
    while (1) delay(1000);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
  Serial.printf("TFLite OK - Arena used: %d/%d bytes\n\n", 
                interpreter->arena_used_bytes(), kTensorArenaSize);

  // ====== Firebase Connection (Using Database Secret) ======
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Init Firebase...");
  
  Serial.println("Initializing Firebase...");
  
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  fbdo.setBSSLBufferSize(1024, 1024);
  
  Serial.println("Firebase initialized!");
  Serial.println("Using legacy database authentication");
  Serial.println();
  
  lcd.setCursor(0, 1);
  lcd.print("Firebase OK!");
  delay(2000);

  // ====== System Ready ======
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Monitoring...");
  delay(2000);
  lcd.clear();

  Serial.println("===========================================");
  Serial.println("System Started - Running every 2 seconds");
  Serial.println("===========================================\n");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check for calibration button (hold for 3 seconds)
  static unsigned long calibrateButtonPressTime = 0;
  static bool calibrateButtonWasPressed = false;
  bool calibrateButtonState = digitalRead(CALIBRATE_BUTTON_PIN);
  
  if (calibrateButtonState == LOW && !calibrateButtonWasPressed) {
    // Button just pressed
    calibrateButtonPressTime = currentMillis;
    calibrateButtonWasPressed = true;
  } else if (calibrateButtonState == LOW && calibrateButtonWasPressed) {
    // Button still held
    if (currentMillis - calibrateButtonPressTime >= 3000 && !calibrationMode) {
      // Held for 3 seconds - start calibration
      startCalibration();
    }
  } else if (calibrateButtonState == HIGH && calibrateButtonWasPressed) {
    // Button released
    calibrateButtonWasPressed = false;
  }
  
  // Handle calibration process
  if (calibrationMode) {
    handleCalibration();
    return;  // Skip normal operation during calibration
  }
  
  // Check for test button press to toggle demo mode
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(TEST_BUTTON_PIN);
  
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    // Button pressed (pulled to ground)
    delay(50);  // Debounce
    demoMode = !demoMode;
    demoDataIndex = 0;
    
    if (demoMode) {
      Serial.println("\n╔════════════════════════════════════════╗");
      Serial.println("║    🔥 DEMO MODE ACTIVATED 🔥          ║");
      Serial.println("║  Using Pre-recorded Fire Data         ║");
      Serial.println("╚════════════════════════════════════════╝\n");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DEMO MODE");
      lcd.setCursor(0, 1);
      lcd.print("Fire Data Test");
      delay(2000);
    } else {
      Serial.println("\n✓ Demo mode deactivated - Returning to live sensors\n");
      // Turn off buzzer when exiting demo
      fireDetected = false;
      digitalWrite(BUZZER_PIN, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Live Mode");
      lcd.setCursor(0, 1);
      lcd.print("Active");
      delay(2000);
    }
  }
  lastButtonState = currentButtonState;
  
  // Handle buzzer beeping pattern when fire is detected
  if (fireDetected) {
    if (currentMillis - lastBuzzerToggle >= buzzerBeepInterval) {
      lastBuzzerToggle = currentMillis;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
  }
  
  // Run inference based on mode
  if (demoMode) {
    // Demo mode - cycle through dummy data
    if (currentMillis - lastDemoUpdate >= demoUpdateInterval) {
      lastDemoUpdate = currentMillis;
      runDemoInference();
      demoDataIndex = (demoDataIndex + 1) % DEMO_DATA_COUNT;
    }
  } else {
    // Normal mode - read live sensors
    if (currentMillis - lastInferenceTime >= inferenceInterval) {
      lastInferenceTime = currentMillis;
      readAllSensors();
      runInference();
    }
  }
}

// ====== Get Formatted Date and Time ======
String getFormattedDateTime() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(buffer);
}

// ====== Get Date Only (for organizing in Firebase) ======
String getDateOnly() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  
  char buffer[15];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
  return String(buffer);
}

// ====== Read All Sensors ======
void readAllSensors() {
  aht.getEvent(&humidity_event, &temp_event);
  bmp.getEvent(&pressure_event);
  ens160.setTempAndHum(temp_event.temperature, humidity_event.relative_humidity);
  
  // Average gas sensor readings
  long sum3 = 0, sum8 = 0;
  for (int i = 0; i < 5; i++) {
    sum3 += analogRead(MQ3_PIN);
    sum8 += analogRead(MQ8_PIN);
    delay(10);
  }
  mq3_raw = sum3 / 5;
  mq8_raw = sum8 / 5;
}

// ====== Run ML Inference ======
void runInference() {
  // Prepare sensor values
  float sensor_values[6] = {
      (float)ens160.getECO2(),
      humidity_event.relative_humidity,
      pressure_event.pressure,
      (float)mq3_raw,
      (float)mq8_raw,
      temp_event.temperature
  };

  // Apply calibration offsets if calibrated
  if (isCalibrated) {
    for (int i = 0; i < 6; i++) {
      sensor_values[i] += calibrationOffsets[i];
    }
  }

  // Print sensor readings
  Serial.println("========== SENSOR READINGS ==========");
  Serial.printf("Temperature:   %.2f °C\n", sensor_values[5]);
  Serial.printf("Humidity:      %.2f %%\n", sensor_values[1]);
  Serial.printf("Pressure:      %.2f hPa\n", sensor_values[2]);
  Serial.printf("eCO2:          %.0f ppm\n", sensor_values[0]);
  Serial.printf("H2 (MQ8):      %d\n", mq8_raw);
  Serial.printf("Ethanol (MQ3): %d\n", mq3_raw);
  if (isCalibrated) {
    Serial.println("Status:        CALIBRATED ✓");
  }

  // Standardize & Quantize
  int8_t quantized_input[6];
  for (int i = 0; i < 6; i++) {
    float std_val = (sensor_values[i] - FEATURE_MEANS[i]) / FEATURE_STDS[i];
    float scaled = (std_val / INPUT_SCALE) + INPUT_ZERO_POINT;
    quantized_input[i] = (int8_t)constrain((int)round(scaled), -128, 127);
  }

  // Run inference
  memcpy(input->data.int8, quantized_input, sizeof(quantized_input));
  
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("ERROR: Inference failed!");
    return;
  }

  // Get result
  int8_t raw_output = output->data.int8[0];
  float probability = (raw_output - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
  probability = constrain(probability, 0.0f, 1.0f);
  int fire_alarm = (probability > 0.5f) ? 1 : 0;

  Serial.println("\n========== INFERENCE RESULT ==========");
  Serial.printf("Fire Probability: %.2f%% (%s)\n", 
                probability * 100.0f, 
                fire_alarm ? "FIRE RISK!" : "Safe");
  Serial.println("======================================\n");

  // Update fire detection state and buzzer
  if (fire_alarm == 1 && !fireDetected) {
    // Fire just detected - start buzzer
    fireDetected = true;
    buzzerState = true;
    digitalWrite(BUZZER_PIN, HIGH);
    lastBuzzerToggle = millis();
    Serial.println("🔥 FIRE DETECTED - BUZZER ACTIVATED!");
  } else if (fire_alarm == 0 && fireDetected) {
    // Fire cleared - stop buzzer
    fireDetected = false;
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("✓ Fire cleared - Buzzer deactivated");
  }

  // Update LCD
  updateLCD(sensor_values[5], sensor_values[1], sensor_values[2], 
            (int)sensor_values[0], mq8_raw, mq3_raw, probability, fire_alarm);

  // Send to Firebase
  if (millis() - sendDataPrevMillis > 2000 || sendDataPrevMillis == 0) {
    sendDataPrevMillis = millis();
    sendToFirebase(sensor_values[5], sensor_values[1], sensor_values[2],
                   (int)sensor_values[0], mq8_raw, mq3_raw, probability, fire_alarm);
  }
}

// ====== Send Data to Firebase ======
void sendToFirebase(float temp, float hum, float press, int eco2,
                    int h2, int ethanol, float probability, int fire_alarm) {
  
  Serial.println(">>> Uploading to Firebase...");
  
  // Get formatted date and time
  String dateTime = getFormattedDateTime();
  String dateOnly = getDateOnly();
  unsigned long timestamp = time(nullptr);
  
  Serial.println("Timestamp: " + dateTime);
  
  // Create JSON with all sensor data
  FirebaseJson json;
  json.set("temperature", temp);
  json.set("humidity", hum);
  json.set("pressure", press);
  json.set("eco2", eco2);
  json.set("rawH2", h2);
  json.set("rawEthanol", ethanol);
  json.set("fireProbability", probability);
  json.set("fireAlarm", fire_alarm);
  json.set("dateTime", dateTime);           // Human-readable date and time
  json.set("unixTimestamp", timestamp);     // For sorting/filtering

  // Update current sensor readings
  if (Firebase.RTDB.setJSON(&fbdo, "sensors/current", &json)) {
    Serial.println("✓ Current data updated");
    Serial.println("  PATH: " + fbdo.dataPath());
  } else {
    Serial.println("✗ Current data FAILED");
    Serial.println("  REASON: " + fbdo.errorReason());
  }

  // Add to history organized by date with timestamp
  String historyPath = "sensors/history/" + dateOnly + "/" + String(timestamp);
  if (Firebase.RTDB.setJSON(&fbdo, historyPath.c_str(), &json)) {
    Serial.println("✓ History updated");
    Serial.println("  PATH: " + historyPath);
  } else {
    Serial.println("✗ History FAILED");
    Serial.println("  REASON: " + fbdo.errorReason());
  }

  // If fire detected, log alert with date/time
  if (fire_alarm == 1) {
    FirebaseJson alertJson;
    alertJson.set("probability", probability);
    alertJson.set("dateTime", dateTime);
    alertJson.set("unixTimestamp", timestamp);
    alertJson.set("temperature", temp);
    alertJson.set("humidity", hum);
    alertJson.set("eco2", eco2);
    alertJson.set("rawH2", h2);
    alertJson.set("rawEthanol", ethanol);
    
    String alertPath = "alerts/" + dateOnly + "/" + String(timestamp);
    if (Firebase.RTDB.setJSON(&fbdo, alertPath.c_str(), &alertJson)) {
      Serial.println("🔥 FIRE ALERT LOGGED!");
      Serial.println("  PATH: " + alertPath);
    } else {
      Serial.println("✗ Alert logging FAILED");
    }
  }

  Serial.println();
}

// ====== Update LCD Display ======
void updateLCD(float temp, float hum, float press, int eco2,
               int h2, int ethanol, float probability, int fire_alarm) {
  lcd.clear();

  // Line 0: Temperature and Humidity
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.print("C H:");
  lcd.print(hum, 0);
  lcd.print("%");

  // Line 1: Pressure and eCO2
  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(press, 0);
  lcd.print(" CO2:");
  lcd.print(eco2);

  // Line 2: Gas sensors
  lcd.setCursor(0, 2);
  lcd.print("H2:");
  lcd.print(h2);
  lcd.setCursor(9, 2);
  lcd.print("Et:");
  lcd.print(ethanol);

  // Line 3: Probability and Fire status
  lcd.setCursor(0, 3);
  lcd.print("Prob:");
  lcd.print((int)(probability * 100));
  lcd.print("% ");
  
  if (fire_alarm == 1) {
    lcd.print("FIRE!");
  } else {
    lcd.print("SAFE");
  }
}

// ====== Run Demo Inference with Dummy Data ======
void runDemoInference() {
  // Get current dummy data
  float eco2 = DEMO_DATA[demoDataIndex][0];
  float hum = DEMO_DATA[demoDataIndex][1];
  float press = DEMO_DATA[demoDataIndex][2];
  int ethanol = (int)DEMO_DATA[demoDataIndex][3];
  int h2 = (int)DEMO_DATA[demoDataIndex][4];
  float temp = DEMO_DATA[demoDataIndex][5];
  int expected_fire = (int)DEMO_DATA[demoDataIndex][6];
  
  // Prepare sensor values for model
  float sensor_values[6] = {
      eco2,
      hum,
      press,
      (float)ethanol,
      (float)h2,
      temp
  };

  // Print sensor readings
  Serial.println("========== DEMO DATA [" + String(demoDataIndex + 1) + "/8] ==========");
  Serial.printf("Temperature:   %.2f °C\n", temp);
  Serial.printf("Humidity:      %.2f %%\n", hum);
  Serial.printf("Pressure:      %.2f hPa\n", press);
  Serial.printf("eCO2:          %.0f ppm\n", eco2);
  Serial.printf("H2 (MQ8):      %d\n", h2);
  Serial.printf("Ethanol (MQ3): %d\n", ethanol);
  Serial.printf("Expected:      FIRE (Label=1)\n");

  // Standardize & Quantize
  int8_t quantized_input[6];
  for (int i = 0; i < 6; i++) {
    float std_val = (sensor_values[i] - FEATURE_MEANS[i]) / FEATURE_STDS[i];
    float scaled = (std_val / INPUT_SCALE) + INPUT_ZERO_POINT;
    quantized_input[i] = (int8_t)constrain((int)round(scaled), -128, 127);
  }

  // Run inference
  memcpy(input->data.int8, quantized_input, sizeof(quantized_input));
  
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("ERROR: Inference failed!");
    return;
  }

  // Get result
  int8_t raw_output = output->data.int8[0];
  float probability = (raw_output - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
  probability = constrain(probability, 0.0f, 1.0f);
  int fire_alarm = (probability > 0.5f) ? 1 : 0;

  Serial.println("\n========== DEMO INFERENCE RESULT ==========");
  Serial.printf("Fire Probability: %.2f%% (%s)\n", 
                probability * 100.0f, 
                fire_alarm ? "🔥 FIRE DETECTED!" : "Safe");
  Serial.printf("Model Prediction: %s\n", fire_alarm ? "CORRECT ✓" : "INCORRECT ✗");
  Serial.println("==========================================\n");

  // Update fire detection state and buzzer
  if (fire_alarm == 1 && !fireDetected) {
    fireDetected = true;
    buzzerState = true;
    digitalWrite(BUZZER_PIN, HIGH);
    lastBuzzerToggle = millis();
    Serial.println("🔥 DEMO: FIRE DETECTED - BUZZER ACTIVATED!");
  } else if (fire_alarm == 0 && fireDetected) {
    fireDetected = false;
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("✓ Demo: Fire cleared - Buzzer deactivated");
  }

  // Update LCD
  updateLCD(temp, hum, press, (int)eco2, h2, ethanol, probability, fire_alarm);

  // Send to Firebase (optional in demo mode - comment out if not needed)
  sendToFirebase(temp, hum, press, (int)eco2, h2, ethanol, probability, fire_alarm);
}

// ====== Start Calibration Process ======
void startCalibration() {
  calibrationMode = true;
  calibrationSampleCount = 0;
  
  // Reset calibration sums
  for (int i = 0; i < 6; i++) {
    calibrationSums[i] = 0;
  }
  
  // Turn off buzzer during calibration
  fireDetected = false;
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║      🔧 CALIBRATION MODE STARTED 🔧          ║");
  Serial.println("║                                               ║");
  Serial.println("║  Please ensure there is NO FIRE or smoke     ║");
  Serial.println("║  in the environment.                          ║");
  Serial.println("║                                               ║");
  Serial.println("║  Collecting 20 samples (40 seconds)...       ║");
  Serial.println("╚═══════════════════════════════════════════════╝\n");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CALIBRATING...");
  lcd.setCursor(0, 1);
  lcd.print("No fire/smoke!");
  lcd.setCursor(0, 2);
  lcd.print("Wait 40 seconds");
}

// ====== Handle Calibration Process ======
void handleCalibration() {
  static unsigned long lastCalibrationSample = 0;
  unsigned long currentMillis = millis();
  
  // Collect sample every 2 seconds
  if (currentMillis - lastCalibrationSample >= 2000) {
    lastCalibrationSample = currentMillis;
    
    // Read sensors
    readAllSensors();
    
    // Add to sums
    calibrationSums[0] += (float)ens160.getECO2();
    calibrationSums[1] += humidity_event.relative_humidity;
    calibrationSums[2] += pressure_event.pressure;
    calibrationSums[3] += (float)mq3_raw;
    calibrationSums[4] += (float)mq8_raw;
    calibrationSums[5] += temp_event.temperature;
    
    calibrationSampleCount++;
    
    Serial.printf("Calibration sample %d/%d collected\n", 
                  calibrationSampleCount, CALIBRATION_SAMPLES);
    
    // Update LCD progress
    lcd.setCursor(0, 3);
    lcd.print("Sample: ");
    lcd.print(calibrationSampleCount);
    lcd.print("/");
    lcd.print(CALIBRATION_SAMPLES);
    
    // Check if calibration complete
    if (calibrationSampleCount >= CALIBRATION_SAMPLES) {
      completeCalibration();
    }
  }
}

// ====== Complete Calibration ======
void completeCalibration() {
  calibrationMode = false;
  
  // Calculate averages for current environment
  float currentEnvironment[6];
  for (int i = 0; i < 6; i++) {
    currentEnvironment[i] = calibrationSums[i] / CALIBRATION_SAMPLES;
  }
  
  // Calculate offsets to shift current environment to baseline
  for (int i = 0; i < 6; i++) {
    calibrationOffsets[i] = BASELINE_MEANS[i] - currentEnvironment[i];
  }
  
  isCalibrated = true;
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║      ✓ CALIBRATION COMPLETED ✓               ║");
  Serial.println("╚═══════════════════════════════════════════════╝\n");
  
  Serial.println("Current Environment Baseline:");
  Serial.printf("  eCO2:     %.1f ppm (offset: %.1f)\n", 
                currentEnvironment[0], calibrationOffsets[0]);
  Serial.printf("  Humidity: %.1f %% (offset: %.1f)\n", 
                currentEnvironment[1], calibrationOffsets[1]);
  Serial.printf("  Pressure: %.1f hPa (offset: %.1f)\n", 
                currentEnvironment[2], calibrationOffsets[2]);
  Serial.printf("  Ethanol:  %.0f (offset: %.0f)\n", 
                currentEnvironment[3], calibrationOffsets[3]);
  Serial.printf("  H2:       %.0f (offset: %.0f)\n", 
                currentEnvironment[4], calibrationOffsets[4]);
  Serial.printf("  Temp:     %.1f °C (offset: %.1f)\n\n", 
                currentEnvironment[5], calibrationOffsets[5]);
  
  Serial.println("Device is now calibrated for this location!");
  Serial.println("Fire detection will be more accurate.\n");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Complete!");
  lcd.setCursor(0, 2);
  lcd.print("Device Ready");
  
  // Beep buzzer 3 times to confirm
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  
  delay(3000);
  lcd.clear();
}