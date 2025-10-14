/*
  ESP32 Fire Detection - TFLite INT8 Model with Firebase Integration
  Using Firebase Database Secret (Legacy Token) for authentication
  
  Libraries needed:
  - Firebase ESP Client by Mobizt
  - DFRobot_ENS160
  - Adafruit AHTX0
  - Adafruit BMP085 Unified
  - LiquidCrystal I2C
  - TensorFlow Lite Micro
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
// Get your Database Secret from: Firebase Console → Project Settings → Service Accounts → Database Secrets
#define DATABASE_URL "https://firely-50a30-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define DATABASE_SECRET "h6iRd4cv0jcYsaYs3k8Im2u85N3nrgsfMkXQsOTE"  // Replace with actual secret from Firebase Console

// If you don't have Database Secret, use this alternative approach
// Set Firebase Rules to: { "rules": { ".read": true, ".write": true } }
// Then leave DATABASE_SECRET as empty string: ""

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

// ====== SENSOR VARIABLES ======
sensors_event_t humidity_event, temp_event, pressure_event;
int mq8_raw = 0;
int mq3_raw = 0;

// ====== TIMING VARIABLES ======
unsigned long lastInferenceTime = 0;
const unsigned long inferenceInterval = 2000;  // 2 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===========================================");
  Serial.println("ESP32 Fire Detection System");
  Serial.println("TFLite + Firebase Integration");
  Serial.println("===========================================\n");

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
  
  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;
  
  /* Assign the database secret (legacy token) */
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  
  /* Initialize Firebase without authentication */
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // Set SSL buffer size
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
  
  // Run inference every 2 seconds
  if (currentMillis - lastInferenceTime >= inferenceInterval) {
    lastInferenceTime = currentMillis;
    
    // Read all sensors
    readAllSensors();
    
    // Run ML inference
    runInference();
  }
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

  // Print sensor readings
  Serial.println("========== SENSOR READINGS ==========");
  Serial.printf("Temperature:   %.2f °C\n", sensor_values[5]);
  Serial.printf("Humidity:      %.2f %%\n", sensor_values[1]);
  Serial.printf("Pressure:      %.2f hPa\n", sensor_values[2]);
  Serial.printf("eCO2:          %.0f ppm\n", sensor_values[0]);
  Serial.printf("H2 (MQ8):      %d\n", mq8_raw);
  Serial.printf("Ethanol (MQ3): %d\n", mq3_raw);

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
  
  unsigned long timestamp = millis();
  
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
  json.set("timestamp", timestamp);

  // Update current sensor readings
  if (Firebase.RTDB.setJSON(&fbdo, "sensors/current", &json)) {
    Serial.println("✓ Current data updated");
    Serial.println("  PATH: " + fbdo.dataPath());
  } else {
    Serial.println("✗ Current data FAILED");
    Serial.println("  REASON: " + fbdo.errorReason());
  }

  // Add to history with timestamp
  String historyPath = "sensors/history/" + String(timestamp);
  if (Firebase.RTDB.setJSON(&fbdo, historyPath.c_str(), &json)) {
    Serial.println("✓ History updated");
  } else {
    Serial.println("✗ History FAILED");
    Serial.println("  REASON: " + fbdo.errorReason());
  }

  // If fire detected, log alert
  if (fire_alarm == 1) {
    FirebaseJson alertJson;
    alertJson.set("probability", probability);
    alertJson.set("timestamp", timestamp);
    alertJson.set("temperature", temp);
    alertJson.set("humidity", hum);
    alertJson.set("eco2", eco2);
    alertJson.set("rawH2", h2);
    alertJson.set("rawEthanol", ethanol);
    
    String alertPath = "alerts/" + String(timestamp);
    if (Firebase.RTDB.setJSON(&fbdo, alertPath.c_str(), &alertJson)) {
      Serial.println("🔥 FIRE ALERT LOGGED!");
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