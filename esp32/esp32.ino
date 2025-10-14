/*
  ESP32 Fire Detection - TFLite INT8 Model with Firebase Integration
  Sends sensor readings and fire risk status to Firebase Realtime Database
  
  Libraries needed:
  - Firebase ESP Client by Mobizt
  - DFRobot_ENS160
  - Adafruit AHTX0
  - Adafruit BMP085 Unified
  - LiquidCrystal I2C
  - TensorFlow Lite Micro
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP085_U.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions
#include "addons/RTDBHelper.h"

// TensorFlow Lite Micro
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tiny_model_int8.h"

// ==================== WiFi Configuration ====================
#define WIFI_SSID "Lord of the pings"
#define WIFI_PASSWORD "15448192"

// ==================== Firebase Configuration ====================
#define API_KEY "AIzaSyCdHnIZKf2pAMMuIEtve_mkykLEptXny40"
#define DATABASE_URL "https://firely-50a30-default-rtdb.asia-southeast1.firebasedatabase.app"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK = false;

// ==================== TFLite arena size ====================
constexpr int kTensorArenaSize = 12 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

// TFLite objects
const tflite::Model* model = nullptr;
tflite::MicroMutableOpResolver<12> resolver;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// ===============================================
// Quantization parameters
// ===============================================
const float INPUT_SCALE = 0.02470421977341175f;
const int INPUT_ZERO_POINT = -23;
const float OUTPUT_SCALE = 0.00390625f;
const int OUTPUT_ZERO_POINT = -128;

// ===============================================
// Standardization parameters
// Order: [eCO2, Humidity, Pressure, Raw_Ethanol, Raw_H2, Temperature]
// ===============================================
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
#define MQ3_PIN 34      // MQ3 (Ethanol) - Analog input
#define MQ8_PIN 35      // MQ8 (H2) - Analog input

// ====== SENSOR VARIABLES ======
sensors_event_t humidity_event, temp_event, pressure_event;
int mq8_raw = 0;
int mq3_raw = 0;

// ====== DATA COLLECTION VARIABLES ======
unsigned long lastInferenceTime = 0;
const unsigned long inferenceInterval = 1000;  // 1 second
unsigned long inferenceCounter = 0;

// ====== FUNCTION DECLARATIONS ======
void initWiFi();
void initFirebase();
void initI2C();
void initSensors();
void initTFLiteModel();
void readAllSensors();
void readGasSensors();
void runInference();
void sendToFirebase(float temp, float hum, float press, int eco2,
                    int h2, int ethanol, float probability, int fire_alarm);
void updateLCDDisplay(float temp, float hum, float press, int eco2,
                      int h2, int ethanol, float probability, int fire_alarm);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===========================================");
  Serial.println("ESP32 Fire Detection System");
  Serial.println("TFLite + Firebase Integration");
  Serial.println("===========================================\n");

  // Initialize WiFi
  initWiFi();

  // Initialize I2C and pins
  initI2C();

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fire Detection");
  lcd.setCursor(0, 1);
  lcd.print("TFLite+Firebase");
  lcd.setCursor(0, 2);
  lcd.print("Initializing...");
  delay(1000);

  // Initialize sensors
  initSensors();

  // Initialize TFLite model
  initTFLiteModel();

  // Initialize Firebase
  initFirebase();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  lcd.setCursor(0, 1);
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 2);
  lcd.print("Firebase Ready");
  delay(2000);
  lcd.clear();

  Serial.println("\n===========================================");
  Serial.println("Fire Detection System Started");
  Serial.println("Running inference every 1 second");
  Serial.println("===========================================\n");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastInferenceTime >= inferenceInterval) {
    lastInferenceTime = currentMillis;

    // Read all sensors
    readAllSensors();

    // Run ML inference
    runInference();

    inferenceCounter++;
  }
}

// ====== WiFi Initialization ======
void initWiFi() {
  Serial.println("Connecting to WiFi...");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  lcd.print(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(attempts % 16, 2);
    lcd.print(".");
    attempts++;
  }
  
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected Successfully!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm\n");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected!");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    Serial.println("WiFi Connection Failed!");
    Serial.println("Please check your credentials and try again.");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    lcd.setCursor(0, 1);
    lcd.print("Check Settings");
    
    while(1) {
      delay(1000);
    }
  }
}

// ====== Firebase Initialization ======
void initFirebase() {
  Serial.println("Initializing Firebase...");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Init Firebase...");
  
  // Assign the api key
  config.api_key = API_KEY;
  
  // Assign the RTDB URL
  config.database_url = DATABASE_URL;
  
  // Sign up (anonymous authentication)
  Serial.println("Signing up to Firebase...");
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase signup successful!");
    signupOK = true;
    
    lcd.setCursor(0, 1);
    lcd.print("Signup Success!");
  } else {
    Serial.println("Firebase signup failed!");
    Serial.printf("Error: %s\n", config.signer.signupError.message.c_str());
    
    lcd.setCursor(0, 1);
    lcd.print("Signup Failed!");
    delay(3000);
  }
  
  // Assign the callback function for token generation
  config.token_status_callback = tokenStatusCallback;
  
  // Set timeout
  config.timeout.serverResponse = 10 * 1000; // 10 seconds
  
  // Initialize Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.println("Firebase initialized successfully!\n");
  
  lcd.setCursor(0, 2);
  lcd.print("Firebase Ready!");
  delay(1500);
}

// ====== I2C Initialization ======
void initI2C() {
  Serial.println("Initializing I2C and GPIO...");
  
  Wire.begin(21, 22);   // SDA = 21, SCL = 22
  Wire.setClock(100000); // 100kHz
  
  pinMode(MQ3_PIN, INPUT);
  pinMode(MQ8_PIN, INPUT);
  analogSetPinAttenuation(MQ3_PIN, ADC_11db);
  analogSetPinAttenuation(MQ8_PIN, ADC_11db);
  
  Serial.println("I2C and GPIO initialized!\n");
}

// ====== Sensors Initialization ======
void initSensors() {
  Serial.println("Initializing sensors...");

  // Initialize AHT sensor
  Serial.print("- AHT (Temp/Humidity)... ");
  if (!aht.begin()) {
    Serial.println("FAILED!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("AHT Error!");
    lcd.setCursor(0, 1);
    lcd.print("Check Wiring");
    while (1) delay(1000);
  }
  Serial.println("OK");

  // Initialize BMP180 sensor
  Serial.print("- BMP180 (Pressure)... ");
  if (!bmp.begin()) {
    Serial.println("FAILED!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BMP180 Error!");
    lcd.setCursor(0, 1);
    lcd.print("Check Wiring");
    while (1) delay(1000);
  }
  Serial.println("OK");

  // Initialize ENS160 sensor
  Serial.print("- ENS160 (eCO2)... ");
  if (ens160.begin() != NO_ERR) {
    Serial.println("FAILED!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ENS160 Error!");
    lcd.setCursor(0, 1);
    lcd.print("Check Wiring");
    while (1) delay(1000);
  }
  Serial.println("OK");
  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(25.0, 50.0);

  // Initialize gas sensors
  Serial.println("- MQ3 (Ethanol)... OK");
  Serial.println("- MQ8 (H2)... OK");

  Serial.println("All sensors initialized successfully!\n");
}

// ====== TFLite Model Initialization ======
void initTFLiteModel() {
  Serial.println("Loading TFLite model...");

  // Load model
  model = tflite::GetModel(tiny_model_int8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERROR: Model schema version mismatch!");
    Serial.printf("Model version: %d, Expected: %d\n", 
                  model->version(), TFLITE_SCHEMA_VERSION);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Model Error!");
    lcd.setCursor(0, 1);
    lcd.print("Version Mismatch");
    while (1) delay(1000);
  }
  Serial.println("- Model loaded successfully");

  // Add required ops
  Serial.println("- Adding operators...");
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    Serial.println("  ERROR: Failed to add FullyConnected");
  } else {
    Serial.println("  FullyConnected added");
  }
  
  if (resolver.AddLogistic() != kTfLiteOk) {
    Serial.println("  ERROR: Failed to add Logistic");
  } else {
    Serial.println("  Logistic added");
  }
  
  if (resolver.AddQuantize() != kTfLiteOk) {
    Serial.println("  ERROR: Failed to add Quantize");
  } else {
    Serial.println("  Quantize added");
  }
  
  if (resolver.AddDequantize() != kTfLiteOk) {
    Serial.println("  ERROR: Failed to add Dequantize");
  } else {
    Serial.println("  Dequantize added");
  }

  // Create interpreter
  Serial.println("- Creating interpreter...");
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate tensors
  Serial.println("- Allocating tensors...");
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERROR: AllocateTensors failed!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tensor Error!");
    lcd.setCursor(0, 1);
    lcd.print("Alloc Failed");
    while (1) delay(1000);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("\nTFLite Model Configuration:");
  Serial.println("---------------------------");
  Serial.printf("Input shape: [1, 6]\n");
  Serial.printf("Input type: INT8\n");
  Serial.printf("Input quantization:\n");
  Serial.printf("  - Scale: %.8f\n", INPUT_SCALE);
  Serial.printf("  - Zero point: %d\n", INPUT_ZERO_POINT);
  Serial.printf("Output quantization:\n");
  Serial.printf("  - Scale: %.8f\n", OUTPUT_SCALE);
  Serial.printf("  - Zero point: %d\n", OUTPUT_ZERO_POINT);
  Serial.printf("Arena used: %d / %d bytes (%.1f%%)\n",
                interpreter->arena_used_bytes(), kTensorArenaSize,
                (interpreter->arena_used_bytes() * 100.0f) / kTensorArenaSize);
  Serial.println("---------------------------\n");
}

// ====== Read All Sensors ======
void readAllSensors() {
  aht.getEvent(&humidity_event, &temp_event);
  bmp.getEvent(&pressure_event);
  ens160.setTempAndHum(temp_event.temperature, humidity_event.relative_humidity);
  readGasSensors();
}

// ====== Read Gas Sensors ======
void readGasSensors() {
  const int samples = 6;
  long sum3 = 0, sum8 = 0;

  for (int i = 0; i < samples; i++) {
    sum3 += analogRead(MQ3_PIN);
    sum8 += analogRead(MQ8_PIN);
    delay(10);
  }

  mq3_raw = sum3 / samples;
  mq8_raw = sum8 / samples;
}

// ====== Run ML Inference ======
void runInference() {
  // Prepare raw sensor values in correct order:
  // [eCO2, Humidity, Pressure, Raw_Ethanol, Raw_H2, Temperature]
  float sensor_values[6] = {
      (float)ens160.getECO2(),              // eCO2
      humidity_event.relative_humidity,     // Humidity
      pressure_event.pressure,              // Pressure
      (float)mq3_raw,                       // Raw_Ethanol (MQ3)
      (float)mq8_raw,                       // Raw_H2 (MQ8)
      temp_event.temperature                // Temperature
  };

  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║       SENSOR READINGS                  ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.printf("║ eCO2:          %6.0f ppm             ║\n", sensor_values[0]);
  Serial.printf("║ Humidity:      %6.2f %%               ║\n", sensor_values[1]);
  Serial.printf("║ Pressure:      %6.2f hPa             ║\n", sensor_values[2]);
  Serial.printf("║ Ethanol (MQ3): %6.0f (raw)           ║\n", sensor_values[3]);
  Serial.printf("║ H2 (MQ8):      %6.0f (raw)           ║\n", sensor_values[4]);
  Serial.printf("║ Temperature:   %6.2f °C              ║\n", sensor_values[5]);
  Serial.println("╚════════════════════════════════════════╝");

  // ========== STANDARDIZATION & QUANTIZATION ==========
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   PREPROCESSING (STANDARDIZE+QUANTIZE) ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  int8_t quantized_input[6];
  const char* names[] = {"eCO2", "Humid", "Press", "EtOH", "H2", "Temp"};
  
  for (int i = 0; i < 6; i++) {
    // 1. Standardize (Z-score normalization)
    float standardized_value = (sensor_values[i] - FEATURE_MEANS[i]) / FEATURE_STDS[i];
    
    // 2. Quantize
    float scaled = (standardized_value / INPUT_SCALE) + INPUT_ZERO_POINT;
    int quant_value = (int)round(scaled);
    
    // 3. Clamp to int8 range [-128, 127]
    quantized_input[i] = (int8_t)max(-128, min(127, quant_value));
    
    Serial.printf("║ [%d] %-5s: %7.1f → %6.2f → %4d   ║\n", 
                  i, names[i], sensor_values[i], standardized_value, quantized_input[i]);
  }
  Serial.println("╚════════════════════════════════════════╝");

  // ========== INFERENCE ==========
  memcpy(input->data.int8, quantized_input, sizeof(quantized_input));
  
  unsigned long start = micros();
  TfLiteStatus invoke_status = interpreter->Invoke();
  unsigned long inference_time = micros() - start;

  if (invoke_status != kTfLiteOk) {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║          ERROR: INFERENCE FAILED       ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    lcd.clear(); 
    lcd.setCursor(0, 0); 
    lcd.print("Inference Error!");
    delay(1000);
    return;
  }

  // ========== POSTPROCESSING ==========
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║        INFERENCE RESULT                ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  int8_t raw_output = output->data.int8[0];
  Serial.printf("║ Raw Output (int8):     %4d            ║\n", raw_output);
  
  // Dequantize to float probability
  float probability = (raw_output - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
  
  // Clamp to valid probability range [0.0, 1.0]
  probability = max(0.0f, min(1.0f, probability));
  
  Serial.printf("║ Fire Probability:      %.4f          ║\n", probability);
  Serial.printf("║ Confidence:            %.1f%%           ║\n", probability * 100.0f);
  Serial.printf("║ Decision Threshold:    0.5000          ║\n");
  Serial.printf("║ Inference Time:        %4lu µs         ║\n", inference_time);
  Serial.printf("║                        %.2f ms         ║\n", inference_time / 1000.0f);
  
  // Fire classification
  int fire_alarm = (probability > 0.5f) ? 1 : 0;
  
  Serial.println("╠════════════════════════════════════════╣");
  if (fire_alarm == 1) {
    Serial.println("║     🔥 FIRE RISK DETECTED! 🔥         ║");
  } else {
    Serial.println("║     ✓ NO FIRE RISK DETECTED           ║");
  }
  Serial.println("╚════════════════════════════════════════╝\n");

  // Send to Firebase
  sendToFirebase(temp_event.temperature, humidity_event.relative_humidity,
                 pressure_event.pressure, ens160.getECO2(),
                 mq8_raw, mq3_raw, probability, fire_alarm);

  // Update LCD
  updateLCDDisplay(temp_event.temperature, humidity_event.relative_humidity,
                   pressure_event.pressure, ens160.getECO2(),
                   mq8_raw, mq3_raw, probability, fire_alarm);
}

// ====== Send Data to Firebase ======
void sendToFirebase(float temp, float hum, float press, int eco2,
                    int h2, int ethanol, float probability, int fire_alarm) {
  
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready. Skipping upload...");
    return;
  }
  
  if (!signupOK) {
    Serial.println("Firebase authentication failed. Skipping upload...");
    return;
  }
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║      UPLOADING TO FIREBASE             ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  // Get current timestamp (milliseconds since epoch)
  unsigned long timestamp = millis();
  
  // Create JSON object with all sensor data
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
  
  // Update current sensor readings (overwrite)
  Serial.print("║ Updating current data...               ");
  if (Firebase.RTDB.setJSON(&fbdo, "sensors/current", &json)) {
    Serial.println("✓ ║");
  } else {
    Serial.println("✗ ║");
    Serial.println("║ Error: " + fbdo.errorReason());
  }
  
  // Add to history (append with timestamp as key)
  String historyPath = "sensors/history/" + String(timestamp);
  Serial.print("║ Adding to history...                   ");
  if (Firebase.RTDB.setJSON(&fbdo, historyPath.c_str(), &json)) {
    Serial.println("✓ ║");
  } else {
    Serial.println("✗ ║");
    Serial.println("║ Error: " + fbdo.errorReason());
  }
  
  // If fire detected, add to alerts
  if (fire_alarm == 1) {
    Serial.print("║ 🔥 FIRE ALERT - Logging alert...       ");
    
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
      Serial.println("✓ ║");
    } else {
      Serial.println("✗ ║");
      Serial.println("║ Error: " + fbdo.errorReason());
    }
  }
  
  Serial.println("╚════════════════════════════════════════╝\n");
}

// ====== Update LCD Display ======
void updateLCDDisplay(float temp, float hum, float press, int eco2,
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