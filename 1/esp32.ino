/*
  ESP32 Fire Detection - TFLite INT8 Model Inference (Real Sensors)
  - Reads sensors: AHT (temp/hum), BMP180 (pressure), ENS160 (eCO2), MQ3, MQ8
  - Runs TFLite quantized model for fire detection
  - Feature order: [eCO2, Humidity, Pressure, Raw_Ethanol, Raw_H2, Temperature]
  - Model input/output already normalized (mean≈0, std=1)
*/

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP085_U.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// TensorFlow Lite Micro
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// This header must be created from your tiny_model_int8.tflite
#include "tiny_model_int8.h"

// ==================== TFLite arena size ====================
constexpr int kTensorArenaSize = 12 * 1024; // 12 KB
static uint8_t tensor_arena[kTensorArenaSize];

// TFLite objects
const tflite::Model* model = nullptr;
tflite::MicroMutableOpResolver<12> resolver;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// ===============================================
// Quantization parameters (from your trained model)
// ===============================================
const float INPUT_SCALE = 0.02470421977341175f;
const int INPUT_ZERO_POINT = -23;
const float OUTPUT_SCALE = 0.00390625f;
const int OUTPUT_ZERO_POINT = -128;

// Model input is already standardized: mean ≈ 0, std = 1
// No additional normalization needed

// ====== SENSOR OBJECTS ======
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
Adafruit_AHTX0 aht;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
LiquidCrystal_I2C lcd(0x27, 16, 4);

// ====== PIN DEFINITIONS ======
#define MQ3_PIN 34    // MQ3 (Ethanol) - Analog input
#define MQ8_PIN 35    // MQ8 (H2) - Analog input

// ====== SENSOR VARIABLES ======
sensors_event_t humidity_event, temp_event, pressure_event;
int mq8_raw = 0;
int mq3_raw = 0;

// ====== DATA COLLECTION VARIABLES ======
unsigned long lastInferenceTime = 0;
const unsigned long inferenceInterval = 1000;  // 1 second
unsigned long inferenceCounter = 0;

// ====== FUNCTION DECLARATIONS ======
void initI2C();
void initSensors();
void initTFLiteModel();
void readAllSensors();
void readGasSensors();
void runInference();
void updateLCDDisplay(float temp, float hum, float press, int eco2,
                     int h2, int ethanol, float probability, int fire_alarm);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ESP32 Fire Detection (TFLite + Real Sensors) ---");

  // Initialize I2C and pins
  initI2C();

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fire Detection");
  lcd.setCursor(0, 1);
  lcd.print("TFLite System");
  lcd.setCursor(0, 2);
  lcd.print("Initializing...");
  delay(1000);

  // Initialize sensors
  initSensors();

  // Initialize TFLite model
  initTFLiteModel();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Monitoring...");
  delay(1000);
  lcd.clear();

  Serial.println("\n--- Fire Detection Started ---");
  Serial.println("Running inference every 1 second...\n");
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

// ====== I2C Initialization ======
void initI2C() {
  Wire.begin(21, 22);  // SDA = 21, SCL = 22
  pinMode(MQ3_PIN, INPUT);
  pinMode(MQ8_PIN, INPUT);
  analogSetPinAttenuation(MQ3_PIN, ADC_11db);
  analogSetPinAttenuation(MQ8_PIN, ADC_11db);
}

// ====== Sensors Initialization ======
void initSensors() {
  Serial.println("Initializing sensors...");

  if (!aht.begin()) {
    Serial.println("AHT sensor not found!");
    lcd.clear();
    lcd.print("AHT Error!");
    while (1) delay(1000);
  }
  Serial.println("AHT OK");

  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
    lcd.clear();
    lcd.print("BMP Error!");
    while (1) delay(1000);
  }
  Serial.println("BMP OK");

  if (ens160.begin() != NO_ERR) {
    Serial.println("ENS160 not found!");
    lcd.clear();
    lcd.print("ENS160 Error!");
    while (1) delay(1000);
  }
  Serial.println("ENS160 OK");
  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(25.0, 50.0);

  Serial.println("All sensors initialized successfully!\n");
}

// ====== TFLite Model Initialization ======
void initTFLiteModel() {
  Serial.println("Loading TFLite model...");

  // Load model
  model = tflite::GetModel(tiny_model_int8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("Model schema version mismatch!\n");
    lcd.clear();
    lcd.print("Model Err!");
    while (1) delay(1000);
  }

  // Add required ops
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    Serial.println("Failed to add FullyConnected");
  }
  if (resolver.AddLogistic() != kTfLiteOk) {
    Serial.println("Failed to add Logistic");
  }
  if (resolver.AddQuantize() != kTfLiteOk) {
    Serial.println("Failed to add Quantize");
  }
  if (resolver.AddDequantize() != kTfLiteOk) {
    Serial.println("Failed to add Dequantize");
  }

  // Create interpreter
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors failed!");
    lcd.clear();
    lcd.print("Tensor Err!");
    while (1) delay(1000);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("TFLite model loaded successfully!");
  Serial.printf(" Input shape: [1, 6]\n");
  Serial.printf(" Input quantization: scale=%.8f, zero_point=%d\n", 
                INPUT_SCALE, INPUT_ZERO_POINT);
  Serial.printf(" Output quantization: scale=%.8f, zero_point=%d\n", 
                OUTPUT_SCALE, OUTPUT_ZERO_POINT);
  Serial.printf(" Arena used: %d / %d bytes\n\n",
                interpreter->arena_used_bytes(), kTensorArenaSize);
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
      (float)ens160.getECO2(),                    // eCO2
      humidity_event.relative_humidity,           // Humidity
      pressure_event.pressure,                    // Pressure
      (float)mq3_raw,                            // Raw_Ethanol (MQ3)
      (float)mq8_raw,                            // Raw_H2 (MQ8)
      temp_event.temperature                     // Temperature
  };

  Serial.println("========== SENSOR READINGS ==========");
  Serial.printf(" eCO2: %.0f ppm\n", sensor_values[0]);
  Serial.printf(" Humidity: %.2f %%\n", sensor_values[1]);
  Serial.printf(" Pressure: %.2f hPa\n", sensor_values[2]);
  Serial.printf(" Raw Ethanol (MQ3): %.0f\n", sensor_values[3]);
  Serial.printf(" Raw H2 (MQ8): %.0f\n", sensor_values[4]);
  Serial.printf(" Temperature: %.2f C\n", sensor_values[5]);

  // ========== QUANTIZATION ==========
  Serial.println("\n========== QUANTIZATION ==========");
  int8_t quantized_input[6];
  
  for (int i = 0; i < 6; i++) {
    // Model expects already-normalized input (mean≈0, std=1)
    // So we directly quantize the sensor reading
    float scaled = (sensor_values[i] / INPUT_SCALE) + INPUT_ZERO_POINT;
    int quant_value = (int)round(scaled);
    
    // Clamp to int8 range [-128, 127]
    quantized_input[i] = (int8_t)max(-128, min(127, quant_value));
    
    const char* names[] = {"CO2", "Humid", "Press", "EtOH", "H2", "Temp"};
    Serial.printf(" [%d] %s: %.2f -> quantized: %d\n", 
                  i, names[i], sensor_values[i], quantized_input[i]);
  }

  // ========== INFERENCE ==========
  memcpy(input->data.int8, quantized_input, sizeof(quantized_input));
  
  unsigned long start = micros();
  TfLiteStatus invoke_status = interpreter->Invoke();
  unsigned long inference_time = micros() - start;

  if (invoke_status != kTfLiteOk) {
    Serial.println("ERROR: Inference failed!");
    lcd.clear(); 
    lcd.setCursor(0, 0); 
    lcd.print("Inference Err");
    delay(1000);
    return;
  }

  // ========== POSTPROCESSING ==========
  Serial.println("\n========== INFERENCE RESULT ==========");
  
  int8_t raw_output = output->data.int8[0];
  Serial.printf(" Raw Output (int8): %d\n", raw_output);
  
  // Dequantize to float probability
  float probability = (raw_output - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
  
  // Clamp to valid probability range [0.0, 1.0]
  probability = max(0.0f, min(1.0f, probability));
  
  Serial.printf(" Fire Probability: %.4f (%.2f%%)\n", probability, probability * 100.0f);
  Serial.printf(" Decision Threshold: 0.50\n");
  Serial.printf(" Inference Time: %lu us (%.2f ms)\n", 
                inference_time, inference_time / 1000.0f);
  
  // Fire classification
  int fire_alarm = (probability > 0.5f) ? 1 : 0;
  
  Serial.println("\n========== STATUS ==========");
  if (fire_alarm == 1) {
    Serial.println(" FIRE RISK DETECTED!");
  } else {
    Serial.println(" No fire risk detected.");
  }
  Serial.println("====================================\n");

  // Update LCD
  updateLCDDisplay(temp_event.temperature, humidity_event.relative_humidity,
                   pressure_event.pressure, ens160.getECO2(),
                   mq8_raw, mq3_raw, probability, fire_alarm);

  delay(1000);
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
    lcd.print("OK ");
  }
}