/*
  ESP32 Fire Detection - TFLite INT8 model inference
  - Uses sensors: AHT (temp/hum), BMP180 (pressure), ENS160 (eCO2)
  - Uses two analog gas sensors (MQ3 ethanol, MQ8 H2)
  - Requires tiny_model_int8.h (C array of your .tflite)
  - Ensure feature order matches training:
      [Temperature, Humidity, eCO2, H2, Ethanol, Pressure]
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
// FIXED Quantization parameters (from your model)
// ===============================================
const float INPUT_SCALE = 0.03486747f;
const int INPUT_ZERO_POINT = 18;
const float OUTPUT_SCALE = 0.00390625f;
const int OUTPUT_ZERO_POINT = -128;

// ====== NORMALIZATION PARAMETERS (from training StandardScaler) ======
float feature_mean[6] = {
  23.36840636,      // Temperature mean (°C)
  47.23058261,      // Humidity mean (%)
  679.20042599,     // eCO2 mean (ppm)
  12934.13038757,   // H2 mean (BME688 equivalent)
  19852.10524557,   // Ethanol mean (BME688 equivalent)
  938.48237934      // Pressure mean (hPa)
};

float feature_std[6] = {
  8.32235178,       // Temperature std
  9.61876165,       // Humidity std
  2057.95925,       // eCO2 std
  286.572397,       // H2 std
  667.883160,       // Ethanol std
  1.41767975        // Pressure std
};

// ====== SENSOR OBJECTS ======
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
Adafruit_AHTX0 aht;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
LiquidCrystal_I2C lcd(0x27, 16, 4);

// ====== PIN DEFINITIONS ======
#define MQ3_PIN 34    // MQ3 (Ethanol) - Analog input
#define MQ8_PIN 35    // MQ8 (H2) - Analog input

const int ADC_MAX = 4095;  // 12-bit ADC

// Gas sensor calibration (ADC -> "BME688-equivalent" mapping)
int MQ8_BME_min = 10700;
int MQ8_BME_max = 13800;
int MQ3_BME_min = 15300;
int MQ3_BME_max = 21400;

// ====== SENSOR VARIABLES ======
sensors_event_t humidity_event, temp_event, pressure_event;
int mq8_raw = 0;
int mq3_raw = 0;

// ====== TEST MODE CONFIGURATION ======
// Set to true to test with known values, false for live sensor readings
#define TEST_MODE true

// Test data from your example (Fire Alarm = 1)
const float TEST_DATA[6] = {
  10.002,   // Temperature [°C]
  52.55,    // Humidity [%]
  631,      // eCO2 [ppm]
  12802,    // Raw H2
  19471,    // Raw Ethanol
  939.097   // Pressure [hPa]
};
const int TEST_EXPECTED_RESULT = 1; // Expected: Fire Alarm = 1

// ====== FUNCTION DECLARATIONS ======
void readAllSensors();
void readGasSensors();
void printToSerial(float features[], long mq8_bme, long mq3_bme,
                   float probability, int risk, unsigned long inference_us, int8_t output_raw);
void updateLCDDisplay(long mq8_bme, long mq3_bme, float probability, int risk);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ESP32 Fire Detection (TFLite INT8) ---");

  // ================= I2C and ADC setup =================
  Wire.begin(21, 22); // SDA = 21, SCL = 22

  pinMode(MQ3_PIN, INPUT);
  pinMode(MQ8_PIN, INPUT);
  analogSetPinAttenuation(MQ3_PIN, ADC_11db);
  analogSetPinAttenuation(MQ8_PIN, ADC_11db);

  // ================= LCD =================
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

  // ================= Sensors init =================
  Serial.println("Init sensors...");
  if (!aht.begin()) {
    Serial.println("AHT sensor not found! Check wiring.");
    lcd.clear(); lcd.print("AHT Error!");
    while (1) delay(1000);
  }
  Serial.println("AHT OK");

  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
    lcd.clear(); lcd.print("BMP Error!");
    while (1) delay(1000);
  }
  Serial.println("BMP OK");

  if (ens160.begin() != NO_ERR) {
    Serial.println("ENS160 not found!");
    lcd.clear(); lcd.print("ENS160 Error!");
    while (1) delay(1000);
  }
  Serial.println("ENS160 OK");
  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(25.0, 50.0);

  // ================= Load TFLite model =================
  Serial.println("\nLoading TFLite model from header...");
  model = tflite::GetModel(tiny_model_int8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("Model schema version %d not equal to expected %d\n",
                  model->version(), TFLITE_SCHEMA_VERSION);
    lcd.clear(); lcd.print("Model Schema Err");
    while (1) delay(1000);
  }

  // Add required ops
  resolver.AddFullyConnected();
  resolver.AddLogistic();
  resolver.AddQuantize();
  resolver.AddDequantize();
  resolver.AddReshape();
  resolver.AddAdd();
  resolver.AddMul();
  resolver.AddSub();

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("Failed to allocate tensors - increase arena size or add missing ops.");
    while (1) delay(1000);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("TFLite model loaded successfully:");
  Serial.printf(" Input shape: [");
  for (int i=0; i<input->dims->size; ++i) 
    Serial.printf("%d%s", input->dims->data[i], i+1<input->dims->size ? ", ":"");
  Serial.println(" ]");
  Serial.printf(" Input quant: scale=%.8f, zero_point=%d\n", INPUT_SCALE, INPUT_ZERO_POINT);
  Serial.printf(" Output quant: scale=%.8f, zero_point=%d\n", OUTPUT_SCALE, OUTPUT_ZERO_POINT);
  Serial.printf(" Arena used: %d / %d bytes (%.1f%%)\n",
                interpreter->arena_used_bytes(), kTensorArenaSize,
                (float)interpreter->arena_used_bytes() * 100.0f / (float)kTensorArenaSize);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("System Ready!");
  lcd.setCursor(0,1); lcd.print("Monitoring...");
  delay(1000);
  lcd.clear();

  Serial.println("--- Starting loop ---");
}

void loop() {
  float raw_features[6];
  long mq8_bme_eq, mq3_bme_eq;

#if TEST_MODE
  // ========== TEST MODE: Use predefined values ==========
  Serial.println("\n***** TEST MODE ENABLED *****");
  Serial.println("Using example data from table:");
  
  // Use test data directly
  for (int i = 0; i < 6; i++) {
    raw_features[i] = TEST_DATA[i];
  }
  mq8_bme_eq = (long)TEST_DATA[3];
  mq3_bme_eq = (long)TEST_DATA[4];
  
  Serial.printf("Expected Result: Fire Alarm = %d\n", TEST_EXPECTED_RESULT);
  Serial.println("*****************************\n");
#else
  // ========== LIVE MODE: Read from sensors ==========
  readAllSensors();

  // Convert MQ raw ADC readings to BME688-equivalent scale
  mq8_bme_eq = MQ8_BME_min + (long)mq8_raw * (MQ8_BME_max - MQ8_BME_min) / ADC_MAX;
  mq3_bme_eq = MQ3_BME_min + (long)mq3_raw * (MQ3_BME_max - MQ3_BME_min) / ADC_MAX;

  // Prepare raw sensor features (before normalization)
  raw_features[0] = temp_event.temperature;          // Temp
  raw_features[1] = humidity_event.relative_humidity;// Humidity
  raw_features[2] = (float)ens160.getECO2();         // eCO2
  raw_features[3] = (float)mq8_bme_eq;               // H2
  raw_features[4] = (float)mq3_bme_eq;               // Ethanol
  raw_features[5] = pressure_event.pressure;         // Pressure
#endif

  Serial.println("\n========== SENSOR READINGS ==========");
  Serial.printf(" Temperature: %.2f °C\n", raw_features[0]);
  Serial.printf(" Humidity: %.2f %%\n", raw_features[1]);
  Serial.printf(" eCO2: %.0f ppm\n", raw_features[2]);
  Serial.printf(" H2: %ld (MQ8 equiv)\n", mq8_bme_eq);
  Serial.printf(" Ethanol: %ld (MQ3 equiv)\n", mq3_bme_eq);
  Serial.printf(" Pressure: %.2f hPa\n", raw_features[5]);

  // ========== PREPROCESSING & QUANTIZATION ==========
  Serial.println("\n========== PREPROCESSING ==========");
  int8_t quantized_input[6];
  
  for (int i = 0; i < 6; i++) {
    // Step 1: Normalize (Z-score standardization)
    float normalized = (raw_features[i] - feature_mean[i]) / feature_std[i];
    
    // Step 2: Quantize to int8
    float scaled = (normalized / INPUT_SCALE) + INPUT_ZERO_POINT;
    int quant_value = (int)round(scaled);
    
    // Step 3: Clamp to int8 range [-128, 127]
    quantized_input[i] = (int8_t)max(-128, min(127, quant_value));
    
    const char* names[] = {"Temp", "Humid", "CO2", "H2", "EtOH", "Press"};
    Serial.printf(" [%d] %s: %.3f -> normalized: %.4f -> quantized: %d\n", 
                  i, names[i], raw_features[i], normalized, quantized_input[i]);
  }

  Serial.print("\nQuantized Input Array: [");
  for (int i=0; i<6; i++) {
    Serial.print((int)quantized_input[i]);
    if (i<5) Serial.print(", ");
  }
  Serial.println("]");

  // ========== INFERENCE ==========
  // Copy quantized data to model input tensor
  memcpy(input->data.int8, quantized_input, sizeof(quantized_input));
  
  unsigned long start = micros();
  TfLiteStatus invoke_status = interpreter->Invoke();
  unsigned long inference_time = micros() - start;

  if (invoke_status != kTfLiteOk) {
    Serial.println("ERROR: Inference failed!");
    lcd.clear(); 
    lcd.setCursor(0,0); 
    lcd.print("Inference Err");
    delay(1000);
    return;
  }

  // ========== POSTPROCESSING ==========
  Serial.println("\n========== POSTPROCESSING ==========");
  
  // Read raw int8 output
  int8_t raw_output = output->data.int8[0];
  Serial.printf(" Raw Output (int8): %d\n", raw_output);
  
  // Dequantize to float probability
  float probability = (raw_output - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
  
  // Clamp to valid probability range [0.0, 1.0]
  probability = max(0.0f, min(1.0f, probability));
  
  Serial.printf(" Dequantized Probability: %.4f (%.2f%%)\n", 
                probability, probability * 100.0f);
  Serial.printf(" Decision Threshold: 0.50\n");
  Serial.printf(" Inference Time: %lu us (%.2f ms)\n", 
                inference_time, inference_time/1000.0f);
  
  // Risk classification
  int risk = (probability > 0.5f) ? 1 : 0;
  
  Serial.println("\n========== RESULT ==========");
  if (risk == 1) {
    Serial.println(" 🔥 FIRE RISK DETECTED!");
  } else {
    Serial.println(" ✅ No fire risk detected.");
  }
  
#if TEST_MODE
  // Validate test result
  Serial.println("\n--- TEST VALIDATION ---");
  Serial.printf(" Expected: Fire Alarm = %d\n", TEST_EXPECTED_RESULT);
  Serial.printf(" Actual:   Fire Alarm = %d\n", risk);
  if (risk == TEST_EXPECTED_RESULT) {
    Serial.println(" ✓ TEST PASSED - Model output matches expected result!");
  } else {
    Serial.println(" ✗ TEST FAILED - Model output does NOT match expected result!");
    Serial.println(" Check: quantization parameters, normalization, or model file");
  }
  Serial.println("-----------------------");
#endif
  
  Serial.println("====================================\n");

  // Update displays
  updateLCDDisplay(mq8_bme_eq, mq3_bme_eq, probability, risk);

  delay(2000); // 2 second update rate
}

// ====== SENSOR READ FUNCTIONS ======
void readAllSensors() {
  // Temperature & Humidity
  aht.getEvent(&humidity_event, &temp_event);
  // Pressure
  bmp.getEvent(&pressure_event);
  // Update ENS160 with ambient readings
  ens160.setTempAndHum(temp_event.temperature, humidity_event.relative_humidity);
  // Read gas sensors
  readGasSensors();
}

void readGasSensors() {
  const int samples = 6;
  long sum3 = 0, sum8 = 0;
  for (int i=0; i<samples; i++) {
    sum3 += analogRead(MQ3_PIN);
    sum8 += analogRead(MQ8_PIN);
    delay(10);
  }
  mq3_raw = sum3 / samples;
  mq8_raw = sum8 / samples;
}

// ====== DISPLAY FUNCTIONS ======
void updateLCDDisplay(long mq8_bme, long mq3_bme, float probability, int risk) {
  lcd.clear();
  
  // Line 1: Temperature and Humidity
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.print(temp_event.temperature,1);
  lcd.print("C H:");
  lcd.print(humidity_event.relative_humidity,0);
  lcd.print("%");

  // Line 2: Pressure and CO2
  lcd.setCursor(0,1);
  lcd.print("P:");
  lcd.print(pressure_event.pressure,0);
  lcd.print(" CO2:");
  lcd.print(ens160.getECO2());

  // Line 3: H2 and Probability
  lcd.setCursor(0,2);
  lcd.print("H2:");
  lcd.print(mq8_bme);
  lcd.setCursor(9,2);
  lcd.print((int)(probability*100));
  lcd.print("%");

  // Line 4: Ethanol and Status
  lcd.setCursor(0,3);
  lcd.print("Et:");
  lcd.print(mq3_bme);
  lcd.setCursor(10,3);
  if (risk==1) {
    lcd.print("FIRE!");
  } else {
    lcd.print("OK   ");
  }
}