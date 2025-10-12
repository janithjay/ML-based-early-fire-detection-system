#include <Arduino.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
//#include "tensorflow/lite/version.h"

// Include your converted model header (must be in the same folder)
#include "tiny_model_int8.h"

// ====== CONFIGURE MEMORY ======
constexpr int kTensorArenaSize = 10 * 1024;  // 10 KB buffer for model tensors
uint8_t tensor_arena[kTensorArenaSize];

// ====== SETUP TFLITE INTERPRETER ======
const tflite::Model* model = tflite::GetModel(tiny_model_int8_tflite);
tflite::MicroMutableOpResolver<10> resolver;  // We'll manually register ops
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing Tiny Neural Network on ESP32...");

  // ====== Register Only Needed Operations ======
  resolver.AddFullyConnected();
  resolver.AddQuantize();
  resolver.AddDequantize();
  resolver.AddSoftmax();
  resolver.AddReshape();
  resolver.AddConv2D();
  resolver.AddLogistic();
  // <-- Add or remove based on your model
  // If your model doesn't use Conv2D, you can remove that line.

  // ====== Model Version Check ======
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("Model schema version mismatch: %d vs %d\n",
                  model->version(), TFLITE_SCHEMA_VERSION);
    while (1)
      ;
  }

  // ====== Create Interpreter ======
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // ====== Allocate Memory for Tensors ======
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Failed to allocate tensors!");
    while (1)
      ;
  }

  // ====== Get Input & Output References ======
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("Tiny Neural Network ready!");
}

void loop() {
  // ====== EXAMPLE INPUT ======
  // Replace with your own sensor or data input (6 features)
  float example_input[6] = { 0.12, -0.34, 0.56, -0.22, 0.48, 0.10 };

  // Quantize input (float → int8)
  for (int i = 0; i < 6; i++) {
    input->data.int8[i] = (int8_t)(example_input[i] * 127.0f);  // adjust scale if needed
  }

  // ====== Run Inference ======
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed!");
    delay(2000);
    return;
  }

  // ====== Read Output (int8 → float) ======
  int8_t output_val = output->data.int8[0];
  float prediction = (float)output_val / 127.0f;

  Serial.print("Model prediction (float): ");
  Serial.println(prediction, 4);

  delay(2000);
}
