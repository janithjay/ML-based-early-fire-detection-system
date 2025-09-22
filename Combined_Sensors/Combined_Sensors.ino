#include <Wire.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <LiquidCrystal_I2C.h>

// === Sensor Objects ===
DFRobot_ENS160_I2C ens160(&Wire, 0x53);    // ENS160 I2C address
Adafruit_AHTX0 aht;                         // AHT21 temperature & humidity
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // BMP180 pressure
LiquidCrystal_I2C lcd(0x27, 16, 4);        // LCD 16x4 I2C

// === Pin Definitions ===
// CD74HC4067 Multiplexer Control Pins
#define MUX_S0 D5        // Multiplexer select pin S0
#define MUX_S1 D6        // Multiplexer select pin S1  
#define MUX_S2 D7        // Multiplexer select pin S2
#define MUX_S3 D8        // Multiplexer select pin S3
#define MUX_SIG A0       // Multiplexer signal pin (to ESP8266 A0)

// Multiplexer channels for sensors
#define MQ3_CHANNEL 0    // MQ3 connected to channel 0 (C0)
#define MQ8_CHANNEL 1    // MQ8 connected to channel 1 (C1)

// === Variables ===
sensors_event_t humidity, temp, pressure_event;

// Gas sensor data variables
int mq8_raw = 0;          // MQ8 data from multiplexer channel 1
int mq3_raw = 0;          // MQ3 data from multiplexer channel 0

// Gas sensor calibration parameters
// MQ8 (Hydrogen) parameters
int MQ8_BME_min = 10700;  // minimum raw H2 value equivalent
int MQ8_BME_max = 13800;  // maximum raw H2 value equivalent

// MQ3 (Ethanol/Alcohol) parameters  
int MQ3_BME_min = 15300;  // minimum raw ethanol value equivalent
int MQ3_BME_max = 21400;  // maximum raw ethanol value equivalent

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C (for ESP8266/NodeMCU)
  Wire.begin(D2, D1);   // SDA=D2, SCL=D1
  // For Arduino Uno, use: Wire.begin();

  // Initialize multiplexer control pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Multi-Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Smoke Detection");
  lcd.setCursor(0, 2);
  lcd.print("System Starting...");
  delay(3000);
  lcd.clear();

  // Initialize AHT21
  if (!aht.begin()) {
    Serial.println("AHT21 not found!");
    lcd.setCursor(0, 0);
    lcd.print("AHT21 Error!");
    while (1);
  }
  Serial.println("AHT21 initialized successfully");

  // Initialize BMP180
  if(!bmp.begin()) {
    Serial.println("BMP180 not found!");
    lcd.setCursor(0, 1);
    lcd.print("BMP180 Error!");
    while(1);
  }
  Serial.println("BMP180 initialized successfully");

  // Initialize ENS160
  if (ens160.begin() != NO_ERR) {
    Serial.println("ENS160 not found!");
    lcd.setCursor(0, 2);
    lcd.print("ENS160 Error!");
    while (1);
  }
  Serial.println("ENS160 initialized successfully");

  // Configure ENS160
  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(25.0, 50.0);  // initial values
  
  Serial.println("All sensors initialized successfully!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("All Sensors Ready");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Read all sensors
  readAllSensors();
  
  // Display on Serial Monitor
  printToSerial();
  
  // Update LCD display
  updateLCDDisplay();
  
  delay(2000);  // Update every 2 seconds
}

void readAllSensors() {
  // Read AHT21 (temperature and humidity)
  aht.getEvent(&humidity, &temp);
  
  // Read BMP180 (pressure)
  bmp.getEvent(&pressure_event);
  
  // Update ENS160 with current temp/humidity for better accuracy
  ens160.setTempAndHum(temp.temperature, humidity.relative_humidity);
  
  // Read MQ3 and MQ8 sensors via multiplexer
  readGasSensors();
}

// Function to select multiplexer channel
void selectMuxChannel(int channel) {
  // Set the 4-bit channel selection (S3, S2, S1, S0)
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);
  
  delay(10); // Small delay for multiplexer to settle
}

// Function to read from specific multiplexer channel
int readMuxChannel(int channel) {
  selectMuxChannel(channel);
  return analogRead(MUX_SIG);
}

// Function to read both gas sensors
void readGasSensors() {
  mq3_raw = readMuxChannel(MQ3_CHANNEL);
  mq8_raw = readMuxChannel(MQ8_CHANNEL);
}

void printToSerial() {
  // Print all sensor readings to Serial Monitor
  Serial.println("=== Sensor Readings ===");
  Serial.print("Temperature: "); Serial.print(temp.temperature, 1); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity, 1); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(pressure_event.pressure, 1); Serial.println(" hPa");
  
  // ENS160 readings
  uint16_t eco2 = ens160.getECO2();
  uint16_t tvoc = ens160.getTVOC();
  Serial.print("eCO2: "); Serial.print(eco2); Serial.println(" ppm");
  Serial.print("TVOC: "); Serial.print(tvoc); Serial.println(" ppb");
  
  // Gas sensors
  // Calculate BME688 equivalents
  long mq8_bme_eq = MQ8_BME_min + (long)mq8_raw * (MQ8_BME_max - MQ8_BME_min) / 1023;
  long mq3_bme_eq = MQ3_BME_min + (long)mq3_raw * (MQ3_BME_max - MQ3_BME_min) / 1023;
  
  Serial.print("MQ8 (H2) Raw: "); Serial.print(mq8_raw);
  Serial.print(" | BME Equivalent: "); Serial.println(mq8_bme_eq);
  Serial.print("MQ3 (Ethanol) Raw: "); Serial.print(mq3_raw);
  Serial.print(" | BME Equivalent: "); Serial.println(mq3_bme_eq);
  Serial.println("========================");
}

void updateLCDDisplay() {
  lcd.clear();
  
  // Calculate BME688 equivalents
  long mq8_bme_eq = MQ8_BME_min + (long)mq8_raw * (MQ8_BME_max - MQ8_BME_min) / 1023;
  long mq3_bme_eq = MQ3_BME_min + (long)mq3_raw * (MQ3_BME_max - MQ3_BME_min) / 1023;
  
  // Display all sensor values on one screen
  // Line 1: Temperature and Humidity
  lcd.setCursor(0, 0);
  lcd.print("Tem:"); lcd.print(temp.temperature, 1); 
  lcd.print("C  Humi:"); lcd.print(humidity.relative_humidity, 0); lcd.print("%");
  
  // Line 2: Pressure and eCO2
  lcd.setCursor(0, 1);
  lcd.print("Pres:"); lcd.print(pressure_event.pressure, 0);
  lcd.print("  CO2:"); lcd.print(ens160.getECO2());
  
  // Line 3: TVOC and MQ8 (Hydrogen via multiplexer)
  lcd.setCursor(0, 2);
  lcd.print("TVOC:"); lcd.print(ens160.getTVOC());
  lcd.print("    H2:"); lcd.print(mq8_bme_eq);
  
  // Line 4: MQ3 (Ethanol/Alcohol via multiplexer)
  lcd.setCursor(0, 3);
  lcd.print("Ethanol:"); lcd.print(mq3_bme_eq);
}