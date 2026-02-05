# ML-Based Early Fire Detection System 🔥

An intelligent fire detection system built on ESP32 that uses Machine Learning and multiple environmental sensors to detect fire risks in real-time. The system leverages TensorFlow Lite for on-device inference and integrates with Firebase for cloud data logging and alerting.

## 🌟 Features

- **Real-time Fire Detection**: Uses a trained TensorFlow Lite model for on-device inference
- **Multi-Sensor Approach**: Combines data from multiple sensors for accurate detection
  - Temperature and Humidity (AHT20)
  - Atmospheric Pressure (BMP180)
  - Air Quality (ENS160 - eCO2)
  - Hydrogen Detection (MQ8)
  - Ethanol Detection (MQ3)
- **Firebase Integration**: Real-time data logging and alert system
- **LCD Display**: 16x4 I2C LCD for local status monitoring
- **Audible Alert**: Buzzer with beeping pattern when fire is detected
- **Calibration Mode**: Automatic sensor calibration for different environments
- **Demo Mode**: Test the system with pre-recorded fire data
- **Time Synchronization**: NTP-based timestamping for accurate logs

## 🎯 System Architecture

The system uses a quantized INT8 TensorFlow Lite model trained to detect fire conditions based on environmental parameters. The model processes 6 input features:
1. eCO2 levels (ppm)
2. Humidity (%)
3. Atmospheric Pressure (hPa)
4. Raw Ethanol reading
5. Raw Hydrogen reading
6. Temperature (°C)

The model outputs a fire probability (0-100%) and triggers an alarm if the probability exceeds 50%.

## 🛠️ Hardware Requirements

### Required Components

1. **ESP32 Development Board** (with WiFi capability)
2. **Sensors**:
   - AHT20 - Temperature and Humidity Sensor
   - BMP180 - Barometric Pressure Sensor
   - ENS160 - Air Quality Sensor (I2C, address 0x53)
   - MQ3 - Alcohol/Ethanol Gas Sensor (analog)
   - MQ8 - Hydrogen Gas Sensor (analog)
3. **Display**: 16x4 I2C LCD (address 0x27)
4. **Buzzer**: Active buzzer (connected to GPIO 25)
5. **Push Buttons**: 
   - Test/Demo button (GPIO 32)
   - Calibration button (GPIO 33)
6. **Miscellaneous**:
   - Jumper wires
   - Breadboard
   - Power supply (5V/3.3V)

### Pin Connections

| Component | ESP32 Pin |
|-----------|-----------|
| I2C SDA | GPIO 21 |
| I2C SCL | GPIO 22 |
| MQ3 Analog | GPIO 34 |
| MQ8 Analog | GPIO 35 |
| Buzzer | GPIO 25 |
| Test Button | GPIO 32 (with internal pull-up) |
| Calibration Button | GPIO 33 (with internal pull-up) |

## 📚 Software Requirements

### Arduino IDE Setup

1. Install [Arduino IDE](https://www.arduino.cc/en/software) (version 1.8.x or 2.x)
2. Add ESP32 board support:
   - Go to File > Preferences
   - Add `https://dl.espressif.com/dl/package_esp32_index.json` to "Additional Board Manager URLs"
   - Go to Tools > Board > Boards Manager
   - Search for "esp32" and install "ESP32 by Espressif Systems"

### Required Libraries

Install the following libraries via Arduino Library Manager (Sketch > Include Library > Manage Libraries):

1. **Firebase ESP Client** by Mobizt
   - Provides Firebase Realtime Database connectivity
2. **DFRobot_ENS160** by DFRobot
   - Driver for ENS160 air quality sensor
3. **Adafruit AHTX0** by Adafruit
   - Driver for AHT20 temperature/humidity sensor
4. **Adafruit BMP085 Unified** by Adafruit
   - Driver for BMP180 pressure sensor
5. **LiquidCrystal I2C** by Frank de Brabander
   - Driver for I2C LCD displays
6. **TensorFlow Lite Micro** (Arduino_TensorFlowLite)
   - Machine learning inference engine

### Additional Dependencies

- **Time library** (built-in with ESP32 core)
- **WiFi library** (built-in with ESP32 core)
- **Wire library** (built-in with ESP32 core)

## ⚙️ Configuration

### 1. WiFi Configuration

Update the WiFi credentials in `esp32.ino`:

```cpp
#define WIFI_SSID "Your_WiFi_SSID"
#define WIFI_PASSWORD "Your_WiFi_Password"
```

### 2. Firebase Configuration

1. Create a Firebase project at [Firebase Console](https://console.firebase.google.com/)
2. Create a Realtime Database
3. Get your database URL and authentication secret:
   - Go to Project Settings > Service Accounts > Database Secrets
   - Copy the database URL and secret token

Update the Firebase configuration in `esp32.ino`:

```cpp
#define DATABASE_URL "https://your-project.firebaseio.com/"
#define DATABASE_SECRET "your_database_secret_token"
```

### 3. Timezone Configuration

Adjust the timezone offset for accurate timestamps:

```cpp
const long gmtOffset_sec = 5*3600;  // UTC+5:30 example
const int daylightOffset_sec = 0;   // Daylight saving time offset
```

## 🚀 Installation & Setup

### Step 1: Hardware Assembly

1. Connect all sensors to ESP32 according to the pin connections table
2. Ensure I2C devices (ENS160, AHT20, BMP180, LCD) share the same I2C bus
3. Connect analog gas sensors (MQ3, MQ8) to specified GPIO pins
4. Connect buzzer to GPIO 25
5. Connect push buttons to GPIO 32 and 33 with internal pull-up resistors

### Step 2: Software Upload

1. Open `esp32/esp32.ino` in Arduino IDE
2. Select your ESP32 board: Tools > Board > ESP32 Arduino > ESP32 Dev Module
3. Select the correct COM port: Tools > Port
4. Configure WiFi and Firebase credentials as described above
5. Click Upload button to flash the code to ESP32

### Step 3: Initial Calibration

Upon first boot, the system should be calibrated for your environment:

1. Ensure there is **NO fire or smoke** in the environment
2. Press and **HOLD** the calibration button (GPIO 33) for **3 seconds**
3. The system will collect 20 samples over 40 seconds
4. The buzzer will beep 3 times to confirm calibration completion
5. The device is now calibrated and ready for use

## 📖 Usage

### Normal Operation

Once powered on and calibrated:
- The system reads sensors every 2 seconds
- LCD displays real-time sensor readings and fire probability
- Data is automatically logged to Firebase
- If fire is detected (probability > 50%):
  - LCD shows "FIRE!" status
  - Buzzer activates with beeping pattern (500ms on/off)
  - Alert is logged to Firebase with timestamp and sensor data

### Demo Mode

To test the system with pre-recorded fire data:

1. Press the **Test Button** (GPIO 32) once
2. System enters demo mode with 8 pre-recorded fire samples
3. Each sample is processed every 3 seconds
4. Press the button again to exit demo mode

### Recalibration

To recalibrate for a different environment:
1. Ensure no fire or smoke present
2. Press and HOLD calibration button (GPIO 33) for 3 seconds
3. Wait for 40 seconds as system collects samples
4. Buzzer beeps 3 times when complete

## 📊 Firebase Data Structure

The system organizes data in Firebase as follows:

```
/sensors
  /current              # Latest sensor readings
    - temperature
    - humidity
    - pressure
    - eco2
    - rawH2
    - rawEthanol
    - fireProbability
    - fireAlarm
    - dateTime
    - unixTimestamp
  
  /history              # Historical data organized by date
    /2024-01-15
      /{timestamp}      # Unix timestamp as key
        - [sensor data]
    /2024-01-16
      /{timestamp}
        - [sensor data]

/alerts                 # Fire detection alerts
  /2024-01-15
    /{timestamp}
      - probability
      - dateTime
      - [sensor data at time of alert]
```

## 🧮 Machine Learning Model

The system uses a quantized INT8 TensorFlow Lite model:
- **Input**: 6 standardized environmental features
- **Architecture**: Fully connected neural network
- **Output**: Fire probability (0.0 to 1.0)
- **Quantization**: INT8 for efficient inference on ESP32
- **Model Size**: ~8KB (stored in `tiny_model_int8.h`)

### Model Training Parameters

The model uses pre-calculated standardization parameters:
- Feature means and standard deviations from training data
- Input quantization scale: 0.0247
- Output quantization scale: 0.00391

## 🔧 Troubleshooting

### Sensor Initialization Errors

If any sensor fails to initialize:
- Check I2C connections (SDA: GPIO 21, SCL: GPIO 22)
- Verify sensor addresses (ENS160: 0x53, LCD: 0x27)
- Ensure proper power supply to sensors

### WiFi Connection Issues

- Verify SSID and password are correct
- Check WiFi signal strength
- Ensure ESP32 is within range of router

### Firebase Connection Problems

- Verify database URL format includes `https://` and ends with `/`
- Check database secret is valid and not expired
- Ensure Firebase Realtime Database is created and rules allow access

### False Positives/Negatives

- Run calibration in your specific environment
- Ensure sensors have warmed up (2-3 minutes after power-on)
- Check that gas sensors (MQ3, MQ8) are properly heated

## 📝 Serial Monitor Output

The system provides detailed logging via Serial Monitor (115200 baud):
- Initialization status for all components
- Real-time sensor readings every 2 seconds
- Inference results with fire probability
- Firebase upload confirmations
- Calibration progress and results

## 🔐 Security Considerations

- **Do not commit credentials**: Remove WiFi and Firebase credentials before sharing code
- **Firebase Security Rules**: Configure appropriate read/write rules in Firebase Console
- **Network Security**: Use WPA2/WPA3 encrypted WiFi networks

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## 📄 License

This project is open source and available for educational and research purposes.

## 👥 Authors

- **Janith Jay** - [janithjay](https://github.com/janithjay)

## 🙏 Acknowledgments

- TensorFlow Lite Micro team for embedded ML framework
- Adafruit and DFRobot for sensor libraries
- Firebase team for real-time database platform

## 📮 Support

For issues, questions, or suggestions, please open an issue on the GitHub repository.

---

**⚠️ Safety Notice**: This is a prototype system for educational purposes. Do not rely solely on this system for critical fire safety. Always use certified fire detection systems and follow local fire safety regulations.
