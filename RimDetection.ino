/*
Spikeball Impact Detection System
Using MPU6500 6-Axis Gyroscope + Accelerometer
Author: Matthew Lee  

ORIENTATION: MPU6500 should be mounted with Z-axis pointing UP (away from net surface)
  
WIRING MPU6500:
- VCC → 3.3V
- GND → GND  
- SCL → A5
- SDA → A4
- INT → Pin 8
  
LED STRIP + MOFSET WIRING:
- Arduino Pin D3 → 220Ω resistor → MOSFET Gate

- Power Bank +5V → LED Strip (+)

- LED Strip (-) → MOSFET Drain  

- MOSFET Source → Power Bank Ground
- Arduino GND → Power Bank Ground
*/

#include <Wire.h>

// MPU6500 I2C address
#define MPU6500_ADDR 0x68

// MPU6500 Register addresses
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B

// Pin definitions
const int LED_STRIP_PIN = 3;    
const int STATUS_LED = 13;
const int MPU_INT_PIN = 8; //Optional at the Moment

// LED brightness levels
const int BRIGHTNESS_LOW = 50;
const int BRIGHTNESS_MID = 150;
const int BRIGHTNESS_HIGH = 255;

// Impact detection thresholds
// Based on actual data analysis: GyroY > 0 = rim hit, GyroY < 0 = net hit
const float GYROY_THRESHOLD = 25.0;        // deg/s; minimum GyroY magnitude to consider impact
const float TOTAL_GYRO_IMPACT = 20.0;      // deg/s; minimum total rotation to consider impact
const int IMPACT_DURATION_MIN = 30;        // Minimum impact duration (ms) - reduced for faster response
const int DEBOUNCE_TIME = 800;             // Prevent multiple triggers - reduced for better responsiveness

// Flash timing
const unsigned long DIM_TIME = 50;      
const unsigned long DIM_HOLD = 30;       
const unsigned long FLASH_TIME = 20;    
const unsigned long FADE_TIME = 300;    

// Rim flash timing - 5 flashes over 2.5 seconds
const int RIM_FLASH_COUNT = 5;
const unsigned long RIM_FLASH_TOTAL_TIME = 2500; // 2.5 seconds
const unsigned long RIM_FLASH_ON_TIME = 200;     // Each flash on time
const unsigned long RIM_FLASH_OFF_TIME = 300;    // Each flash off time

// Variables
struct SensorData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float totalAccel;
  float totalGyro;
  float lateralAccel;
};

SensorData current, baseline;
bool flashActive = false;
unsigned long flashStartTime = 0;
unsigned long lastImpactTime = 0;

// Rim flash variables
bool rimFlashActive = false;
unsigned long rimFlashStartTime = 0;
int currentRimFlash = 0;
bool rimFlashState = false; // true = on, false = off

// Calibration variables
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Moving average filter for gyroscope data
const int FILTER_SIZE = 5;
float gyroYBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Peak detection variables
float lastGyroY = 0;
float peakGyroY = 0;
unsigned long peakTime = 0;
bool peakDetected = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(LED_STRIP_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  
  // Initialize LED strip to base brightness
  analogWrite(LED_STRIP_PIN, BRIGHTNESS_MID);
  digitalWrite(STATUS_LED, HIGH);
  
  Serial.println("Spikeball Impact Detection System");
  Serial.println("===================================================");
  Serial.println();
  
  // Initialize MPU6500
  if (initMPU6500()) {
    Serial.println("MPU6500 initialized successfully.");
    // Give time to calibrate sensor
    Serial.println("Calibrating... Keep still.");
    calibrateSensor();
    Serial.println("Calibration complete.");
    
    // Take baseline reading
    readSensorData(baseline);
    Serial.println("Reading Data.");
    Serial.println();
    
  } else {
    Serial.println("Failed to initialize MPU6500!");
    while(1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
  }
}

void loop() {
  // Read current sensor data
  readSensorData(current);
  
  // Check for impact
  checkForImpact();
  
  // Handle flash sequence if active
  if (rimFlashActive) {
    handleRimFlashSequence();
  } else if (flashActive) {
    handleFlashSequence();
  }
  
  // Enable debugging output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    //printDebugInfo();
    lastPrint = millis();
  }

  // Disable constant debug output - only show impact detections
  /*
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    printDebugInfo();
    lastPrint = millis();
  }
  */

  delay(10);
}

bool initMPU6500() {
  // Wake up MPU6500
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  
  // Check WHO_AM_I register
  uint8_t whoami = readRegister(0x75);
  if (whoami != 0x70) {  // MPU6500 WHO_AM_I value
    Serial.print("Wrong WHO_AM_I: 0x");
    Serial.println(whoami, HEX);
    return false;
  }
  
  // Configure accelerometer (±4g range for better sensitivity)
  writeRegister(ACCEL_CONFIG, 0x08);
  
  // Configure gyroscope (±500°/s range for better sensitivity)
  writeRegister(GYRO_CONFIG, 0x08);
  
  // Set sample rate divider (1kHz sample rate)
  writeRegister(0x19, 0x07);
  
  // Configure digital low pass filter (bandwidth 5Hz for better noise reduction)
  writeRegister(CONFIG, 0x06);
  
  delay(100);
  return true;
}

void calibrateSensor() {
  const int numSamples = 1000;
  float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  
  for (int i = 0; i < numSamples; i++) {
    SensorData temp;
    readSensorData(temp);
    
    sumAccelX += temp.accelX;
    sumAccelY += temp.accelY;
    sumAccelZ += temp.accelZ;
    sumGyroX += temp.gyroX;
    sumGyroY += temp.gyroY;
    sumGyroZ += temp.gyroZ;
    
    delay(2);
  }
  
  accelOffsetX = sumAccelX / numSamples;
  accelOffsetY = sumAccelY / numSamples;
  accelOffsetZ = sumAccelZ / numSamples;
  gyroOffsetX = sumGyroX / numSamples;
  gyroOffsetY = sumGyroY / numSamples;
  gyroOffsetZ = sumGyroZ / numSamples;
  
  Serial.print("Calibration offsets - X: ");
  Serial.print(accelOffsetX, 3);
  Serial.print(", Y: ");
  Serial.print(accelOffsetY, 3);
  Serial.print(", Z: ");
  Serial.println(accelOffsetZ, 3);
}

void readSensorData(SensorData &data) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 14, true);
  
  // Read accelerometer data
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();
  
  // Skip temperature data
  Wire.read(); Wire.read();
  
  // Read gyroscope data
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert to meaningful units and apply calibration
  data.accelX = (accelX / 8192.0) - accelOffsetX;  // ±4g range
  data.accelY = (accelY / 8192.0) - accelOffsetY;
  data.accelZ = (accelZ / 8192.0) - accelOffsetZ;  // This should be primary detection axis
  
  data.gyroX = (gyroX / 65.5) - gyroOffsetX;  // ±500°/s range
  data.gyroY = (gyroY / 65.5) - gyroOffsetY;
  data.gyroZ = (gyroZ / 65.5) - gyroOffsetZ;
  
  // Calculate total magnitudes
  data.totalAccel = sqrt(data.accelX*data.accelX + 
                        data.accelY*data.accelY + 
                        data.accelZ*data.accelZ);
  
  data.totalGyro = sqrt(data.gyroX*data.gyroX + 
                       data.gyroY*data.gyroY + 
                       data.gyroZ*data.gyroZ);
  
  // Calculate lateral (X-Y plane) acceleration for rim detection
  data.lateralAccel = sqrt(data.accelX*data.accelX + data.accelY*data.accelY);
  
  // Apply moving average filter to GyroY
  data.gyroY = filterGyroY(data.gyroY);
}

// Moving average filter for GyroY to reduce noise
float filterGyroY(float newValue) {
  gyroYBuffer[bufferIndex] = newValue;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
  
  if (bufferIndex == 0) {
    bufferFilled = true;
  }
  
  float sum = 0;
  int count = bufferFilled ? FILTER_SIZE : bufferIndex;
  
  for (int i = 0; i < count; i++) {
    sum += gyroYBuffer[i];
  }
  
  return sum / count;
}

void triggerFlash(String hitType = "unknown") {
  if (hitType == "rim") {
    if (!rimFlashActive) {
      rimFlashActive = true;
      rimFlashStartTime = millis();
      currentRimFlash = 0;
      rimFlashState = false;
      Serial.println("Triggering RIM flash pattern");
    }
  } else if (hitType == "net") {
    if (!flashActive) {
      flashActive = true;
      flashStartTime = millis();
      Serial.println("Triggering NET flash pattern");
    }
  }
}

void checkForImpact() {
  unsigned long currentTime = millis();
  
  // Check if we're in debounce period
  if (currentTime - lastImpactTime < DEBOUNCE_TIME) {
    return;
  }
  
  // Peak detection for GyroY
  if (abs(current.gyroY) > abs(lastGyroY) && abs(current.gyroY) > GYROY_THRESHOLD) {
    // We're climbing towards a peak
    if (!peakDetected) {
      peakDetected = true;
      peakTime = currentTime;
    }
    peakGyroY = current.gyroY;
  } else if (peakDetected && (currentTime - peakTime) > 20) {
    // Peak has passed, analyze the impact
    if (abs(peakGyroY) > GYROY_THRESHOLD && current.totalGyro > TOTAL_GYRO_IMPACT) {
      // Classify based on peak GyroY value
      if (peakGyroY > GYROY_THRESHOLD) {
        // Strongly positive GyroY = rim hit
        Serial.println(">>> RIM HIT DETECTED!");
        Serial.print("    Peak GyroY: "); Serial.print(peakGyroY, 1); 
        Serial.print("°/s, TotalGyro: "); Serial.print(current.totalGyro, 1);
        Serial.print("°/s, Duration: "); Serial.print(currentTime - peakTime); Serial.println("ms");
        triggerFlash("rim");
        lastImpactTime = currentTime;
      } else if (peakGyroY < -GYROY_THRESHOLD) {
        // Strongly negative GyroY = net hit
        Serial.println(">>> NET HIT DETECTED!");
        Serial.print("    Peak GyroY: "); Serial.print(peakGyroY, 1); 
        Serial.print("°/s, TotalGyro: "); Serial.print(current.totalGyro, 1);
        Serial.print("°/s, Duration: "); Serial.print(currentTime - peakTime); Serial.println("ms");
        triggerFlash("net");
        lastImpactTime = currentTime;
      }
    }
    
    // Reset peak detection
    peakDetected = false;
    peakGyroY = 0;
  }
  
  // Update last value for next iteration
  lastGyroY = current.gyroY;
}

//Rim Flash Sequence - 5 flashes from 80% to 5% over 2.5 seconds
void handleRimFlashSequence() {
  if (!rimFlashActive) {
    analogWrite(LED_STRIP_PIN, BRIGHTNESS_MID);
    return;
  }
  
  unsigned long rimFlashElapsed = millis() - rimFlashStartTime;
  
  // Check if rim flash sequence is complete
  if (rimFlashElapsed >= RIM_FLASH_TOTAL_TIME) {
    rimFlashActive = false;
    analogWrite(LED_STRIP_PIN, BRIGHTNESS_MID);
    return;
  }
  
  // Calculate which flash we're on (0-4)
  unsigned long flashPeriod = RIM_FLASH_TOTAL_TIME / RIM_FLASH_COUNT;
  currentRimFlash = rimFlashElapsed / flashPeriod;
  
  // Calculate brightness for current flash (80% to 5%)
  int targetBrightness = map(currentRimFlash, 0, RIM_FLASH_COUNT - 1, 
                            BRIGHTNESS_MID * 0.8, BRIGHTNESS_MID * 0.05);
  
  // Calculate timing within current flash period
  unsigned long timeInFlash = rimFlashElapsed % flashPeriod;
  
  if (timeInFlash < RIM_FLASH_ON_TIME) {
    // Flash ON
    analogWrite(LED_STRIP_PIN, targetBrightness);
  } else if (timeInFlash < RIM_FLASH_ON_TIME + RIM_FLASH_OFF_TIME) {
    // Flash OFF
    analogWrite(LED_STRIP_PIN, 0);
  } else {
    // Brief pause before next flash
    analogWrite(LED_STRIP_PIN, 0);
  }
}

//Flash Sequence - In Progress (for net hits)
void handleFlashSequence() {
  if (!flashActive) {
    analogWrite(LED_STRIP_PIN, BRIGHTNESS_MID);
    return;
  }
  
  unsigned long flashElapsed = millis() - flashStartTime;
  
  if (flashElapsed < DIM_TIME) {
    // Dim to LOW
    int brightness = map(flashElapsed, 0, DIM_TIME, BRIGHTNESS_MID, BRIGHTNESS_LOW);
    analogWrite(LED_STRIP_PIN, brightness);
    
  } else if (flashElapsed < DIM_TIME + DIM_HOLD) {
    // Hold at LOW
    analogWrite(LED_STRIP_PIN, BRIGHTNESS_LOW);
    
  } else if (flashElapsed < DIM_TIME + DIM_HOLD + FLASH_TIME) {
    // Flash to HIGH
    analogWrite(LED_STRIP_PIN, BRIGHTNESS_HIGH);
    
  } else if (flashElapsed < DIM_TIME + DIM_HOLD + FLASH_TIME + FADE_TIME) {
    // Fade back to MID
    unsigned long fadeElapsed = flashElapsed - (DIM_TIME + DIM_HOLD + FLASH_TIME);
    int brightness = map(fadeElapsed, 0, FADE_TIME, BRIGHTNESS_HIGH, BRIGHTNESS_MID);
    analogWrite(LED_STRIP_PIN, brightness);
    
  } else {
    // Flash complete
    flashActive = false;
    analogWrite(LED_STRIP_PIN, BRIGHTNESS_MID);
  }
}

//Helper functions for I2C communication
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 1, true);
  return Wire.read();
}

//Debugging
void printDebugInfo() {
  Serial.print("GyroY: ");
  Serial.print(current.gyroY, 1);
  Serial.print("°/s | GyroX: ");
  Serial.print(current.gyroX, 1);
  Serial.print("°/s | GyroZ: ");
  Serial.print(current.gyroZ, 1);
  Serial.print("°/s | TotalGyro: ");
  Serial.print(current.totalGyro, 1);
  Serial.print("°/s | Threshold: ");
  Serial.print(GYROY_THRESHOLD, 1);
  Serial.print("°/s");
  
  if (peakDetected) { // Changed from impactDetected to peakDetected
    Serial.print(" | IMPACT DETECTED");
  }
  
  Serial.println();
}
