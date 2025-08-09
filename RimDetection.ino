/*
Spikeball Impact Detection System
Using MPU6500 6-Axis Gyroscope + Accelerometer
  
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

// Impact detection thresholds - Lateral (X/Y) focused for rim detection
const float RIM_LATERAL_THRESHOLD = 0.6;    // Any significant X/Y acceleration = rim hit
const float MIN_IMPACT_ACCEL = 0.5;         // Minimum total acceleration to register
const float NET_HIT_ACCEL_Z = 0.6;          // Z-axis threshold for net hits
const float NET_LATERAL_MAX = 0.6;          // Max lateral movement for pure net hits
const float VIBRATION_THRESHOLD_Z = 0.6;    // Sustained Z-axis vibration for net
const int IMPACT_DURATION_MIN = 10;         // Minimum impact duration (ms)
const int DEBOUNCE_TIME = 400;              // Prevent multiple triggers

// Flash timing
const unsigned long DIM_TIME = 50;      
const unsigned long DIM_HOLD = 30;       
const unsigned long FLASH_TIME = 20;    
const unsigned long FADE_TIME = 300;    

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
unsigned long impactStartTime = 0;
bool impactDetected = false;

// Variables for pattern analysis
struct ImpactPattern {
  float peakAccelZ;
  float peakLateral;
  float peakGyro;
  float sustainedShakeZ;
  unsigned long duration;
  bool isRimHit;
  bool isNetHit;
};

ImpactPattern currentPattern;
float accelZHistory[10] = {0}; // Rolling average for Z-axis shake detection
int historyIndex = 0;

// Calibration variables
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

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
  handleFlashSequence();
  
  /* Remove Comment for Debugging
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    printDebugInfo();
    lastPrint = millis();
  }
  */

  delay(2);
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
  
  // Configure accelerometer (±8g range)
  writeRegister(ACCEL_CONFIG, 0x10);
  
  // Configure gyroscope (±1000°/s range)
  writeRegister(GYRO_CONFIG, 0x10);
  
  // Set sample rate divider (1kHz sample rate)
  writeRegister(0x19, 0x07);
  
  // Configure digital low pass filter
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
  data.accelX = (accelX / 4096.0) - accelOffsetX;  // ±8g range
  data.accelY = (accelY / 4096.0) - accelOffsetY;
  data.accelZ = (accelZ / 4096.0) - accelOffsetZ;  // This should be primary detection axis
  
  data.gyroX = (gyroX / 32.8) - gyroOffsetX;  // ±1000°/s range
  data.gyroY = (gyroY / 32.8) - gyroOffsetY;
  data.gyroZ = (gyroZ / 32.8) - gyroOffsetZ;
  
  // Calculate total magnitudes
  data.totalAccel = sqrt(data.accelX*data.accelX + 
                        data.accelY*data.accelY + 
                        data.accelZ*data.accelZ);
  
  data.totalGyro = sqrt(data.gyroX*data.gyroX + 
                       data.gyroY*data.gyroY + 
                       data.gyroZ*data.gyroZ);
  
  // Calculate lateral (X-Y plane) acceleration for rim detection
  data.lateralAccel = sqrt(data.accelX*data.accelX + data.accelY*data.accelY);
}

void triggerFlash(String hitType = "unknown") {
  if (!flashActive) {
    flashActive = true;
    flashStartTime = millis();
    
    if (hitType == "rim") {
      Serial.println("Triggering RIM flash pattern");
    } else if (hitType == "net") {
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
  
  // Update rolling history for sustained Z-axis shake detection
  accelZHistory[historyIndex] = abs(current.accelZ);
  historyIndex = (historyIndex + 1) % 10;
  
  // Calculate sustained Z-axis shake (average of recent readings)
  float sustainedShakeZ = 0;
  for (int i = 0; i < 10; i++) {
    sustainedShakeZ += accelZHistory[i];
  }
  sustainedShakeZ /= 10.0;
  
  // Analyze impact patterns - Lateral movement indicates rim hits
  bool rimHit = false;
  bool netHit = false;
  
  // Must have minimum impact to register anything
  if (current.totalAccel < MIN_IMPACT_ACCEL) {
    impactDetected = false;
    return;
  }
  
  // RIM HIT SIGNATURE 
  // - ANY significant lateral (X/Y) movement indicates rim contact
  // - Ball bouncing off rim creates sideways forces
  if (current.lateralAccel > RIM_LATERAL_THRESHOLD) {
    rimHit = true;
  }
  
  // NET HIT SIGNATURE:
  // - Primarily Z-axis movement (ball striking net)
  // - Minimal lateral movement (no rim contact)
  // - Sustained vibration as net flexes
  else if ((abs(current.accelZ) > NET_HIT_ACCEL_Z) && 
           (current.lateralAccel <= NET_LATERAL_MAX) &&
           (sustainedShakeZ > VIBRATION_THRESHOLD_Z)) {
    netHit = true;
  }
  
  // Trigger detection logic
  if (rimHit || netHit) {
    if (!impactDetected) {
      impactStartTime = currentTime;
      impactDetected = true;
      currentPattern.peakAccelZ = abs(current.accelZ);
      currentPattern.peakLateral = current.lateralAccel;
      currentPattern.peakGyro = current.totalGyro;
      currentPattern.sustainedShakeZ = sustainedShakeZ;
    }
    
    // Check if impact has lasted long enough
    if ((currentTime - impactStartTime) >= IMPACT_DURATION_MIN) {
      lastImpactTime = currentTime;
      currentPattern.duration = (currentTime - impactStartTime);
      impactDetected = false;
      
      if (rimHit) {
        Serial.println(">>> RIM HIT DETECTED!");
        Serial.print("    Lateral: "); Serial.print(currentPattern.peakLateral, 2); 
        Serial.print("g, Z-Accel: "); Serial.print(currentPattern.peakAccelZ, 2); Serial.println("g");
        triggerFlash("rim");
      } else if (netHit) {
        Serial.println(">>> NET HIT DETECTED!");
        Serial.print("    Z-Peak: "); Serial.print(currentPattern.peakAccelZ, 2); 
        Serial.print("g, Lateral: "); Serial.print(currentPattern.peakLateral, 2);
        Serial.print("g, Z-Sustained: "); Serial.print(currentPattern.sustainedShakeZ, 2); Serial.println("g");
        triggerFlash("net");
      }
    }
  } else {
    impactDetected = false;
  }
}

//Flash Sequence - In Progress
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
  Serial.print("Z-Accel: ");
  Serial.print(current.accelZ, 2);
  Serial.print("g | Lateral: ");
  Serial.print(current.lateralAccel, 2);
  Serial.print("g | Gyro: ");
  Serial.print(current.totalGyro, 1);
  Serial.print("°/s");
  
  if (impactDetected) {
    Serial.print(" | IMPACT DETECTED");
  }
  
  Serial.println();
}