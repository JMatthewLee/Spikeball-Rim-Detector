/*
Spikeball Impact Detection - Serial Data Collection Script
Using MPU6500 6-Axis Gyroscope + Accelerometer
Author: Matthew Lee (Optimized for Serial Output)

This script collects data via serial monitor in CSV format for analysis.
Output can be copied to a spreadsheet for further analysis.

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

DATA OUTPUT: CSV format via Serial Monitor
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
const int STATUS_LED = 13;




// Data collection settings
const int SAMPLE_RATE = 100;

// Impact detection thresholds
const float MIN_IMPACT_ACCEL = 1.5;  // Must be above normal gravity (1g) + some margin
const int DEBOUNCE_TIME = 2000;  // 2 seconds between hits
// Gyroscope-based impact thresholds (from analysis: GyroY is highly discriminative)
const float MIN_IMPACT_GYROY = 50.0;     // degrees/second on Y axis
const float MIN_IMPACT_TOTALGYRO = 90.0; // overall rotational speed threshold

// Enhanced data structures with gyroscope
struct SensorData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float totalAccel;
  float lateralAccel;
  float totalGyro;
};



// Global variables
SensorData current;
unsigned long lastSampleTime = 0;
bool impactDetected = false;
unsigned long lastImpactTime = 0;
unsigned long impactStartTime = 0;
int dataPointsSent = 0;

// Calibration variables
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;



void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(STATUS_LED, OUTPUT);

  
     digitalWrite(STATUS_LED, HIGH);
  
  Serial.println("Spikeball Impact Detection - Serial Data Collection");
  Serial.println("==================================================");
  Serial.println();
  
  // Initialize MPU6500
  if (initMPU6500()) {
    Serial.println("MPU6500 initialized successfully.");
    Serial.println("Calibrating accelerometer and gyroscope... Keep still.");
    calibrateSensor();
    Serial.println("Calibration complete.");
    
    
    Serial.println();
    
    // Print menu
    printMenu();
    
  } else {
    Serial.println("Failed to initialize MPU6500!");
    while(1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
  }
}

void loop() {
  // Read sensor data at specified rate
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
    lastSampleTime = currentTime;
    readSensorData(current);
    
    // Check for impact and plot data only during impacts
    checkForImpact();
  }
  
  delay(1);
}



void checkForImpact() {
  // Primary trigger: gyroscope-based (GyroY or TotalGyro)
  bool gyroImpact = (fabs(current.gyroY) > MIN_IMPACT_GYROY) || (current.totalGyro > MIN_IMPACT_TOTALGYRO);
  // Optional additional guard: require some acceleration to avoid pure drift noise
  bool accelGuard = current.totalAccel > (MIN_IMPACT_ACCEL * 0.8);

  if (!impactDetected && (gyroImpact || (current.totalAccel > MIN_IMPACT_ACCEL))) {
    if (!gyroImpact && !accelGuard) {
      // If only weak accel without gyro, ignore
      return;
    }
    unsigned long currentTime = millis();
    if (currentTime - lastImpactTime > DEBOUNCE_TIME) {
      impactDetected = true;
      lastImpactTime = currentTime;
      impactStartTime = currentTime;
      dataPointsSent = 0;
      
      // Start plotting data for this impact
      Serial.println("--- IMPACT START ---");
    }
  }
  
  // Only output data if we're in an impact and haven't sent 5 points yet
  if (impactDetected && dataPointsSent < 5) {
    // Output data in CSV format for easy analysis
          Serial.print(dataPointsSent + 1);
      Serial.print(",");
      Serial.print(current.accelX, 3);
      Serial.print(",");
      Serial.print(current.accelY, 3);
      Serial.print(",");
      Serial.print(current.accelZ, 3);
      Serial.print(",");
      Serial.print(current.gyroX, 3);
      Serial.print(",");
      Serial.print(current.gyroY, 3);
      Serial.print(",");
      Serial.print(current.gyroZ, 3);
      Serial.print(",");
      Serial.print(current.totalAccel, 3);
      Serial.print(",");
      Serial.print(current.lateralAccel, 3);
      Serial.print(",");
      Serial.print(current.totalGyro, 3);
      Serial.println();
    
    dataPointsSent++;
    
    // End impact after sending 5 data points
    if (dataPointsSent >= 5) {
      impactDetected = false;
      Serial.println("--- IMPACT END ---");
    }
  }
}



void printMenu() {
  Serial.println("Spikeball Impact Detection - CSV Data Collection");
  Serial.println("===============================================");
  Serial.println();
  Serial.println("Data is output in CSV format for easy spreadsheet analysis");
  Serial.println("Open Serial Monitor at 115200 baud to see impact data");
  Serial.println();
  Serial.println("CSV format: Point,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,TotalAccel,LateralAccel,TotalGyro");
  Serial.println();
  Serial.println("Impacts trigger when GyroY > 50°/s or TotalGyro > 90°/s");
  Serial.println("Acceleration guard ~1.2g helps filter noise");
  Serial.println("Each impact sends exactly 5 data points");
  Serial.println();
  Serial.println("Copy CSV data and paste into Excel/Google Sheets for analysis");
  Serial.println();
}





// MPU6500 functions
bool initMPU6500() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;
  
  delay(100);
  
  // Configure accelerometer (±2g)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  
  // Configure gyroscope (±250°/s)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  

  
  // Configure low pass filter
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x06);
  if (Wire.endTransmission() != 0) return false;
  
  return true;
}

void calibrateSensor() {
  const int samples = 100;
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  
  for (int i = 0; i < samples; i++) {
    readSensorData(current);
    sumAX += current.accelX;
    sumAY += current.accelY;
    sumAZ += current.accelZ;
    sumGX += current.gyroX;
    sumGY += current.gyroY;
    sumGZ += current.gyroZ;
    delay(10);
  }
  
  accelOffsetX = sumAX / samples;
  accelOffsetY = sumAY / samples;
  accelOffsetZ = sumAZ / samples;
  gyroOffsetX = sumGX / samples;
  gyroOffsetY = sumGY / samples;
  gyroOffsetZ = sumGZ / samples;
}

void readSensorData(SensorData &data) {
  // Read accelerometer data
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 6, true);
  
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();
  
  // Read gyroscope data
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 6, true);
  
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert to physical units
  data.accelX = (accelX / 16384.0) - accelOffsetX;
  data.accelY = (accelY / 16384.0) - accelOffsetY;
  data.accelZ = (accelZ / 16384.0) - accelOffsetZ;
  
  data.gyroX = (gyroX / 131.0) - gyroOffsetX;  // ±250°/s range
  data.gyroY = (gyroY / 131.0) - gyroOffsetY;
  data.gyroZ = (gyroZ / 131.0) - gyroOffsetZ;
  
  // Calculate derived values
  data.totalAccel = sqrt(data.accelX * data.accelX + data.accelY * data.accelY + data.accelZ * data.accelZ);
  data.lateralAccel = sqrt(data.accelX * data.accelX + data.accelY * data.accelY);
  data.totalGyro = sqrt(data.gyroX * data.gyroX + data.gyroY * data.gyroY + data.gyroZ * data.gyroZ);
}
