#include <Arduino.h>
#include "FastIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <ctime>
#include <cstdlib>

// Default Gyro LPF 0x00
// Default Accel LPF 0x00

// Current Gyro LPF 0x02
// Current Accel LPF 0x04

// Baurdrate
#define BAUD_RATE 9600

#define HC12_RX 16  // UART2 RX pin
#define HC12_TX 17  // UART2 TX pin

#define IMU_ADDRESS 0x68     // MPU6500 I2C Address
#define PERFORM_CALIBRATION  // Comment to disable startup calibration
#define MAG_ADDRESS 0x0D     // QMC5883L I2C Address

// Register Addresses
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_GYRO_CONFIG_LPF 0x1A
#define MPU6500_ACCEL_CONFIG_LPF 0x1D
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C

#define INTERVAL_MS_PRINT 10  // Time in ms for printing data on Serial Monitor

/* PACKET STRUCTURE */
typedef struct message {
  float pitch = 0; // Pitch Value
  float roll = 0; // Roll Value
  float yaw = 0; // Yaw Value
  int16_t magX = 0; // Magnetometer X Value
  int16_t magY = 0; // Magnetometer Y Value
  int16_t magZ = 0; // Magnetometer Z Value
  int packetID; // Packet ID
} message_t;

/* Function Decleration */
void readQMC5883L(int16_t*, int16_t*, int16_t*);
float Calculate_PID(unsigned long);
void setGyroLPF(uint8_t);
void setAccelLPF(uint8_t);
void writeRegister(uint8_t, uint8_t);
uint8_t readRegister(uint8_t);
void calculateNormalizedAccelerometerValues();
void calculateAccelerometerAngles();
void calculateNormalizedGyroscopeValues();
void calculateGyroscopeAngles(unsigned long);
void Calculate_Roll();
void Calculate_Pitch();
bool readSample();
bool SendDataToGroundStation(message_t* data);

MPU6500 IMU;  // Change to the name of any supported IMU!

calData calib = { 0 };  // Calibration data
AccelData accelData;    // Sensor data
GyroData gyroData;
MagData magData;

const float GYRO_SENSITIVITY = 131.0;    // LSB/(°/s)
const float ACCEL_SENSITIVITY = 16384.0; // LSB/g 16384 for 2g
const float GRAVITY = 9.81;              // m/s² 
const float PITCH_TARGET_HEADING = 0.55;  
const float ACCEL_TARGET_HEADING = 0.91;
const float ALPHA  = 0.02;
unsigned long lastSampleMicros = 0;     // Save time for last reading captured
unsigned long lastPrintMillis = 0;

/* PARAMETERS */

// Normalized value variables
float norm_gyroX, norm_gyroY, norm_gyroZ;
float norm_accelX, norm_accelY, norm_accelZ;

// Magnetometer Values
int16_t magX, magY, magZ;

// Final value variables
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;

// Pitch and Roll
float pitch = 0, roll = 0, yaw = 0;

// PID Parameters
float Kp = 6.0 , Ki = 0.5, Kd = 0.1;

// PID Variables
float error = 0, prevError = 0, derivative = 0, integral = 0;
float deltaTime = 0;

/* FUNCTION DEFINITION */

// Calculate PID
float Calculate_PID(unsigned long sampleMicros) {
  double seconds = sampleMicros / 1000000.0;

  error = ACCEL_TARGET_HEADING - roll;
  //integral += error * seconds; (Integral Windup Issue)
  derivative = (error - prevError) / seconds;

  float correction = (Kp * error) + (Kd * derivative);

  prevError = error;

  return correction;
}

// Function to set Gyroscope LPF
void setGyroLPF(uint8_t freq) {
  uint8_t config;
  switch (freq) {
    case 250: config = 0x00; break;
    case 184: config = 0x01; break;
    case 92:  config = 0x02; break;
    case 41:  config = 0x03; break;
    case 20:  config = 0x04; break;
    case 10:  config = 0x05; break;
    case 5:   config = 0x06; break;
    default:  config = 0x02; // Default to 92 Hz
  }
  writeRegister(MPU6500_GYRO_CONFIG_LPF, config);
  Serial.print("Gyro LPF set to ");
  Serial.print(freq);
  Serial.println(" Hz");
}

// Function to set Accelerometer LPF
void setAccelLPF(uint8_t freq) {
  uint8_t config;
  switch (freq) {
    case 460: config = 0x00; break;
    case 184: config = 0x01; break;
    case 92:  config = 0x02; break;
    case 41:  config = 0x03; break;
    case 20:  config = 0x04; break;
    case 10:  config = 0x05; break;
    case 5:   config = 0x06; break;
    default:  config = 0x04; // Default to 20 Hz
  }
  writeRegister(MPU6500_ACCEL_CONFIG_LPF, config);
  Serial.print("Accel LPF set to ");
  Serial.print(freq);
  Serial.println(" Hz");
}

// Function to write to MPU6500 registers
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read MPU6500 register
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDRESS, 1);
  return Wire.read();
}

// Function to calculate Normalized values from Accelerometer
void calculateNormalizedAccelerometerValues() {
  norm_accelX = (accelData.accelX / ACCEL_SENSITIVITY) * GRAVITY;  
  norm_accelY = (accelData.accelY / ACCEL_SENSITIVITY) * GRAVITY;
  norm_accelZ = (accelData.accelZ / ACCEL_SENSITIVITY) * GRAVITY;
}

// Function to calculate Angles from Normalized Values
void calculateAccelerometerAngles() {
  accelX = atan(norm_accelY / sqrt(sq(norm_accelX) + sq(norm_accelZ)));
  accelY = atan(-1 * norm_accelX / sqrt(sq(norm_accelY) + sq(norm_accelZ)));
  accelZ = atan2(accelY, accelX);
}

void calculateNormalizedGyroscopeValues() {
  norm_gyroX = (gyroData.gyroX / GYRO_SENSITIVITY); // Angular Velocity (Normalized Form)
  norm_gyroY = (gyroData.gyroY / GYRO_SENSITIVITY);
  norm_gyroZ = (gyroData.gyroZ / GYRO_SENSITIVITY);
}

void calculateGyroscopeAngles(unsigned long sampleMicros) {
  gyroX = norm_gyroX * sampleMicros / 1000000;  // Angle Change = Angular Velocity * time(s)
  gyroY = norm_gyroY * sampleMicros / 1000000;
  gyroZ = norm_gyroZ * sampleMicros / 1000000;
}

void Calculate_Roll() {
  roll = (1 - ALPHA) * (roll + degrees(gyroY)) + ALPHA * degrees(accelY); 
}

void Calculate_Pitch() {
  pitch = (1 - ALPHA) * (pitch + degrees(gyroX)) + ALPHA * degrees(accelX);
}

// Function to calculate Yaw
void Calculate_Yaw() {
  yaw = (1 - ALPHA) * (yaw + degrees(gyroZ)) + ALPHA * degrees(accelZ); 
}

// Function to read data from QMC5883L
void readQMC5883L(int16_t* magX, int16_t* magY, int16_t* magZ) {
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(0x00);  // Starting register for reading data
  Wire.endTransmission();

  Wire.requestFrom(MAG_ADDRESS, 6);  // Request 6 bytes: X, Y, Z axes
  if (Wire.available() == 6) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    *magX = (int16_t)(msb << 8 | lsb);

    lsb = Wire.read();
    msb = Wire.read();
    *magY = (int16_t)(msb << 8 | lsb);

    lsb = Wire.read();
    msb = Wire.read();
    *magZ = (int16_t)(msb << 8 | lsb);
  }
}

// Function to send data to ground station via RF

bool SendDataToGroundStation(message_t* data) {
  const uint8_t START_MARKER[2] = { 0xAA, 0x55 };  // Start of packet marker

  // Check if Serial2 is ready
  size_t totalSize = sizeof(START_MARKER) + 3 * sizeof(float) + 3 * sizeof(int16_t) + sizeof(int);
  if (Serial2.availableForWrite() < totalSize) {
    Serial.println("HC-12 not ready for transmission.");
    return false;
  }

  // 1. Send start marker
  Serial2.write(START_MARKER, sizeof(START_MARKER));

  // 2. Send each field manually
  Serial2.write((uint8_t*)&data->pitch, sizeof(data->pitch));
  Serial2.write((uint8_t*)&data->roll, sizeof(data->roll));
  Serial2.write((uint8_t*)&data->yaw, sizeof(data->yaw));
  
  Serial2.write((uint8_t*)&data->magX, sizeof(data->magX));
  Serial2.write((uint8_t*)&data->magY, sizeof(data->magY));
  Serial2.write((uint8_t*)&data->magZ, sizeof(data->magZ));

  Serial2.write((uint8_t*)&data->packetID, sizeof(data->packetID));

  return true;
}


bool readSample() {
  unsigned long sampleMicros = (lastSampleMicros > 0) ? micros() - lastSampleMicros : 0;
  lastSampleMicros = micros();
  
  IMU.update();

  // Get accelerometer data
  IMU.getAccel(&accelData);

  // Get gyroscope data
  IMU.getGyro(&gyroData);

  // Get Magnetometer data
  readQMC5883L(&magX, &magY, &magZ);

  // Normalize Accelerometer Values
  calculateNormalizedAccelerometerValues();

  // Normalize Gyroscope Values
  calculateNormalizedGyroscopeValues();

  // Calculate Accelerometer Angles
  calculateAccelerometerAngles();

  // Calculate Gyroscope Angles
  calculateGyroscopeAngles(sampleMicros);

  // Calculate Roll
  Calculate_Roll();

  // Calculate Pitch
  Calculate_Pitch();

  // Calculate Yaw
  Calculate_Yaw();

  // Calculate PID
  float Correction = Calculate_PID(sampleMicros);

  /* Send Data to Ground Station */
  message_t packet; // Create a message object
  packet.pitch = pitch;
  packet.roll = roll;
  packet.yaw = yaw;
  packet.magX = magX;
  packet.magY = magY;
  packet.magZ = magZ;
  packet.packetID = rand() % 16777216; // Random Packet ID

  bool Sent = SendDataToGroundStation(&packet); // Send data to ground station 

  if(Sent) {
    Serial.println("Data sent successfully!");
  } else {
    Serial.println("Failed to send data.");
  }

  delay(10);  // To be replaced with millis() for better timing
  

  // Serial.print("Correction: ");
  // Serial.print(Correction);clc
  // Serial.println();

  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("\t Pitch: ");
  Serial.print(pitch);
  Serial.print("\t Roll: ");
  Serial.println(roll);
  
  return 0;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, HC12_RX, HC12_TX);
  Serial.println("Serial2 Initialized!");
  Wire.begin();

  delay(1000);
  Serial.println("Initializing MPU6500...");

  // Initialize MPU6500
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing MPU6500: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  Serial.println("MPU6500 initialized!");

  // Set Gyro Range
  int result = IMU.setGyroRange(250);
  if(result == -1) {
    Serial.print("\nError setting Gyro Range.");
    while(true);
  }

  Serial.println("Gyro Range Succesfully Set.");

  // Set Accel Range
  result = IMU.setAccelRange(2);
  if(result == -1) {
    Serial.print("\nError Setting Accelerometer Range.");
    while(true);
  }

  Serial.println("Accelerometer Range Successfully Set.");

  // Read Gyro Value
  Serial.print("Gyro Val (MPU6500_GYRO_CONFIG): ");
  Serial.println(readRegister(MPU6500_GYRO_CONFIG), HEX);

  // Read Accel Value
  Serial.print("Accel Val (MPU6500_ACCEL_CONFIG): ");
  Serial.println(readRegister(MPU6500_ACCEL_CONFIG), HEX);

  // Set Gyroscope LPF to 92 Hz
  setGyroLPF(184);  // 184 

  // Set Accelerometer LPF to 20 Hz
  setAccelLPF(184); // 184

  // Read and print LPF settings
  Serial.print("Gyro LPF (MPU6500_GYRO_CONFIG_LPF): ");
  Serial.println(readRegister(MPU6500_GYRO_CONFIG_LPF), HEX);

  Serial.print("Accel LPF (MPU6500_ACCEL_CONFIG_LPF): ");
  Serial.println(readRegister(MPU6500_ACCEL_CONFIG_LPF), HEX);

  #ifdef PERFORM_CALIBRATION
    // Perform calibration for MPU6500
    Serial.println("FastIMU calibration & data example");
    delay(5000);
    IMU.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
  #endif

  /* Initialise Magnetometer */
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(0x09);  // Control register
  Wire.write(0x0D);  // Continuous mode, 200Hz output, full-scale range (+/-8 Gauss)
  Wire.endTransmission();

  Serial.println("MPU6500 and QMC5883L initialized!");

  srand(time(NULL)); // Seed for random number generation

  delay(100); 

}

void loop() {

  readSample();

  delay(100);

  // if (HC12.available()) {
  //   Serial.write(HC12.read());
  // }
  // if (Serial.available()) {
  //   HC12.write(Serial.read());
  // }
}
// This code sets up a simple serial communication between the Arduino and the HC-12 module.