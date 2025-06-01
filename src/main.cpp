#include <Arduino.h>
#include "FastIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <ctime>
#include <cstdlib>
#include <mbedtls/aes.h>
#include <SX127XLT.h>
#include <SPI.h>
#include <common/mavlink.h>
#include <TinyGPS++.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/ecdsa.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>

// Undefine conflicting macros from SX1278 before including BMP280
#ifdef MODE_SLEEP
#undef MODE_SLEEP
#endif
#ifdef MODE_STDBY
#undef MODE_STDBY
#endif
#ifdef MODE_TX
#undef MODE_TX
#endif
#ifdef MODE_RX
#undef MODE_RX
#endif

#include <Adafruit_BMP280.h>

/* SX1278 CONFIG SELECTIONS */
#define NSS     5
#define NRESET  4
#define DIO0    2

#define FREQUENCY                 433000000
#define OFFSET                    0
#define BANDWIDTH                 LORA_BW_125 // 125kHz
#define SPREADING_FACTOR          LORA_SF7
#define CODING_RATE               LORA_CR_4_5  // 4/5
#define HEADERMODE                0x00 // Explicit Header Modes
#define PA_OUTPUT_PA_BOOST_PIN    5
#define RAMP_TIME                 0x02
#define TX_POWER                  14
// Default Sync word 0x12 (Can be changed) (Transmitter and Receiver must have same sync word)
#define LORA_SYNCWORD            LORA_MAC_PRIVATE_SYNCWORD // Private LoRa sync word

/* GPS MODULE PIN DEFINITIONS */
#define GPS_TX 16  // UART2 RX pin
#define GPS_RX 17  // UART2 TX pin
 
// Default Gyro LPF 0x00
// Default Accel LPF 0x00

// Current Gyro LPF 0x02
// Current Accel LPF 0x04

/* SX1278 Instance */
SX127XLT LT;

/* GPS Module Instance */
TinyGPSPlus gps;

/* BMP280 Instance */
Adafruit_BMP280 bmp;

// --- ECC/ECDSA Contexts and Buffers ---
mbedtls_ecdh_context ecdh;
mbedtls_ecdsa_context ecdsa; // Our ECDSA keypair
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_entropy_context entropy;

// Baurdrate
#define BAUD_RATE 9600

// Define Device
#define LORA_DEVICE DEVICE_SX1278

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
/* Compiler uses padding to make the structure a multiple of 4 (Total Size: 24 Bytes) */
typedef struct {
  float pitch;    // 4 bytes
  float roll;     // 4 bytes
  float yaw;      // 4 bytes
  int16_t magX;   // 2 bytes
  int16_t magY;   // 2 bytes
  int16_t magZ;   // 2 bytes
  int packetID;   // 4 bytes
} message_t;      // Total: 22 + 2 bytes


/* Function Decleration */
void updateBaro();
void updateGPS();
bool EncryptBuffer(const uint8_t*, size_t, uint8_t*, uint8_t*, size_t );
bool Decrypt(uint8_t*, uint8_t* , message_t*);
void printHex(uint8_t*, size_t);
void sample_temp();
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
bool SendDataToGroundStation(message_t*);
bool SendDataToGroundStation(uint8_t*);
bool SendDataToGroundStation_LoRa(uint8_t*, size_t);
void sendTelemetryMavlink();
void receiveTelemetryMavlink();

MPU6500 IMU;  

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

// Key
const char aes_key[] = "!@#$%^&*()!@#$%^"; // 16 bytes key

// Barometer Variables
float baro_pressure = 0, baro_temp = 0, baro_alt = 0;

uint8_t shared_secret[32]; // 256-bit shared secret
uint8_t aes_key_buf[16];   // AES-128 key (first 16 bytes of shared_secret)
bool handshake_done = false;

// Transmitter's static private key (32 bytes)
const uint8_t my_ecdsa_privkey[32] = {
    0xcc, 0x79, 0xee, 0xf4, 0x97, 0xbb, 0x49, 0x7d, 0xf0, 0xd8, 0x49, 0xd4, 0xae, 0x9d, 0x7d, 0xd1,
    0xbd, 0xde, 0xe7, 0x48, 0x91, 0xa3, 0x04, 0x8d, 0xae, 0xd2, 0xc1, 0xe1, 0xb3, 0xf9, 0x35, 0xb3
};
// Transmitter's static public key (65 bytes, uncompressed)
const uint8_t my_ecdsa_pubkey[65] = {
    0x04, 0x25, 0xc9, 0x2e, 0x3f, 0xdd, 0x2b, 0x6b, 0x78, 0x10, 0xeb, 0xdb, 0xde, 0x71, 0x97,
    0x95, 0xe8, 0x13, 0xf4, 0xc2, 0xef, 0x45, 0x74, 0x2e, 0xb5, 0x42, 0x86, 0x30, 0xbe, 0x4a,
    0x39, 0xfa, 0x80, 0xd4, 0x74, 0xd6, 0x99, 0xd3, 0xe1, 0x94, 0xe8, 0x0a, 0x50, 0x6c, 0x40,
    0xe4, 0xd0, 0x31, 0xc1, 0xb9, 0xf6, 0x26, 0x69, 0x05, 0x89, 0x70, 0x39, 0x05, 0x47, 0xcc,
    0x7f, 0x8a, 0xfb, 0x64, 0x2d
};
// Receiver's static public key (peer)
const uint8_t peer_ecdsa_pubkey[65] = {
    0x04, 0xd0, 0x4d, 0xef, 0x42, 0x2c, 0xf2, 0xd4, 0x80, 0x33, 0x9d, 0x43, 0x61, 0xcb, 0x93,
    0x21, 0x3f, 0x01, 0xed, 0xba, 0x90, 0xe9, 0xbb, 0x94, 0x3b, 0xe9, 0xbe, 0x75, 0xa8, 0x09,
    0xa3, 0xe5, 0x26, 0x47, 0x5a, 0x99, 0xda, 0x61, 0x81, 0x3f, 0x35, 0x2e, 0xad, 0x93, 0x89,
    0xec, 0x3c, 0xc8, 0x5d, 0x1e, 0x1f, 0x6b, 0x86, 0xaf, 0x10, 0x8c, 0xdc, 0x0d, 0x2e, 0xf0,
    0xf0, 0x81, 0xcf, 0xf6, 0x46
};

/* FUNCTION DEFINITION */
void sample_temp() {
  const char aes_key[] = "!@#$%^&*()!@#$%^"; // 16 bytes key
  message_t msg;
  msg.pitch = 1.1;
  msg.roll = 2.2;
  msg.yaw = 3.3;
  msg.magX = 4;
  msg.magY = 5;
  msg.magZ = 6;
  msg.packetID = 100; // Random Packet ID
  
  // Buffer must be large enough for padded data (Multiple of 16 bytes)
  // AES block size is 16 bytes, so we need to pad to the next multiple of 16 
  uint8_t packet[32];
  
  Serial.print("Packet Size: ");
  Serial.print("Msg Size (without Addition): ");
  Serial.println(sizeof(message_t));   // 24 bytes padding included

  // Check available bytes for writing
  Serial.print("Available Bytes: ");
  Serial.println(Serial2.availableForWrite()); // 128 Bytes

  Serial.println("Original Data:");
  Serial.print("Roll: "); Serial.println(msg.roll);
  Serial.print("Yaw: "); Serial.println(msg.yaw);
  Serial.print("Pitch: "); Serial.println(msg.pitch);
  Serial.print("MagX: "); Serial.println(msg.magX);
  Serial.print("MagY: "); Serial.println(msg.magY);
  Serial.print("MagZ: "); Serial.println(msg.magZ);
  Serial.print("PacketID: "); Serial.println(msg.packetID);
  
  //Encrypt(&msg, aes_key_buf, packet);
  
  Serial.print("Encrypted data (hex): ");
  printHex(packet, 32);
  
  message_t decryptedData;
  Decrypt(packet, aes_key_buf, &decryptedData);
  
  Serial.println("Decrypted Data:");
  Serial.print("Roll: "); Serial.println(decryptedData.roll);
  Serial.print("Yaw: "); Serial.println(decryptedData.yaw);
  Serial.print("Pitch: "); Serial.println(decryptedData.pitch);
  Serial.print("MagX: "); Serial.println(decryptedData.magX);
  Serial.print("MagY: "); Serial.println(decryptedData.magY);
  Serial.print("MagZ: "); Serial.println(decryptedData.magZ);
  Serial.print("PacketID: "); Serial.println(decryptedData.packetID);
  Serial.println("------------------");

  delay(1000); // Delay for readability
}
// Function to encrypt data using AES
// Use CBC mode for better security (requires IV)
/*
bool Encrypt(message_t* data, const char* key, uint8_t* packet) {
  mbedtls_aes_context aes;
  uint8_t iv[16] = {0}; // For testing - use random IV in production!
  
  // Pad the message to 32 bytes (next multiple of 16)
  uint8_t padded[32];
  memset(padded, 0, 32);
  memcpy(padded, data, sizeof(message_t));
  
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, (const unsigned char*)key, 128);
  mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, 32, iv, padded, packet);
  mbedtls_aes_free(&aes);

  return true;
}
*/

// Update Barometer

void updateBaro() {
    baro_temp = bmp.readTemperature(); // °C
    baro_pressure = bmp.readPressure() / 100.0F; // hPa
    baro_alt = bmp.readAltitude(1013.25); // meters, sea level pressure as reference
    //Serial.print("Baro Temp: "); Serial.print(baro_temp); Serial.print(" °C, ");
    //Serial.print("Pressure: "); Serial.print(baro_pressure); Serial.print(" hPa, ");
    //Serial.print("Altitude: "); Serial.print(baro_alt); Serial.println(" m");
}

// Update GPS
void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // Print GPS data if valid
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 7);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 7);
  } else {
    Serial.println("Location not valid");
  }

  if (gps.altitude.isValid()) {
    Serial.print("Altitude (m): ");
    Serial.println(gps.altitude.meters());
  } else {
    Serial.println("Altitude not valid");
  }

  if (gps.speed.isValid()) {
    Serial.print("Speed (m/s): ");
    Serial.println(gps.speed.mps());
  } else {
    Serial.println("Speed not valid");
  }

  Serial.println("------------------");
}

// Encrypts any buffer
bool EncryptBuffer(const uint8_t* data, size_t dataLen, uint8_t* key, uint8_t* encrypted, size_t paddedLen) {
    mbedtls_aes_context aes;
    uint8_t iv[16] = {0};
    uint8_t padded[256] = {0};
    memcpy(padded, data, dataLen);

    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, (const unsigned char*)key, 128);
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, paddedLen, iv, padded, encrypted);
    mbedtls_aes_free(&aes);

    return true;
}


bool Decrypt(uint8_t* packet, uint8_t* key, message_t* data) {
  mbedtls_aes_context aes;
  uint8_t iv[16] = {0}; // Must match encryption IV
  
  uint8_t decrypted[32];
  
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes, (const unsigned char*)key, 128);
  mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, 32, iv, packet, decrypted);
  mbedtls_aes_free(&aes);
  
  memcpy(data, decrypted, sizeof(message_t));
  return true;
}

void printHex(uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}



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

bool SendDataToGroundStation(uint8_t* data) {
  const uint8_t START_MARKER[2] = {0xAA, 0x55};

  const size_t totalSize = sizeof(START_MARKER) + sizeof(data); // 2 + 32 = 34 bytes

  // Wait for buffer to be available
  while(Serial2.availableForWrite() < totalSize) {
    Serial.println("HC-12 not ready for transmission.");
  }

  // 1. Send start marker
  Serial2.write(START_MARKER, sizeof(START_MARKER));

  // 2. Send data
  Serial2.write(data, sizeof(data)); // Send the data

  return true;
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

  if(!handshake_done) {
    static bool warned = false;
    if(!warned) {
      Serial.println("Handshake not done yet. Waiting for handshake...");
      warned = true;
    }
    return false; // Handshake not done, skip reading sample
  }

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

  // Get GPS Data
  updateGPS();

  // Get Barometer Data
  updateBaro();

  // Calculate PID
  float Correction = Calculate_PID(sampleMicros);

  /* Send Data to Ground Station */
  /*
  message_t packet; // Create a message object
  packet.pitch = pitch;
  packet.roll = roll;
  packet.yaw = yaw;
  packet.magX = magX;
  packet.magY = magY;
  packet.magZ = magZ;
  packet.packetID = rand() % 16777216; // Random Packet ID
  */

  /* Send MAVLink Data */
  sendTelemetryMavlink();

  // Encrypt data before sending
  //uint8_t encryptedPacket[32];
  //Encrypt(&packet, aes_key, encryptedPacket);
  
  // Send data to ground station
  //bool Sent = SendDataToGroundStation_LoRa(encryptedPacket, sizeof(encryptedPacket)); 
  
  /*
  if(Sent) {
    Serial.println("Data sent successfully!");
  } else {
    Serial.println("Failed to send data.");
  } */
  
  //delay(10);  // To be replaced with millis() for better timing
  

  // Serial.print("Correction: ");
  // Serial.print(Correction);clc
  // Serial.println();

  Serial.println("ORIENTATION");
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("\t Pitch: ");
  Serial.print(pitch);
  Serial.print("\t Roll: ");
  Serial.println(roll);
  Serial.println("BAROMETER DATA");
  Serial.print("\t Temperature: ");
  Serial.print(baro_temp);
  Serial.print("\t Pressure: ");
  Serial.print(baro_pressure);
  Serial.print("\t Altitude: ");
  Serial.println(baro_alt);

  //printHex(encryptedPacket, sizeof(encryptedPacket)); // Print encrypted data in hex format
  return 0;
}

// Function to send encrypted telemetry via LoRa
bool SendDataToGroundStation_LoRa(uint8_t* data, size_t len) {
  // txtimeout: 5000ms, txPower: 14dBm, wait: 1 (blocking)
  uint8_t result = LT.transmit(data, len, 1000, TX_POWER, WAIT_TX);
  if(result == 0) {
    Serial.println("LoRa transmission failed!");
    Serial.print("Error Returned: ");
    Serial.println(result);
    return false;
  }
  Serial.println("LoRa transmission successful!");
  return true;
}

void sendTelemetryMavlink() {
    mavlink_message_t msg;
    uint8_t buf[255];
    uint16_t len = 0;

    // 1. Pack Attitude (Roll, Pitch, Yaw)
    mavlink_msg_attitude_pack(
        1, 200, &msg, millis(),
        roll, pitch, yaw, 
        gyroX, gyroY, gyroZ
    );
    len += mavlink_msg_to_send_buffer(buf + len, &msg);

    // 2. Pack SCALED_IMU (Accel, Gyro, Mag)
    mavlink_msg_scaled_imu_pack(
        1, 200, &msg, millis(),
        norm_accelX, norm_accelY, norm_accelZ,
        norm_gyroX, norm_gyroY, norm_gyroZ,
        magX, magY, magZ, 0 // No Temperature
    );

    len += mavlink_msg_to_send_buffer(buf + len, &msg);
    
    // 3. Send Barometer Data
    mavlink_msg_scaled_pressure_pack(
      1, 200, &msg, millis(),
      baro_pressure, // Absolute pressure (hPa)
      0,          // Differential Pressure diff
      baro_temp * 100, // Absolute Temperature (cdegC)
      0           // Differential Pressure Temperature
    );
    
    len += mavlink_msg_to_send_buffer(buf + len, &msg);
    // 4. Pack GPS Data (if available)
    if(gps.location.isValid()) {
        mavlink_msg_gps_raw_int_pack(
            1, 200, &msg, millis(), 3, 
            gps.location.lat() * 1E7, // Latitude in degrees * 1E7
            gps.location.lng() * 1E7, // Longitude in degrees * 1E7
            gps.altitude.meters() * 1000, // Altitude in mm
            gps.hdop.isValid() ? gps.hdop.value() * 100 : UINT16_MAX, // eph in cm
            UINT16_MAX, // Vertical Dilution of Precision (not used)
            gps.speed.isValid() ? gps.speed.mps() * 100 : 0, // vel in cm/s
            gps.course.isValid() ? gps.course.deg() * 100 : 0, // cog in cdeg
            gps.satellites.isValid() ? gps.satellites.value() : 0,
            0, // Altitude above ellipsoid (not used)
            0, // h_acc
            0, // v_acc
            0, // vel_acc
            0,  // hdg_acc
            0 // Yaw
        );
        len += mavlink_msg_to_send_buffer(buf + len, &msg);
    }
    // 3. (Optional) Add more MAVLink messages here, e.g. GPS, battery, etc.

    // Send barometer Altitude with GPS data

    // 4. Calculate padded length for AES-128
    size_t paddedLen = ((len + 15) / 16) * 16;
    if (paddedLen > 255) {
        Serial.print("Error: Encryption MAVLink packet too large for LoRa to send! ");
        Serial.println(paddedLen);
        return;
    }

    // 5. Dynamically allocate buffer for encryption
    uint8_t* encryptedBuf = new uint8_t[paddedLen];
    EncryptBuffer(buf, len, aes_key_buf, encryptedBuf, paddedLen);

    // 6. Send encrypted data via LoRa
    uint8_t result = LT.transmit(encryptedBuf, paddedLen, 1000, TX_POWER, WAIT_TX);
    if(result == 0) {
        Serial.println("LoRa transmission failed!");
        Serial.print("Error Returned: ");
        Serial.println(result);
        delete[] encryptedBuf;
        return;
    } else {
        Serial.println("LoRa transmission for ALL MAVLink successful!");
        Serial.print("Padded Length: ");
        Serial.println(paddedLen);
        Serial.print("Encrypted Data (hex): ");
        printHex(encryptedBuf, paddedLen);
    }

    delete[] encryptedBuf;
}

/* RECEIVE TELEMETRY MAVLINK */
void receiveTelemetryMavlink() {
    uint8_t rxBuffer[255];
    int16_t length = LT.receive(rxBuffer, sizeof(rxBuffer), 5000, WAIT_RX);

    if (length > 0) {
        Serial.print("Received packet, length: ");
        Serial.println(length);

        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < length; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rxBuffer[i], &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        Serial.println("MAVLink ATTITUDE:");
                        Serial.print("  Roll: "); Serial.println(attitude.roll);
                        Serial.print("  Pitch: "); Serial.println(attitude.pitch);
                        Serial.print("  Yaw: "); Serial.println(attitude.yaw);
                        Serial.print("  GyroX: "); Serial.println(attitude.rollspeed);
                        Serial.print("  GyroY: "); Serial.println(attitude.pitchspeed);
                        Serial.print("  GyroZ: "); Serial.println(attitude.yawspeed);
                        break;
                    }
                    case MAVLINK_MSG_ID_SCALED_IMU: {
                        mavlink_scaled_imu_t imu;
                        mavlink_msg_scaled_imu_decode(&msg, &imu);
                        Serial.println("MAVLink SCALED_IMU:");
                        Serial.print("  AccelX: "); Serial.println(imu.xacc);
                        Serial.print("  AccelY: "); Serial.println(imu.yacc);
                        Serial.print("  AccelZ: "); Serial.println(imu.zacc);
                        Serial.print("  GyroX: "); Serial.println(imu.xgyro);
                        Serial.print("  GyroY: "); Serial.println(imu.ygyro);
                        Serial.print("  GyroZ: "); Serial.println(imu.zgyro);
                        Serial.print("  MagX: "); Serial.println(imu.xmag);
                        Serial.print("  MagY: "); Serial.println(imu.ymag);
                        Serial.print("  MagZ: "); Serial.println(imu.zmag);
                        break;
                    }
                    // Add more cases for other MAVLink messages as needed
                    default:
                        Serial.print("Received MAVLink msgid: ");
                        Serial.println(msg.msgid);
                        break;
                }
            }
        }
    } else {
        Serial.println("No MAVLink packet received (timeout).");
    }
}

// --- ECC/ECDSA Initialization ---
void init_crypto() {
    mbedtls_ecdh_init(&ecdh);
    mbedtls_ecdsa_init(&ecdsa);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, NULL, 0);

    // ECDH: secp256r1
    mbedtls_ecp_group_load(&ecdh.grp, MBEDTLS_ECP_DP_SECP256R1);
    mbedtls_ecdh_gen_public(&ecdh.grp, &ecdh.d, &ecdh.Q, mbedtls_ctr_drbg_random, &ctr_drbg);

    // ECDSA: secp256r1 (load static keypair)
    mbedtls_ecp_group_load(&ecdsa.grp, MBEDTLS_ECP_DP_SECP256R1);
    mbedtls_mpi_read_binary(&ecdsa.d, my_ecdsa_privkey, 32);
    mbedtls_ecp_point_read_binary(&ecdsa.grp, &ecdsa.Q, my_ecdsa_pubkey, 65);
}

// --- Export ECDH public key ---
size_t export_ecdh_pubkey(uint8_t* buf, size_t buf_len) {
    size_t olen = 0;
    mbedtls_ecp_point_write_binary(&ecdh.grp, &ecdh.Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, buf, buf_len);
    return olen;
}

// --- Sign ECDH public key with ECDSA ---
size_t sign_pubkey(const uint8_t* pubkey, size_t pubkey_len, uint8_t* sig, size_t sig_len) {
    size_t olen = 0;
    mbedtls_md_type_t md_alg = MBEDTLS_MD_SHA256;
    mbedtls_ecdsa_write_signature(&ecdsa, md_alg, pubkey, pubkey_len, sig, &olen, mbedtls_ctr_drbg_random, &ctr_drbg);
    return olen;
}

// --- Verify peer's signature ---
bool verify_peer_signature(const uint8_t* pubkey, size_t pubkey_len, const uint8_t* sig, size_t sig_len) {
    mbedtls_ecdsa_context peer_ctx;
    mbedtls_ecdsa_init(&peer_ctx);
    mbedtls_ecp_group_load(&peer_ctx.grp, MBEDTLS_ECP_DP_SECP256R1);
    mbedtls_ecp_point_read_binary(&peer_ctx.grp, &peer_ctx.Q, peer_ecdsa_pubkey, 65);
    int ret = mbedtls_ecdsa_read_signature(&peer_ctx, pubkey, pubkey_len, sig, sig_len);
    mbedtls_ecdsa_free(&peer_ctx);
    return ret == 0;
}

// --- Import peer's ECDH public key ---
bool import_peer_pubkey(const uint8_t* buf, size_t buf_len) {
    return mbedtls_ecp_point_read_binary(&ecdh.grp, &ecdh.Qp, buf, buf_len) == 0;
}

// --- Compute shared secret ---
bool compute_shared_secret() {
    if (mbedtls_ecdh_compute_shared(&ecdh.grp, &ecdh.z, &ecdh.Qp, &ecdh.d, mbedtls_ctr_drbg_random, &ctr_drbg) != 0)
        return false;
    mbedtls_mpi_write_binary(&ecdh.z, shared_secret, 32);
    memcpy(aes_key_buf, shared_secret, 16);
    handshake_done = true;
    return true;
}

// --- Send our ECDH public key and signature via LoRa ---
void send_handshake_packet() {
    uint8_t pubkey[65], sig[72];
    size_t pubkey_len = export_ecdh_pubkey(pubkey, sizeof(pubkey));
    size_t sig_len = sign_pubkey(pubkey, pubkey_len, sig, sizeof(sig));

    // Send: [pubkey_len][pubkey][sig_len][sig]
    uint8_t packet[1 + 65 + 1 + 72];
    packet[0] = pubkey_len;
    memcpy(packet + 1, pubkey, pubkey_len);
    packet[1 + pubkey_len] = sig_len;
    memcpy(packet + 1 + pubkey_len + 1, sig, sig_len);

    LT.transmit(packet, 1 + pubkey_len + 1 + sig_len, 1000, TX_POWER, WAIT_TX);
    Serial.println("Handshake packet sent via LoRa.");
}

// --- Receive peer's ECDH public key and signature via LoRa ---
bool receive_handshake_packet() {
    uint8_t rx[1 + 65 + 1 + 72];
    int16_t len = LT.receive(rx, sizeof(rx), 5000, WAIT_RX);
    if (len < 3) return false;
    size_t pubkey_len = rx[0];
    const uint8_t* pubkey = rx + 1;
    size_t sig_len = rx[1 + pubkey_len];
    const uint8_t* sig = rx + 1 + pubkey_len + 1;

    if (!verify_peer_signature(pubkey, pubkey_len, sig, sig_len)) {
        Serial.println("Peer signature verification failed!");
        return false;
    }
    if (!import_peer_pubkey(pubkey, pubkey_len)) {
        Serial.println("Peer ECDH public key import failed!");
        return false;
    }
    Serial.println("Peer handshake packet verified and imported.");
    return true;
}

// --- Perform handshake (call in setup) ---
void perform_handshake() {
    init_crypto();
    delay(100);
    send_handshake_packet();
    delay(100);
    if (receive_handshake_packet()) {
        if (compute_shared_secret()) {
            Serial.println("Handshake complete. AES key derived:");
            printHex(aes_key_buf, 16);
        } else {
            Serial.println("Shared secret computation failed.");
        }
    } else {
        Serial.println("Handshake failed.");
    }
}

void print_ecdsa_pubkey() {
    uint8_t buf[65];
    size_t olen = 0;
    mbedtls_ecp_point_write_binary(&ecdsa.grp, &ecdsa.Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, buf, sizeof(buf));
    Serial.println("=== Copy this ECDSA public key to your peer's code ===");
    Serial.print("const uint8_t peer_ecdsa_pubkey[65] = {");
    for (size_t i = 0; i < olen; i++) {
        Serial.print("0x"); Serial.print(buf[i], HEX);
        if (i < olen - 1) Serial.print(", ");
    }
    Serial.println("};");
    Serial.println("=====================================================");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX); // Initialize Serial2 for GPS communication
  Serial.println("Serial2 Initialized!");
  Wire.begin();
  SPI.begin();

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
  
  Serial.println("Initialising LoRa.");
  delay(1000);
  /* SET LORA CONFIGURATION */
  if(LT.begin(NSS, NRESET, DIO0, DEVICE_SX1278)) {
    Serial.println("SX1278 initialised.");
    delay(100);
  } else {
    Serial.println("SX1278 failed to initialised.");
    while(1);
  }

  delay(50);

  if (!bmp.begin(0x76)) { // 0x76 or 0x77 are common I2C addresses for BMP280
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP280 initialized!");

  delay(20);

  // Frequency: 433MHz,  Bandwidth: 250kHz, Spreading Factor: 7 
  LT.setMode(MODE_STDBY_RC);
  LT.setPacketType(PACKET_TYPE_LORA); // Set LoRa packet type
  LT.setRfFrequency(FREQUENCY, OFFSET); // Set frequency and offset
  LT.calibrateImage(0); // Calibrate image for frequency
  LT.setModulationParams(SPREADING_FACTOR, BANDWIDTH, CODING_RATE, LDRO_AUTO); // Set modulation parameters
  LT.setBufferBaseAddress(0x00, 0x00); // Set buffer base address for TX and RX
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.setSyncWord(LORA_SYNCWORD); // Set sync word for private LoRa network
  LT.setHighSensitivity(); // Enable high sensitivity mode
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);

  /* PRINT LoRa Parameters */
  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);                         //print contents of device registers, normally 0x00 to 0x4F
  Serial.println();
  Serial.println();

  Serial.print(F("Transmitter ready"));
  Serial.println();

  delay(1000);
  Serial.println("Performing Handshake...");
  
  // Perform ECDH/ECDSA handshake
  while (!handshake_done) {
    perform_handshake();
    if(!handshake_done) {
      Serial.println("Handshake failed, retrying...");
      delay(500); // Wait before retrying
    }
  }   
  print_ecdsa_pubkey();
  Serial.println("Handshake completed successfully!");
  Serial.println("AES Key:");
  printHex(aes_key_buf, 16); // Print AES key in hex format
  delay(1000);
}

void loop() {

  readSample();
  //sample_temp();
  delay(100); // 10Hz Telemetry (100ms)

  // if (HC12.available()) {
  //   Serial.write(HC12.read());
  // }
  // if (Serial.available()) {
  //   HC12.write(Serial.read());
  // }
}
// This code sets up a simple serial communication between the Arduino and the HC-12 module.