#include <Wire.h>
#include <Servo.h>

// MPU9250 I2C address
#define MPU9250_ADDRESS 0x68

// Servo configuration
#define SERVO_CENTER 90
#define SERVO_MIN 0
#define SERVO_MAX 180
#define COMPENSATION_GAIN 1.0

#define ALPHA 0.96
#define DT 0.02
#define CALIBRATION_SAMPLES 200

Servo rollServo;    // Left-right
Servo pitchServo;   // Up-down

float pitchAngle = 0;
float rollAngle = 0;

float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initMPU9250();

  rollServo.attach(9);
  pitchServo.attach(10);

  rollServo.write(SERVO_CENTER);
  pitchServo.write(SERVO_CENTER);

  Serial.println("Calibrating gyro...");
  calibrateGyro();
  Serial.println("Calibration done!");
}

void loop() {
  static unsigned long prevTime = millis();

  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  readIMU(accX, accY, accZ, gyroX, gyroY, gyroZ);

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
  float accRoll  = atan2(accY, accZ) * 180 / PI;

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // 🔁 FIXED: swap gyroX/gyroY and accPitch/accRoll
  rollAngle  = ALPHA * (rollAngle + gyroY * dt) + (1 - ALPHA) * accPitch;
  pitchAngle = ALPHA * (pitchAngle + gyroX * dt) + (1 - ALPHA) * accRoll;

  int pitchCompensation = -pitchAngle * COMPENSATION_GAIN;
  int rollCompensation  = -rollAngle * COMPENSATION_GAIN;

  rollServo.write(constrain(SERVO_CENTER + rollCompensation, SERVO_MIN, SERVO_MAX));
  pitchServo.write(constrain(SERVO_CENTER + pitchCompensation, SERVO_MIN, SERVO_MAX));

  printSensorData(pitchAngle, rollAngle, pitchCompensation, rollCompensation);
  delay(20);
}

void initMPU9250() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x08); // ±4g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x08); // ±500 dps
  Wire.endTransmission(true);
}

void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }

  gyroXoffset = sumX / CALIBRATION_SAMPLES;
  gyroYoffset = sumY / CALIBRATION_SAMPLES;
  gyroZoffset = sumZ / CALIBRATION_SAMPLES;
}

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);

  ax = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
  ay = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
  az = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);

  gx = (int16_t)(Wire.read() << 8 | Wire.read()) / 65.5;
  gy = (int16_t)(Wire.read() << 8 | Wire.read()) / 65.5;
  gz = (int16_t)(Wire.read() << 8 | Wire.read()) / 65.5;
}

void printSensorData(float pitch, float roll, int pitchComp, int rollComp) {
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("°\tRoll: ");
  Serial.print(roll, 1);
  Serial.print("°\tComp: ");
  Serial.print(pitchComp);
  Serial.print(",");
  Serial.println(rollComp);
}