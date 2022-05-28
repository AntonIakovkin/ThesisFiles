size_t  numm = 1;
void initDMP() {
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  startDMP = true;
}

void getAngles() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX = ypr[2] * toDeg;
    angleY = ypr[1] * toDeg;
    angleZ = ypr[0] * toDeg;
  }
}


byte Read(int reg) {
  Wire.beginTransmission(MPU9255_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9255_I2C_ADDR, numm, false);
  byte val = Wire.read();
  Wire.endTransmission(true);
  return val;
}


void Write(int reg, int data) {
  Wire.beginTransmission(MPU9255_I2C_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}


byte ReadMag(int reg) {
  Wire.beginTransmission(MPU9255_MAG_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9255_MAG_I2C_ADDR, numm, false);
  byte val = Wire.read();
  Wire.endTransmission(true);
  return val;
}


void WriteMag(int reg, int data) {
  Wire.beginTransmission(MPU9255_MAG_I2C_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}


float GetMagneticFlux(int value) {
  return (value * 0.15);
}

float GetAzimuth(int x, int y) {
  float azimuth = atan2(x, y) * 180 / PI;
  return azimuth;
}
