#include <string.h>
#include <Wire.h>
#include "JY901.h"

const uint8_t  JY901_imu_cali_cmd[5] = {0xFF,0xAA,0x01,0x01,0x00}; // start
const uint8_t  JY901_mag_cali_cmd[5] = {0xFF,0xAA,0x01,0x02,0x01}; // working on
const uint8_t JY901_quit_cali_cmd[5] = {0xFF,0xAA,0x01,0x00,0x00}; // these
const uint8_t JY901_save_conf_cmd[5] = {0xFF,0xAA,0x00,0x00,0x00}; // commands

CJY901::CJY901() {
  lastTime = millis();
}

void CJY901::attach(Stream & Serial_temp) {
  Serial_ = &Serial_temp;
}

bool CJY901::readSerialData(uint8_t data) {
  rxBuffer[rxCnt] = data;
  rxCnt++;
  if (rxBuffer[0] != 0x55) { // data start with 0x55 contains what we need
    rxCnt = 0;
    return false;
  }
  if (rxCnt<11) {
    return false;
  }
  rxCnt = 0;  // reset count to 0

  // do sum check to confirm data is not corrupted
  uint8_t sum = 0;
  for (uint8_t cnt = 0; cnt<10; cnt++)
    sum += rxBuffer[cnt];
  if (sum != rxBuffer[10])
    return false;

  switch (rxBuffer[1]) { // these cases are based on Manual p26,27,28,29 and 30
    case 0x50:  memcpy(&JY901_data.time,    &rxBuffer[2], 8); break; // time
    case 0x51:  memcpy(&JY901_data.acc,     &rxBuffer[2], 8); break; // acceleration
    case 0x52:  memcpy(&JY901_data.gyro,    &rxBuffer[2], 8); break; // angular velocity
    case 0x53:  memcpy(&JY901_data.angle,   &rxBuffer[2], 8); break; // angle
    case 0x54:  memcpy(&JY901_data.mag,     &rxBuffer[2], 8); break; // magnetic field and temperature
    case 0x55:  memcpy(&JY901_data.dStatus, &rxBuffer[2], 8); break; // D port status
    case 0x56:  memcpy(&JY901_data.pressure,&rxBuffer[2], 4);        // pressure
                memcpy(&JY901_data.altitude,&rxBuffer[6], 4);        // altitude
                break;
    case 0x57:  memcpy(&JY901_data.lon,     &rxBuffer[2], 4);        // longtitude
                memcpy(&JY901_data.lat,     &rxBuffer[6], 4);        // latitude
                break;
    case 0x58:  memcpy(&JY901_data.GPSHeight,   &rxBuffer[2], 2);    //
                memcpy(&JY901_data.GPSYaw,      &rxBuffer[4], 2);    // GPS data
                memcpy(&JY901_data.GPSVelocity, &rxBuffer[6], 4);    //
                break;
  }
  lastTime = millis(); // last receive time
  return true;
}

bool CJY901::receiveSerialData(void) {
  bool status = false;
  while (Serial_->available()) {
    status = CJY901::readSerialData(Serial_->read());
  }
  return status;
} // if data has been retrieved, return true

void CJY901::readData(uint8_t address, uint8_t length, uint8_t data[]) {
  readRegisters(address_, address, length, data);
} // for user's own usage

uint16_t CJY901::getTime(const char* str) {
  if (strcmp(str,       "year") == 0) return       JY901_data.time.year; // get year
  if (strcmp(str,      "month") == 0) return      JY901_data.time.month; // get month
  if (strcmp(str,        "day") == 0) return        JY901_data.time.day; // get day
  if (strcmp(str,       "hour") == 0) return       JY901_data.time.hour; // get hour
  if (strcmp(str,     "minute") == 0) return     JY901_data.time.minute; // get minute
  if (strcmp(str,     "second") == 0) return     JY901_data.time.second; // get second
  if (strcmp(str, "milisecond") == 0) return JY901_data.time.milisecond; // get milisecond

  return 0;
}

double CJY901::getAccX() { // 
  return JY901_data.acc.x / (32768.0/16.0);
}

double CJY901::getAccY() {
  return JY901_data.acc.y / (32768.0/16.0);
}

double CJY901::getAccZ() {
  return JY901_data.acc.z / (32768.0/16.0);
}

double CJY901::getGyroX() {
  return JY901_data.gyro.x / (32768.0/2000.0);
}

double CJY901::getGyroY() {
  return JY901_data.gyro.y / (32768.0/2000.0);
}

double CJY901::getGyroZ() {
  return JY901_data.gyro.z / (32768.0/2000.0);
}

double CJY901::getMagX() {
  return JY901_data.mag.x / (32768.0/180.0);
}

double CJY901::getMagY() {
  return JY901_data.mag.y / (32768.0/180.0);
}

double CJY901::getMagZ() {
  return JY901_data.mag.z / (32768.0/180.0);
}

int16_t CJY901::getAccRawX() {
  return JY901_data.acc.x;
}

int16_t CJY901::getAccRawY() {
  return JY901_data.acc.y;
}

int16_t CJY901::getAccRawZ() {
  return JY901_data.acc.z;
}

int16_t CJY901::getGyroRawX() {
  return JY901_data.gyro.x;
}

int16_t CJY901::getGyroRawY() {
  return JY901_data.gyro.y;
}

int16_t CJY901::getGyroRawZ() {
  return JY901_data.gyro.z;
}

int16_t CJY901::getMagRawX() {
  return JY901_data.mag.x;
}

int16_t CJY901::getMagRawY() {
  return JY901_data.mag.y;
}

int16_t CJY901::getMagRawZ() {
  return JY901_data.mag.z;
}

double CJY901::getRoll() {
  return JY901_data.angle.roll / (32768.0/180.0);
}

double CJY901::getPitch() {
  return JY901_data.angle.pitch / (32768.0/180.0);
}

double CJY901::getYaw() {
  return JY901_data.angle.yaw / (32768.0/180.0);
}

 double CJY901::getTemp() {
  return JY901_data.mag.temperature / 100.0;
}

int32_t CJY901::getPressure(void) {
  return JY901_data.pressure; //Pa
}

int32_t CJY901::getAltitude(void) {
  return JY901_data.altitude; //cm
}

int16_t CJY901::getD0Status() {
  return JY901_data.dStatus.d_0;
}

int16_t CJY901::getD1Status() {
  return JY901_data.dStatus.d_1;
}

int16_t CJY901::getD2Status() {
  return JY901_data.dStatus.d_2;
}

int16_t CJY901::getD3Status() {
  return JY901_data.dStatus.d_3;
}

int32_t CJY901::getLon(void) {
  return JY901_data.lon;
}

int32_t CJY901::getLat(void) {
  return JY901_data.lat;
}

double CJY901::getGPSH(void) {
  return JY901_data.GPSHeight / 10.0;
}

double CJY901::getGPSY(void) {   //度
  return JY901_data.GPSYaw / 10.0;
}

double CJY901::getGPSV(void) {   //km/h
  return JY901_data.GPSVelocity / 1000.0;
}

unsigned long CJY901::getLastTime(void) {
  return lastTime;
}

/* --- The following functions are for IIC only. I'm working on the Serial method funcs. ---- */
//
// void CJY901::saveConf(void) {
//   // if (transferMode_) {
//      uint8_t cmd[2] = {0x00,0x00};
//     writeRegister(address_, JY901_SAVE, 2, cmd);
//   //}
// }

// void CJY901::quitCali(void) {
//   // if (transferMode_) {
//      uint8_t cmd[2] = {0x00,0x00};
//     writeRegister(address_, JY901_CALSW, 2, cmd);
//   //}
// }

// void CJY901::caliIMU(void) {
//   // if (transferMode_) {
//      uint8_t cmd[2] = {0x01,0x00};
//     writeRegister(address_, JY901_CALSW, 2, cmd);
//   //}
// }

// void CJY901::caliMag(void) {
//   // if (transferMode_) {
//      uint8_t cmd[2] = {0x02,0x00};
//     writeRegister(address_, JY901_CALSW, 2, cmd);
//   //}
// }
//
/* ---------------------------- End Line ---------------------------- */


void CJY901::readRegisters(uint8_t deviceAddr, uint8_t addressToRead, uint8_t bytesToRead, uint8_t * dest) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(deviceAddr, bytesToRead); //Ask for bytes, once done, bus is released by default

  if (Wire.available() >= bytesToRead) {//Hang out until we get the # of bytes we expect
    for (int x = 0; x < bytesToRead; x++)
      dest[x] = Wire.read();
    lastTime = millis();
  }
}

void CJY901::writeRegister(uint8_t deviceAddr, uint8_t addressToWrite, uint8_t bytesToWrite, uint8_t *dataToWrite) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToWrite);
  for (int i = 0; i < bytesToWrite; i++)
    Wire.write(dataToWrite[i]);
  Wire.endTransmission(); //Stop transmitting
}

CJY901 JY901;
