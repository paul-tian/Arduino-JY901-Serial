#include <string.h>
#include <Wire.h>
#include "JY901_Serial.h"

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

    case 0x56:  memcpy(&JY901_data.pressure, &rxBuffer[2], 4); // pressure
                memcpy(&JY901_data.altitude, &rxBuffer[6], 4); // altitude
                break;

    case 0x57:  memcpy(&JY901_data.lon, &rxBuffer[2], 4); // longtitude
                memcpy(&JY901_data.lat, &rxBuffer[6], 4); // latitude
                break;

    case 0x58:  memcpy(&JY901_data.GPSHeight,   &rxBuffer[2], 2);    //
                memcpy(&JY901_data.GPSYaw,      &rxBuffer[4], 2);    // GPS data
                memcpy(&JY901_data.GPSVelocity, &rxBuffer[6], 4);    //
                break;

    case 0x59:  memcpy(&JY901_data.quater,  &rxBuffer[2], 8); break; // quaternion
    case 0x5A:  memcpy(&JY901_data.GPS_DOP, &rxBuffer[2], 8); break; // GPS DOP
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

// void CJY901::readData(uint8_t address, uint8_t length, uint8_t data[]) {
//   readRegisters(address_, address, length, data);
// } // for user's own usage IIC?

/* ------------ (JY901 --> Host) functions ------------ */
uint16_t CJY901::getTime(const char* str) {
  if (strcmp(str,       "year") == 0) return       JY901_data.time.year; // get year
  if (strcmp(str,      "month") == 0) return      JY901_data.time.month; // get month
  if (strcmp(str,        "day") == 0) return        JY901_data.time.day; // get day
  if (strcmp(str,       "hour") == 0) return       JY901_data.time.hour; // get hour
  if (strcmp(str,     "minute") == 0) return     JY901_data.time.minute; // get minute
  if (strcmp(str,     "second") == 0) return     JY901_data.time.second; // get second
  if (strcmp(str, "milisecond") == 0) return JY901_data.time.milisecond; // get milisecond

  return 0;
} // getTime()

double CJY901::getTemp() {
  return JY901_data.mag.temperature / 100.0; // are all the data(from mag acc gyro) same?
} // may need further test 

double CJY901::getAccX() {
  return JY901_data.acc.x / (32768.0 / 16.0);
} // getAccX() unit: G(gravity)

double CJY901::getAccY() {
  return JY901_data.acc.y / (32768.0 / 16.0);
} // getAccY() unit: G(gravity)

double CJY901::getAccZ() {
  return JY901_data.acc.z / (32768.0 / 16.0);
} // getAccZ() unit: G(gravity)

double CJY901::getGyroX() {
  return JY901_data.gyro.x / (32768.0 / 2000.0);
} // getGyroX() unit: degree(s) per second

double CJY901::getGyroY() {
  return JY901_data.gyro.y / (32768.0 / 2000.0);
} // getGyroY() unit: degree(s) per second

double CJY901::getGyroZ() {
  return JY901_data.gyro.z / (32768.0 / 2000.0);
} // getGyroZ() unit: degree(s) per second

/* -- Noticed that The Euler angles' order here is ---- */
/* ----------- Z-Y-X, for more please visit ----------- */
/* --- http://web.mit.edu/2.05/www/Handout/HO2.PDF ---- */
double CJY901::getRoll() { // X-axis
  return JY901_data.angle.roll / (32768.0 / 180.0);
} // getRoll() unit: degree(s)

double CJY901::getPitch() { // Y-axis
  return JY901_data.angle.pitch / (32768.0/180.0);
} // getPitch() unit: degree(s)

double CJY901::getYaw() { // Z-axis
  return JY901_data.angle.yaw / (32768.0/180.0);
} // getYaw() unit: degree(s)

double CJY901::getMagX() {
  return JY901_data.mag.x / (32768.0 / 180.0);
} // getMagX()

double CJY901::getMagY() {
  return JY901_data.mag.y / (32768.0 / 180.0);
} // getMagY()

double CJY901::getMagZ() {
  return JY901_data.mag.z / (32768.0 / 180.0);
} // getMagZ()

/* ------ The port status output depends on its mode. ------ */
/* ----------- For more, please read the manual. ----------- */
int16_t CJY901::getD0Status() { return JY901_data.dStatus.d0; }
int16_t CJY901::getD1Status() { return JY901_data.dStatus.d1; }
int16_t CJY901::getD2Status() { return JY901_data.dStatus.d2; }
int16_t CJY901::getD3Status() { return JY901_data.dStatus.d3; }

int32_t CJY901::getPressure() {
  return JY901_data.pressure;
} // getPressure() unit: Pa

int32_t CJY901::getAltitude() {
  return JY901_data.altitude;
} // getAltitude() unit: cm

/* ------------- According to NMEA8013, ------------ */
/* ------- GPS output format is dd mm.mmmmm, ------- */
/* ----- JY901 output format is ddmm(.)mmmmm, ------ */
/* --------- dd and mm can be calculated ----------- */
/* ---------- by divide(/) and modulo(%) ----------- */
int32_t CJY901::getLon() { return JY901_data.lon; }
int32_t CJY901::getLat() { return JY901_data.lat; }

double CJY901::getGPSH() {
  return JY901_data.GPSHeight / 10.0;
} // get GPS Height, unit: m(meters)

double CJY901::getGPSY() {
  return JY901_data.GPSYaw / 10.0;
} // get GPS Yaw, unit: degree(s)

double CJY901::getGPSV() {
  return JY901_data.GPSVelocity / 1000.0;
} // get GPS Velocity, unit: kilometers per hour

double CJY901::getQuater(const char* str) {
  if (strcmp(str, "q0") == 0) return JY901_data.quater.q0; // get q0
  if (strcmp(str, "q1") == 0) return JY901_data.quater.q1; // get q1
  if (strcmp(str, "q2") == 0) return JY901_data.quater.q2; // get q2
  if (strcmp(str, "q3") == 0) return JY901_data.quater.q3; // get q3

  return 0;
} // getQuater()

double CJY901::getDOP(const char* str) {
  if (strcmp(str,   "sn") == 0) return   JY901_data.GPS_DOP.sn; // get number of satellites
  if (strcmp(str, "pdop") == 0) return JY901_data.GPS_DOP.pdop; // get PDOP
  if (strcmp(str, "hdop") == 0) return JY901_data.GPS_DOP.hdop; // get HDOP
  if (strcmp(str, "vdop") == 0) return JY901_data.GPS_DOP.vdop; // get VDOP

  return 0;
} // getDOP()

unsigned long CJY901::getLastTime() {
  return lastTime;
} // get last receive time

/* ----------------- Get Raw data if needed ----------------- */
int16_t  CJY901::getAccRawX() { return  JY901_data.acc.x; }  ///
int16_t  CJY901::getAccRawY() { return  JY901_data.acc.y; }  ///
int16_t  CJY901::getAccRawZ() { return  JY901_data.acc.z; }  ///

int16_t CJY901::getGyroRawX() { return JY901_data.gyro.x; }  ///
int16_t CJY901::getGyroRawY() { return JY901_data.gyro.y; }  ///
int16_t CJY901::getGyroRawZ() { return JY901_data.gyro.z; }  ///

int16_t  CJY901::getMagRawX() { return  JY901_data.mag.x; }  ///
int16_t  CJY901::getMagRawY() { return  JY901_data.mag.y; }  ///
int16_t  CJY901::getMagRawZ() { return  JY901_data.mag.z; }  ///
/* ----------------- Raw data Functions end ----------------- */

/* ------------ (Host --> JY901) functions ------------ */
void CJY901::enterHiber() {
  const uint8_t JY901_Hibernate[5] = {0xFF,0xAA,0x22,0x01,0x00};
  Serial1.write(JY901_Hibernate, 5);
} // enter hibernation mode, send again to wake

void CJY901::turnLED(bool flag) {
  const uint8_t JY901_LED_on[5]  = {0xFF,0xAA,0x1B,0x00,0x00};
  const uint8_t JY901_LED_off[5] = {0xFF,0xAA,0x1B,0x01,0x00};
  if(flag)
    Serial1.write(JY901_LED_on, 5);
  else
    Serial1.write(JY901_LED_off, 5);
} // enter hibernation mode, send again to wake


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
