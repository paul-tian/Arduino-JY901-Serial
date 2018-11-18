
#include "JY901_Serial.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "JY901_Control.h"

CJY901::CJY901() {
  lastTime = millis();
}

void CJY901::attach(Stream& Serial_temp) {
  Serial_ = &Serial_temp;
}

bool CJY901::readSerialData(uint8_t data) {
  rxBuffer[rxCnt] = data;
  rxCnt++;
  if (rxBuffer[0] != 0x55) {  // data start with 0x55 contains what we need
    rxCnt = 0;
    return false;
  }
  if (rxCnt < 11) {
    return false;
  }
  rxCnt = 0;  // reset count to 0

  // do sum check to confirm data is not corrupted
  uint8_t sum = 0;
  for (uint8_t cnt = 0; cnt < 10; cnt++)
    sum += rxBuffer[cnt];
  if (sum != rxBuffer[10])
    return false;

  switch (rxBuffer[1]) {  // these cases are based on Manual p26,27,28,29 and 30
    case 0x50:
      memcpy(&JY901_data.time, &rxBuffer[2], 8);
      break;  // time
    case 0x51:
      memcpy(&JY901_data.acc, &rxBuffer[2], 8);
      break;  // acceleration
    case 0x52:
      memcpy(&JY901_data.gyro, &rxBuffer[2], 8);
      break;  // angular velocity
    case 0x53:
      memcpy(&JY901_data.angle, &rxBuffer[2], 8);
      break;  // angle
    case 0x54:
      memcpy(&JY901_data.mag, &rxBuffer[2], 8);
      break;  // magnetic field and temperature
    case 0x55:
      memcpy(&JY901_data.dStatus, &rxBuffer[2], 8);
      break;  // D port status

    case 0x56:
      memcpy(&JY901_data.pressure, &rxBuffer[2], 4);  // pressure
      memcpy(&JY901_data.altitude, &rxBuffer[6], 4);  // altitude
      break;

    case 0x57:
      memcpy(&JY901_data.lon, &rxBuffer[2], 4);  // longtitude
      memcpy(&JY901_data.lat, &rxBuffer[6], 4);  // latitude
      break;

    case 0x58:
      memcpy(&JY901_data.GPSHeight, &rxBuffer[2], 2);    //
      memcpy(&JY901_data.GPSYaw, &rxBuffer[4], 2);       // GPS data
      memcpy(&JY901_data.GPSVelocity, &rxBuffer[6], 4);  //
      break;

    case 0x59:
      memcpy(&JY901_data.quater, &rxBuffer[2], 8);
      break;  // quaternion
    case 0x5A:
      memcpy(&JY901_data.GPS_DOP, &rxBuffer[2], 8);
      break;  // GPS DOP
  }
  lastTime = millis();  // last receive time
  return true;
}

bool CJY901::receiveSerialData(void) {
  bool status = false;
  while (Serial_->available()) {
    status = CJY901::readSerialData(Serial_->read());
  }
  return status;
}  // if data has been retrieved, return true

/* ------------ (JY901 --> Host) functions ------------ */
uint16_t CJY901::getTime(const char* str) {
  if (strcmp(str, "year") == 0)
    return JY901_data.time.year;  // get year
  if (strcmp(str, "month") == 0)
    return JY901_data.time.month;  // get month
  if (strcmp(str, "day") == 0)
    return JY901_data.time.day;  // get day
  if (strcmp(str, "hour") == 0)
    return JY901_data.time.hour;  // get hour
  if (strcmp(str, "minute") == 0)
    return JY901_data.time.minute;  // get minute
  if (strcmp(str, "second") == 0)
    return JY901_data.time.second;  // get second
  if (strcmp(str, "milisecond") == 0)
    return JY901_data.time.milisecond;  // get milisecond
  return 0;
}  // getTime()

double CJY901::getTemp() {
  return JY901_data.mag.temperature /
         100.0;  // are all the data(from mag acc gyro) same?
}  // may need further test

double CJY901::getAccX() {
  return JY901_data.acc.x / (32768.0 / 16.0);
}  // getAccX() unit: G(gravity)

double CJY901::getAccY() {
  return JY901_data.acc.y / (32768.0 / 16.0);
}  // getAccY() unit: G(gravity)

double CJY901::getAccZ() {
  return JY901_data.acc.z / (32768.0 / 16.0);
}  // getAccZ() unit: G(gravity)

double CJY901::getGyroX() {
  return JY901_data.gyro.x / (32768.0 / 2000.0);
}  // getGyroX() unit: degree(s) per second

double CJY901::getGyroY() {
  return JY901_data.gyro.y / (32768.0 / 2000.0);
}  // getGyroY() unit: degree(s) per second

double CJY901::getGyroZ() {
  return JY901_data.gyro.z / (32768.0 / 2000.0);
}  // getGyroZ() unit: degree(s) per second

/* -- Noticed that The Euler angles' order here is ---- */
/* ----------- Z-Y-X, for more please visit ----------- */
/* --- http://web.mit.edu/2.05/www/Handout/HO2.PDF ---- */
double CJY901::getRoll() {  // X-axis
  return JY901_data.angle.roll / (32768.0 / 180.0);
}  // getRoll() unit: degree(s)

double CJY901::getPitch() {  // Y-axis
  return JY901_data.angle.pitch / (32768.0 / 180.0);
}  // getPitch() unit: degree(s)

double CJY901::getYaw() {  // Z-axis
  return JY901_data.angle.yaw / (32768.0 / 180.0);
}  // getYaw() unit: degree(s)

double CJY901::getMagX() {
  return JY901_data.mag.x / (32768.0 / 180.0);
}  // getMagX()

double CJY901::getMagY() {
  return JY901_data.mag.y / (32768.0 / 180.0);
}  // getMagY()

double CJY901::getMagZ() {
  return JY901_data.mag.z / (32768.0 / 180.0);
}  // getMagZ()

/* ------ The port status output depends on its mode. ------ */
/* ----------- For more, please read the manual. ----------- */
int16_t CJY901::getD0Status() {
  return JY901_data.dStatus.d0;
}
int16_t CJY901::getD1Status() {
  return JY901_data.dStatus.d1;
}
int16_t CJY901::getD2Status() {
  return JY901_data.dStatus.d2;
}
int16_t CJY901::getD3Status() {
  return JY901_data.dStatus.d3;
}

int32_t CJY901::getPressure() {
  return JY901_data.pressure;
}  // getPressure() unit: Pa

int32_t CJY901::getAltitude() {
  return JY901_data.altitude;
}  // getAltitude() unit: cm

/* ------------- According to NMEA8013, ------------ */
/* ------- GPS output format is dd mm.mmmmm, ------- */
/* ----- JY901 output format is ddmm(.)mmmmm, ------ */
/* --------- dd and mm can be calculated ----------- */
/* ---------- by divide(/) and modulo(%) ----------- */
int32_t CJY901::getLon() {
  return JY901_data.lon;
}
int32_t CJY901::getLat() {
  return JY901_data.lat;
}

double CJY901::getGPSH() {
  return JY901_data.GPSHeight / 10.0;
}  // get GPS Height, unit: m(meters)

double CJY901::getGPSY() {
  return JY901_data.GPSYaw / 10.0;
}  // get GPS Yaw, unit: degree(s)

double CJY901::getGPSV() {
  return JY901_data.GPSVelocity / 1000.0;
}  // get GPS Velocity, unit: kilometers per hour

double CJY901::getQuater(const char* str) {
  if (strcmp(str, "q0") == 0)
    return JY901_data.quater.q0;  // get q0
  if (strcmp(str, "q1") == 0)
    return JY901_data.quater.q1;  // get q1
  if (strcmp(str, "q2") == 0)
    return JY901_data.quater.q2;  // get q2
  if (strcmp(str, "q3") == 0)
    return JY901_data.quater.q3;  // get q3

  return 0;
}  // getQuater()

double CJY901::getDOP(const char* str) {
  if (strcmp(str, "sn") == 0)
    return JY901_data.GPS_DOP.sn;  // get number of satellites
  if (strcmp(str, "pdop") == 0)
    return JY901_data.GPS_DOP.pdop;  // get PDOP
  if (strcmp(str, "hdop") == 0)
    return JY901_data.GPS_DOP.hdop;  // get HDOP
  if (strcmp(str, "vdop") == 0)
    return JY901_data.GPS_DOP.vdop;  // get VDOP

  return 0;
}  // getDOP()

unsigned long CJY901::getLastTime() {
  return lastTime;
}  // get last receive time

/* ----------------- Get Raw data if needed ----------------- */
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
/* ----------------- Raw data Functions end ----------------- */

/* ------------ (Host --> JY901) functions ------------ */
void CJY901::saveConf(int saveFlag) {
  JY901_SAVECONF[3] = saveFlag;
  Serial1.write(JY901_SAVECONF, 5);
}  // save configuration

void CJY901::setCali(int caliFlag) {
  JY901_SETCALI[3] = caliFlag;
  Serial1.write(JY901_SETCALI, 5);
}  // calibration mode

void CJY901::setDir(int dirFlag) {
  JY901_INSTALL[3] = dirFlag;
  Serial1.write(JY901_INSTALL, 5);
}  // set install direction

void CJY901::enterHiber() {
  Serial1.write(JY901_SLEEP, 5);
}  // enter hibernation mode, send again to wake

void CJY901::changeALG(int algFlag) {
  JY901_ALGAXIS[3] = algFlag;
  Serial1.write(JY901_ALGAXIS, 5);
}  // change algorithm

void CJY901::autoCaliGyro(int gyroFlag) {
  JY901_GYROAUTO[3] = gyroFlag;
  Serial1.write(JY901_GYROAUTO, 5);
}  // auto gyro calibration

void CJY901::confReport() {
  memcpy(&JY901_RPTCONF[3], &JY901_ctrl.report.confl, 2);
  Serial1.write(JY901_RPTCONF, 5);
}  // need to write conf to  JY901_ctrl.report.conf first

void CJY901::setReportRate(int rateFlag) {
  JY901_RPTRT[3] = rateFlag;
  Serial1.write(JY901_RPTRT, 5);
}

void CJY901::setBaudRate(int baudFlag) {
  JY901_BAUDRT[3] = baudFlag;
  Serial1.write(JY901_BAUDRT, 5);
}

/* ------To avoid negative value been changed, please use --------- */
/* ------------------- memcpy() to set value of ------------------- */
/* --- JY901_ctrl.aoffset JY901_ctrl.goffset JY901_ctrl.hoffset --- */
/* ----------- For more please read the example folder ------------ */
void CJY901::setAXoffset() {
  memcpy(&JY901_AXOFF[3], &JY901_ctrl.aoffset.xl, 2);
  Serial1.write(JY901_AXOFF, 5);
}
void CJY901::setAYoffset() {
  memcpy(&JY901_AYOFF[3], &JY901_ctrl.aoffset.yl, 2);
  Serial1.write(JY901_AYOFF, 5);
}
void CJY901::setAZoffset() {
  memcpy(&JY901_AZOFF[3], &JY901_ctrl.aoffset.zl, 2);
  Serial1.write(JY901_AZOFF, 5);
}

void CJY901::setGXoffset() {
  memcpy(&JY901_GXOFF[3], &JY901_ctrl.goffset.xl, 2);
  Serial1.write(JY901_GXOFF, 5);
}
void CJY901::setGYoffset() {
  memcpy(&JY901_GYOFF[3], &JY901_ctrl.goffset.yl, 2);
  Serial1.write(JY901_GYOFF, 5);
}
void CJY901::setGZoffset() {
  memcpy(&JY901_GZOFF[3], &JY901_ctrl.goffset.zl, 2);
  Serial1.write(JY901_GZOFF, 5);
}

void CJY901::setHXoffset() {
  memcpy(&JY901_HXOFF[3], &JY901_ctrl.hoffset.xl, 2);
  Serial1.write(JY901_HXOFF, 5);
}
void CJY901::setHYoffset() {
  memcpy(&JY901_HYOFF[3], &JY901_ctrl.hoffset.yl, 2);
  Serial1.write(JY901_HYOFF, 5);
}
void CJY901::setHZoffset() {
  memcpy(&JY901_HZOFF[3], &JY901_ctrl.hoffset.zl, 2);
  Serial1.write(JY901_HZOFF, 5);
}

void CJY901::setD0mode(int modeFlag) {
  JY901_D0MODECONF[3] = modeFlag;
  Serial1.write(JY901_D0MODECONF, 5);
}
void CJY901::setD1mode(int modeFlag) {
  JY901_D1MODECONF[3] = modeFlag;
  Serial1.write(JY901_D1MODECONF, 5);
}
void CJY901::setD2mode(int modeFlag) {
  JY901_D2MODECONF[3] = modeFlag;
  Serial1.write(JY901_D2MODECONF, 5);
}
void CJY901::setD3mode(int modeFlag) {
  JY901_D3MODECONF[3] = modeFlag;
  Serial1.write(JY901_D3MODECONF, 5);
}

void CJY901::setD0PWMH() {
  memcpy(&JY901_D0PWMHCONF[3], &JY901_ctrl.pwmh.d0l, 2);
  Serial1.write(JY901_D0PWMHCONF, 5);
}
void CJY901::setD1PWMH() {
  memcpy(&JY901_D1PWMHCONF[3], &JY901_ctrl.pwmh.d1l, 2);
  Serial1.write(JY901_D1PWMHCONF, 5);
}
void CJY901::setD2PWMH() {
  memcpy(&JY901_D2PWMHCONF[3], &JY901_ctrl.pwmh.d2l, 2);
  Serial1.write(JY901_D2PWMHCONF, 5);
}
void CJY901::setD3PWMH() {
  memcpy(&JY901_D3PWMHCONF[3], &JY901_ctrl.pwmh.d3l, 2);
  Serial1.write(JY901_D3PWMHCONF, 5);
}

void CJY901::setD0PWMT() {
  memcpy(&JY901_D0PWMTCONF[3], &JY901_ctrl.pwmt.d0l, 2);
  Serial1.write(JY901_D0PWMTCONF, 5);
}
void CJY901::setD1PWMT() {
  memcpy(&JY901_D1PWMTCONF[3], &JY901_ctrl.pwmt.d1l, 2);
  Serial1.write(JY901_D1PWMTCONF, 5);
}
void CJY901::setD2PWMT() {
  memcpy(&JY901_D2PWMTCONF[3], &JY901_ctrl.pwmt.d2l, 2);
  Serial1.write(JY901_D2PWMTCONF, 5);
}
void CJY901::setD3PWMT() {
  memcpy(&JY901_D3PWMTCONF[3], &JY901_ctrl.pwmt.d3l, 2);
  Serial1.write(JY901_D3PWMTCONF, 5);
}

void CJY901::setIICaddr(int addrFlag) {
  JY901_IICADDRESS[3] = addrFlag;
  Serial1.write(JY901_IICADDRESS, 5);
}

void CJY901::turnLED(int ledFlag) {
  JY901_LED[3] = ledFlag;
  Serial1.write(JY901_LED, 5);
  if (ledFlag == 0)
    Serial.println("LED on");
  else if (ledFlag == 1)
    Serial.println("LED off");
}  // turn off LED, send again to lighten

void CJY901::setGPSrate(int gpsFlag) {
  JY901_GPSBAUDRATE[3] = gpsFlag;
  Serial1.write(JY901_GPSBAUDRATE, 5);
}

CJY901 JY901;
