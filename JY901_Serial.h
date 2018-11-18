#ifndef _JY901_SERIAL_H_
#define _JY901_SERIAL_H_

#include <Arduino.h>

class CJY901 {
 public:
  CJY901();                           // construct function
  void attach(Stream& Serial_temp);   // bind serial port
  bool readSerialData(uint8_t data);  // process recieved data
  bool receiveSerialData(void);       // recieve data from serial port

  /* ------------ (JY901 --> Host) functions ------------ */
  uint16_t getTime(const char*);  // get time
  double getTemp();               // get temperature
  double getAccX();               // get X-axis acceleration
  double getAccY();               // get Y-axis acceleration
  double getAccZ();               // get Z-axis acceleration
  double getGyroX();              // get X-axis angular velocity
  double getGyroY();              // get Y-axis angular velocity
  double getGyroZ();              // get Z-axis angular velocity
  double getRoll();               // get X-axis(Roll) angle
  double getPitch();              // get Y-axis(Pitch) angle
  double getYaw();                // get Z-axis(Yaw) angle
  double getMagX();               // get X-axis magnetic field
  double getMagY();               // get Y-axis magnetic field
  double getMagZ();               // get Z-axis magnetic field
  int16_t getD0Status();          // get D0 Status
  int16_t getD1Status();          // get D1 Status
  int16_t getD2Status();          // get D2 Status
  int16_t getD3Status();          // get D3 Status
  int32_t getPressure();          // get pressure(JY-901B)
  int32_t getAltitude();          // get altitude(JY-901B)
  int32_t getLon();               // get lontitude
  int32_t getLat();               // get latitude
  double getGPSH();               // GPS height
  double getGPSY();               // GPS speed angle
  double getGPSV();               // GPS speed
  double getQuater(const char*);  // get quaternion
  double getDOP(const char*);     // get GPS DOP
  unsigned long getLastTime();    // get last receive time
  int16_t getAccRawX();           // get X-axis raw acceleration data
  int16_t getAccRawY();           // get Y-axis raw acceleration data
  int16_t getAccRawZ();           // get Z-axis raw acceleration data
  int16_t getGyroRawX();          // get X-axis raw angular velocity data
  int16_t getGyroRawY();          // get Y-axis raw angular velocity data
  int16_t getGyroRawZ();          // get Z-axis raw angular velocity data
  int16_t getMagRawX();           // get X-axis raw magnetic field data
  int16_t getMagRawY();           // get Y-axis raw magnetic field data
  int16_t getMagRawZ();           // get Z-axis raw magnetic field data

  /* ------------ (Host --> JY901) functions ------------ */
  void saveConf(int);      // save configuration
  void setCali(int);       // calibration mode
  void setDir(int);        // set install direction
  void enterHiber();       // enter hibernation or wake
  void changeALG(int);     // change algorithm
  void autoCaliGyro(int);  // enable auto gyro calibration
  void confReport();       // configure report contents
  void setReportRate(int);
  void setBaudRate(int);

  void setAXoffset();
  void setAYoffset();
  void setAZoffset();

  void setGXoffset();
  void setGYoffset();
  void setGZoffset();

  void setHXoffset();
  void setHYoffset();
  void setHZoffset();

  void setD0mode(int);
  void setD1mode(int);
  void setD2mode(int);
  void setD3mode(int);

  void setD0PWMH();
  void setD1PWMH();
  void setD2PWMH();
  void setD3PWMH();

  void setD0PWMT();
  void setD1PWMT();
  void setD2PWMT();
  void setD3PWMT();

  void setIICaddr(int);
  void turnLED(int);
  void setGPSrate(int);

  struct {
    struct {
      uint8_t confl;
      uint8_t confh;
    } report;

    struct {
      int8_t xl;
      int8_t xh;
      int8_t yl;
      int8_t yh;
      int8_t zl;
      int8_t zh;
    } aoffset;

    struct {
      int8_t xl;
      int8_t xh;
      int8_t yl;
      int8_t yh;
      int8_t zl;
      int8_t zh;
    } goffset;

    struct {
      int8_t xl;
      int8_t xh;
      int8_t yl;
      int8_t yh;
      int8_t zl;
      int8_t zh;
    } hoffset;

    struct {
      uint8_t d0l;
      uint8_t d0h;
      uint8_t d1l;
      uint8_t d1h;
      uint8_t d2l;
      uint8_t d2h;
      uint8_t d3l;
      uint8_t d3h;
    } pwmh;

    struct {
      uint8_t d0l;
      uint8_t d0h;
      uint8_t d1l;
      uint8_t d1h;
      uint8_t d2l;
      uint8_t d2h;
      uint8_t d3l;
      uint8_t d3h;
    } pwmt;

  } JY901_ctrl;

 private:
  Stream* Serial_ = NULL;
  unsigned long lastTime;
  uint8_t rxBuffer[12] = {0};
  uint8_t rxCnt = 0;

  struct {
    struct {
      uint8_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t minute;
      uint8_t second;
      uint16_t milisecond;
    } time;

    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t temperature;
    } acc;

    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t temperature;
    } gyro;

    struct {
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      int16_t temperature;
    } angle;

    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t temperature;
    } mag;

    struct {
      int16_t d0;
      int16_t d1;
      int16_t d2;
      int16_t d3;
    } dStatus;

    int32_t pressure;
    int32_t altitude;  // JY-901B

    int32_t lon;
    int32_t lat;
    int16_t GPSHeight;
    int16_t GPSYaw;
    int32_t GPSVelocity;

    struct {
      int16_t q0;
      int16_t q1;
      int16_t q2;
      int16_t q3;
    } quater;

    struct {  // DOP stands for Dilution of Precision
      int16_t sn;
      int16_t pdop;
      int16_t hdop;
      int16_t vdop;
    } GPS_DOP;

  } JY901_data;
};

extern CJY901 JY901;
#endif
