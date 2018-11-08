#ifndef _JY901_H_
#define _JY901_H_

/* 
All these defines (from JY901_SAVE to JY901_GPSVH) 
is used for Host -> JY901 data sending,  like calibration, date setting, etc.
These are all from Official Mannual PDF (Chinese Ver., p31,32 and p41,42)
                          !!important!!
--- The translation of terms might be inaccurate, please issue if find any. ---
*/

#define JY901_SAVE         0x00 // save current settings
#define JY901_CALSW        0x01 // calibrate
#define JY901_RSW          0x02 // report data contents
#define JY901_RRATE        0x03 // report rate
#define JY901_BAUD         0x04 // serial baud rate
#define JY901_AXOFFSET     0x05 // X-axis accelerometer offset
#define JY901_AYOFFSET     0x06 // Y-axis accelerometer offset
#define JY901_AZOFFSET     0x07 // Z-axis accelerometer offset
#define JY901_GXOFFSET     0x08 // X-axis angular velocity meter offset
#define JY901_GYOFFSET     0x09 // Y-axis angular velocity meter offset
#define JY901_GZOFFSET     0x0a // Z-axis angular velocity meter offset
#define JY901_HXOFFSET     0x0b // X-axis magnetic field meter offset
#define JY901_HYOFFSET     0x0c // Y-axis magnetic field meter offset
#define JY901_HZOFFSET     0x0d // Z-axis magnetic field meter offset
#define JY901_D0MODE       0x0e // D0 mode
#define JY901_D1MODE       0x0f // D1 mode
#define JY901_D2MODE       0x10 // D2 mode
#define JY901_D3MODE       0x11 // D3 mode
#define JY901_D0PWMH       0x12 // D0 PWM lenth of High Level
#define JY901_D1PWMH       0x13 // D1 PWM lenth of High Level
#define JY901_D2PWMH       0x14 // D2 PWM lenth of High Level
#define JY901_D3PWMH       0x15 // D3 PWM lenth of High Level
#define JY901_D0PWMT       0x16 // D0 PWM period
#define JY901_D1PWMT       0x17 // D1 PWM period
#define JY901_D2PWMT       0x18 // D2 PWM period
#define JY901_D3PWMT       0x19 // D3 PWM period

#define JY901_IICADDR      0x1a // IIC address
#define JY901_LEDOFF       0x1b // turn off LED
#define JY901_GPSBAUD      0x1c // GPS connection baud rate

#define JY901_YYMM         0x30 // year and month
#define JY901_DDHH         0x31 // day and hour
#define JY901_MMSS         0x32 // minute and second
#define JY901_MS           0x33 // milliseconds
#define JY901_AX           0x34 // X-axis acceleration
#define JY901_AY           0x35 // Y-axis acceleration
#define JY901_AZ           0x36 // Z-axis acceleration
#define JY901_GX           0x37 // X-axis angular velocity
#define JY901_GY           0x38 // Y-axis angular velocity
#define JY901_GZ           0x39 // Z-axis angular velocity
#define JY901_HX           0x3a // X-axis magnetic Field
#define JY901_HY           0x3b // Y-axis magnetic Field
#define JY901_HZ           0x3c // Z-axis magnetic Field
#define JY901_Roll         0x3d // X-axis angle
#define JY901_Pitch        0x3e // Y-axis angle
#define JY901_Yaw          0x3f // Z-axis angle
#define JY901_TEMP         0x40 // module temperature
#define JY901_D0Status     0x41 // D0 port status
#define JY901_D1Status     0x42 // D1 port status
#define JY901_D2Status     0x43 // D2 port status
#define JY901_D3Status     0x44 // D3 port status
#define JY901_PressureL    0x45 // pressure low word
#define JY901_PressureH    0x46 // pressure high word
#define JY901_HeightL      0x47 // height low word
#define JY901_HeightH      0x48 // height high word
#define JY901_LonL         0x49 // longitude low word
#define JY901_LonH         0x4a // longitude high word
#define JY901_LatL         0x4b // lattitude low word
#define JY901_LatH         0x4c // lattitude high word
#define JY901_GPSHeight    0x4d // GPS height
#define JY901_GPSYAW       0x4e // GPS speed angle
#define JY901_GPSVL        0x4f // GPS speed(ground speed) low word
#define JY901_GPSVH        0x50 // GPS speed(ground speed) high word
#define JY901_Q0           0x51 // quaternion Q0 (I'm not a specialist and just translated this)
#define JY901_Q1           0x52 // quaternion Q1 (I'm not a specialist and just translated this)
#define JY901_Q2           0x53 // quaternion Q2 (I'm not a specialist and just translated this)
#define JY901_Q3           0x54 // quaternion Q3 (I'm not a specialist and just translated this)

#define JY901_DIO_MODE_AIN     0 // 
#define JY901_DIO_MODE_DIN     1 // 
#define JY901_DIO_MODE_DOH     2 // 
#define JY901_DIO_MODE_DOL     3 // 
#define JY901_DIO_MODE_DOPWM   4 // 
#define JY901_DIO_MODE_GPS     5 // 

#include <Arduino.h>

extern const uint8_t  JY901_imu_cali_cmd[5];
extern const uint8_t  JY901_mag_cali_cmd[5];
extern const uint8_t JY901_quit_cali_cmd[5];
extern const uint8_t JY901_save_conf_cmd[5];

class CJY901 {
  public:
  CJY901();                              // construct function
  void attach(Stream & Serial_temp);     // bind serial port
  void startIIC(uint8_t address = 0x50); // initial IIC with address 0x50
  bool readSerialData(uint8_t data);     // process recieved data
  bool receiveSerialData(void);          // recieve data from serial port
  void readData(uint8_t address,         // data address
                    uint8_t length,      // data length
                    uint8_t data[]);     // address to store the data[length]

  uint16_t getTime(const char*);         // get time
  double   getAccX();                    // get X-axis acceleration
  double   getAccY();                    // get Y-axis acceleration
  double   getAccZ();                    // get Z-axis acceleration
  double   getGyroX();                   // get X-axis angular velocity
  double   getGyroY();                   // get Y-axis angular velocity
  double   getGyroZ();                   // get Z-axis angular velocity
  double   getMagX();                    //获取磁场
  double   getMagY();                    //获取磁场
  double   getMagZ();                    //获取磁场
  int16_t  getAccRawX();                 //获取加速度计原始数据
  int16_t  getAccRawY();                 //获取加速度计原始数据
  int16_t  getAccRawZ();                 //获取加速度计原始数据
  int16_t  getGyroRawX();                //获取陀螺仪原始数据
  int16_t  getGyroRawY();                //获取陀螺仪原始数据
  int16_t  getGyroRawZ();                //获取陀螺仪原始数据
  int16_t  getMagRawX();                 //获取磁力计原始数据
  int16_t  getMagRawY();                 //获取磁力计原始数据
  int16_t  getMagRawZ();                 //获取磁力计原始数据
  double   getRoll();                    // 
  double   getPitch();                   // 
  double   getYaw();                     // 
  double   getTemp();                    // 
  int32_t  getPressure();                //获取压力
  int32_t  getAltitude();                //获取高度
  int16_t  getD0Status();                //获取端口输出
  int16_t  getD1Status();                //获取端口输出
  int16_t  getD2Status();                //获取端口输出
  int16_t  getD3Status();                //获取端口输出
  int32_t  getLon();                     //获取经度
  int32_t  getLat();                     //获取纬度
  double   getGPSH();                    //GPS 高度
  double   getGPSY();                    //GPS 航向角
  double   getGPSV();                    //GPS 地速
  void     saveConf();                   //保存配置
  void     quitCali();                   //退出校准
  void     caliIMU();                    //IMU校准
  void     caliMag();                    //磁力计校准
  unsigned long getLastTime();           // 

  private:
  Stream * Serial_ = NULL;
  uint8_t address_ = 0x50;  //默认设备地址0x50
  bool    transferMode_ = 0;
  unsigned long lastTime;
  uint8_t rxBuffer[12]={0};
  uint8_t rxCnt = 0;
  void readRegisters(uint8_t deviceAddr, uint8_t addressToRead,  uint8_t bytesToRead, uint8_t * dest);
  void writeRegister(uint8_t deviceAddr, uint8_t addressToWrite, uint8_t bytesToWrite, uint8_t * dataToWrite);
  struct {
    struct {
      uint8_t  year;
      uint8_t  month;
      uint8_t  day;
      uint8_t  hour;
      uint8_t  minute;
      uint8_t  second;
      uint16_t milisecond;
    }time;

    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t temperature;
    }acc;
    
    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t temperature;
    }gyro;
    
    struct {
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      int16_t temperature;
    }angle;
    
    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      int16_t temperature;
    }mag;
    
    struct {
      int16_t d_0;
      int16_t d_1;
      int16_t d_2;
      int16_t d_3;
    }dStatus;
    
    int32_t pressure;
    int32_t altitude;
    int32_t lon;
    int32_t lat;

    int16_t GPSHeight;
    int16_t GPSYaw;
    int32_t GPSVelocity;
  }JY901_data;
};

extern CJY901 JY901;
#endif
