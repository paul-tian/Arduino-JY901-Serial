#ifndef _JY901_CONTROL_H_
#define _JY901_CONTROL_H_

#include <Arduino.h>

// All these defines (from JY901_SAVE to JY901_GPSVH) are for both
// Serial TO-MODULE functions and IIC usage(which is not this repo's target)
// Copied from Official Mannual PDF (Chinese Ver., p31,32 and p41,42)
// -------------------------------- !!important!!
// --------------------------------
// --- The translation of terms might be inaccurate, please issue if find any.
// ---

#define JY901_CONTROL 0xFF, 0xAA  // for control usage
#define JY901_SAVE 0x00           // save current settings
#define JY901_CALSW 0x01          // calibrate
#define JY901_RSW 0x02            // report data contents
#define JY901_RRATE 0x03          // report rate
#define JY901_BAUD 0x04           // serial baud rate
#define JY901_AXOFFSET 0x05       // X-axis accelerometer offset
#define JY901_AYOFFSET 0x06       // Y-axis accelerometer offset
#define JY901_AZOFFSET 0x07       // Z-axis accelerometer offset
#define JY901_GXOFFSET 0x08       // X-axis angular velocity meter offset
#define JY901_GYOFFSET 0x09       // Y-axis angular velocity meter offset
#define JY901_GZOFFSET 0x0A       // Z-axis angular velocity meter offset
#define JY901_HXOFFSET 0x0B       // X-axis magnetic field meter offset
#define JY901_HYOFFSET 0x0C       // Y-axis magnetic field meter offset
#define JY901_HZOFFSET 0x0D       // Z-axis magnetic field meter offset
#define JY901_D0MODE 0x0E         // D0 mode
#define JY901_D1MODE 0x0F         // D1 mode
#define JY901_D2MODE 0x10         // D2 mode
#define JY901_D3MODE 0x11         // D3 mode
#define JY901_D0PWMH 0x12         // D0 PWM lenth of High Level
#define JY901_D1PWMH 0x13         // D1 PWM lenth of High Level
#define JY901_D2PWMH 0x14         // D2 PWM lenth of High Level
#define JY901_D3PWMH 0x15         // D3 PWM lenth of High Level
#define JY901_D0PWMT 0x16         // D0 PWM period
#define JY901_D1PWMT 0x17         // D1 PWM period
#define JY901_D2PWMT 0x18         // D2 PWM period
#define JY901_D3PWMT 0x19         // D3 PWM period
#define JY901_IICADDR 0x1A        // IIC address
#define JY901_LEDOFF 0x1B         // turn off LED
#define JY901_GPSBAUD 0x1C        // GPS connection baud rate

#define JY901_HIBERNATE 0x22  // module install direction
#define JY901_DIRECTION 0x23  // module install direction
#define JY901_CHANGEALG 0x24  // module install direction

#define JY901_YYMM 0x30       // year and month
#define JY901_DDHH 0x31       // day and hour
#define JY901_MMSS 0x32       // minute and second
#define JY901_MS 0x33         // milliseconds
#define JY901_AX 0x34         // X-axis acceleration
#define JY901_AY 0x35         // Y-axis acceleration
#define JY901_AZ 0x36         // Z-axis acceleration
#define JY901_GX 0x37         // X-axis angular velocity
#define JY901_GY 0x38         // Y-axis angular velocity
#define JY901_GZ 0x39         // Z-axis angular velocity
#define JY901_HX 0x3A         // X-axis magnetic Field
#define JY901_HY 0x3B         // Y-axis magnetic Field
#define JY901_HZ 0x3C         // Z-axis magnetic Field
#define JY901_Roll 0x3D       // X-axis angle
#define JY901_Pitch 0x3E      // Y-axis angle
#define JY901_Yaw 0x3F        // Z-axis angle
#define JY901_TEMP 0x40       // module temperature
#define JY901_D0Status 0x41   // D0 port status
#define JY901_D1Status 0x42   // D1 port status
#define JY901_D2Status 0x43   // D2 port status
#define JY901_D3Status 0x44   // D3 port status
#define JY901_PressureL 0x45  // pressure low word
#define JY901_PressureH 0x46  // pressure high word
#define JY901_HeightL 0x47    // height low word
#define JY901_HeightH 0x48    // height high word
#define JY901_LonL 0x49       // longitude low word
#define JY901_LonH 0x4A       // longitude high word
#define JY901_LatL 0x4B       // lattitude low word
#define JY901_LatH 0x4C       // lattitude high word
#define JY901_GPSHeight 0x4D  // GPS height
#define JY901_GPSYAW 0x4E     // GPS speed angle
#define JY901_GPSVL 0x4F      // GPS speed(ground speed) low word
#define JY901_GPSVH 0x50      // GPS speed(ground speed) high word

// I'm not a specialist and just translated this
#define JY901_Q0 0x51  // quaternion Q0
#define JY901_Q1 0x52  // quaternion Q1
#define JY901_Q2 0x53  // quaternion Q2
#define JY901_Q3 0x54  // quaternion Q3

#define JY901_GYROAUTOCALI 0x63  // gyroscope auto calibration

uint8_t JY901_SAVECONF[5] = {JY901_CONTROL, JY901_SAVE, 0, 0x00};

uint8_t JY901_SETCALI[5] = {JY901_CONTROL, JY901_CALSW, 0, 0x00};

uint8_t JY901_INSTALL[5] = {JY901_CONTROL, JY901_DIRECTION, 0, 0x00};

const uint8_t JY901_SLEEP[5] = {JY901_CONTROL, JY901_HIBERNATE, 0x01, 0x00};

uint8_t JY901_ALGAXIS[5] = {JY901_CONTROL, JY901_CHANGEALG, 0, 0x00};

uint8_t JY901_GYROAUTO[5] = {JY901_CONTROL, JY901_GYROAUTOCALI, 0, 0x00};

uint8_t JY901_RPTCONF[5] = {JY901_CONTROL, JY901_RSW, 0, 0};

uint8_t JY901_RPTRT[5] = {JY901_CONTROL, JY901_RRATE, 0, 0x00};

uint8_t JY901_BAUDRT[5] = {JY901_CONTROL, JY901_BAUD, 0, 0x00};

uint8_t JY901_AXOFF[5] = {JY901_CONTROL, JY901_AXOFFSET, 0, 0};
uint8_t JY901_AYOFF[5] = {JY901_CONTROL, JY901_AYOFFSET, 0, 0};
uint8_t JY901_AZOFF[5] = {JY901_CONTROL, JY901_AZOFFSET, 0, 0};

uint8_t JY901_GXOFF[5] = {JY901_CONTROL, JY901_GXOFFSET, 0, 0};
uint8_t JY901_GYOFF[5] = {JY901_CONTROL, JY901_GYOFFSET, 0, 0};
uint8_t JY901_GZOFF[5] = {JY901_CONTROL, JY901_GZOFFSET, 0, 0};

uint8_t JY901_HXOFF[5] = {JY901_CONTROL, JY901_HXOFFSET, 0, 0};
uint8_t JY901_HYOFF[5] = {JY901_CONTROL, JY901_HYOFFSET, 0, 0};
uint8_t JY901_HZOFF[5] = {JY901_CONTROL, JY901_HZOFFSET, 0, 0};

uint8_t JY901_D0MODECONF[5] = {JY901_CONTROL, JY901_D0MODE, 0, 0x00};
uint8_t JY901_D1MODECONF[5] = {JY901_CONTROL, JY901_D1MODE, 0, 0x00};
uint8_t JY901_D2MODECONF[5] = {JY901_CONTROL, JY901_D2MODE, 0, 0x00};
uint8_t JY901_D3MODECONF[5] = {JY901_CONTROL, JY901_D3MODE, 0, 0x00};

uint8_t JY901_D0PWMHCONF[5] = {JY901_CONTROL, JY901_D0PWMH, 0, 0};
uint8_t JY901_D1PWMHCONF[5] = {JY901_CONTROL, JY901_D1PWMH, 0, 0};
uint8_t JY901_D2PWMHCONF[5] = {JY901_CONTROL, JY901_D2PWMH, 0, 0};
uint8_t JY901_D3PWMHCONF[5] = {JY901_CONTROL, JY901_D3PWMH, 0, 0};

uint8_t JY901_D0PWMTCONF[5] = {JY901_CONTROL, JY901_D0PWMT, 0, 0};
uint8_t JY901_D1PWMTCONF[5] = {JY901_CONTROL, JY901_D1PWMT, 0, 0};
uint8_t JY901_D2PWMTCONF[5] = {JY901_CONTROL, JY901_D2PWMT, 0, 0};
uint8_t JY901_D3PWMTCONF[5] = {JY901_CONTROL, JY901_D3PWMT, 0, 0};

uint8_t JY901_IICADDRESS[5] = {JY901_CONTROL, JY901_IICADDR, 0, 0x00};

uint8_t JY901_LED[5] = {JY901_CONTROL, JY901_LEDOFF, 0, 0x00};

uint8_t JY901_GPSBAUDRATE[5] = {JY901_CONTROL, JY901_GPSBAUD, 0, 0x00};

#endif
