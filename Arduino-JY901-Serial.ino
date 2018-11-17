#include <Wire.h>
#include "JY901_Serial.h"

/* test field */
// --------------------------------------------------------------------------------
unsigned long previousMillis = 0;
const long interval = 5000;
bool flag = true;
void Sleep() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    flag = !flag;
    // JY901.enterHiber();
    if (flag)
      JY901.turnLED(0);
    else
      JY901.turnLED(1);
  }
}
// --------------------------------------------------------------------------------

/*
Modified for MKR1000 and MKR WiFi 1010.
JY901     MKR1000/1010
  TX   <->   13(Rx)
  RX   <->   14(Tx)
  GND  <->    GND
  VCC  <->    VCC
*/

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  JY901.attach(Serial1);
}

void loop() {
  JY901.receiveSerialData();
  // print received data. Data was received in serialEvent;
  //	printAcc();
  //	printGyro();
  //	printMag();
  //	printAngle();
  Serial.print("Temp:");
  Serial.print(JY901.getTemp());
  Serial.println("");
  delay(500);
  Sleep();
  // add time counting and send sleep
}

void printDate() {  // a demo code from official, but I believe it's for GPS
                    // combination
  Serial.print("Time:20");
  Serial.print(JY901.getTime("year"));
  Serial.print("-");
  Serial.print(JY901.getTime("month"));
  Serial.print("-");
  Serial.print(JY901.getTime("day"));
  Serial.print(" ");
  Serial.print(JY901.getTime("hour"));
  Serial.print(":");
  Serial.print(JY901.getTime("minute"));
  Serial.print(":");
  Serial.println((float)JY901.getTime("second") +
                 (float)JY901.getTime("milisecond") / 1000);
}

void printAcc() {  // Accelerometer data
  Serial.print("Acc:");
  Serial.print(JY901.getAccX());
  Serial.print(" ");
  Serial.print(JY901.getAccY());
  Serial.print(" ");
  Serial.print(JY901.getAccZ());
  Serial.print("\n");
}

void printGyro() {  // Gyroscope data
  Serial.print("Gyro:");
  Serial.print(JY901.getGyroX());
  Serial.print(" ");
  Serial.print(JY901.getGyroY());
  Serial.print(" ");
  Serial.print(JY901.getGyroZ());
  Serial.print("\n");
}

void printMag() {  // Magnetic field data
  Serial.print("Mag:");
  Serial.print(JY901.getMagX());
  Serial.print(" ");
  Serial.print(JY901.getMagY());
  Serial.print(" ");
  Serial.print(JY901.getMagZ());
  Serial.print("\n");
}

void printAngle() {
  Serial.print("Angle:");
  Serial.print(JY901.getRoll());
  Serial.print(" ");
  Serial.print(JY901.getPitch());
  Serial.print(" ");
  Serial.print(JY901.getYaw());
  Serial.print("\n");
}

void printPressure() {  // for JY901B, a special model with pressure measuring
                        // function
  Serial.print("Pressure:");
  Serial.println(JY901.getPressure());
  Serial.print("Altitude:");
  Serial.println(JY901.getAltitude() / 100.0);
}

void printDStatus() {
  Serial.print("DStatus:");
  Serial.print(JY901.getD0Status());
  Serial.print(" ");
  Serial.print(JY901.getD1Status());
  Serial.print(" ");
  Serial.print(JY901.getD2Status());
  Serial.print(" ");
  Serial.print(JY901.getD3Status());
  Serial.print("\n");
}

void printLongLat() {
  Serial.print("Longitude:");
  Serial.print(JY901.getLon() / 10000000);
  Serial.print("Deg");
  Serial.print((double)(JY901.getLon() % 10000000) / 1e5);
  Serial.println("m");

  Serial.print("Lattitude:");
  Serial.print(JY901.getLat() / 10000000);
  Serial.print("Deg");
  Serial.print((double)(JY901.getLat() % 10000000) / 1e5);
  Serial.println("m");
}

void printGPS() {
  Serial.print("GPSHeight:");
  Serial.print(JY901.getGPSH());
  Serial.println("m");
  Serial.print("GPSYaw:");
  Serial.print(JY901.getGPSY());
  Serial.println("Deg:");
  Serial.print("GPSV:");
  Serial.print(JY901.getGPSV());
  Serial.println("km/h");
}
