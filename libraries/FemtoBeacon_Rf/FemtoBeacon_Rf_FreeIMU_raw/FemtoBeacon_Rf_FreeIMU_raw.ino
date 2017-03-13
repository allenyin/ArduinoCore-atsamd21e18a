/**
 * FreeIMU library serial communication protocol
 * 
 * This sketch assumes the following FreeIMU.h values:
 * 
 *   - MAG_DEC needs to be set to your location's magnetic declination (degrees)
 *   - Calibrate your IMU using the FreeIMU GUI tool (should generate a calibration.h file, include alongside this sketch)
 */

#include <Wire.h>
#include <SPI.h>

#define Serial SERIAL_PORT_USBVIRTUAL

//#include "calibration.h" // Uncomment once you have calibrated your IMU, generated a calibration.h file and updated FreeIMU.h!

//These are optional depending on your IMU configuration
//#include <ADXL345.h>
//#include <HMC58X3.h>
//#include <LSM303.h>
//#include <LPS.h> 
//#include <ITG3200.h> //note LPS library must come before ITG lib
//#include <bma180.h>
//#include <MS561101BA.h> //Comment out for APM 2.5
//#include <BMP085.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
//#include <AK8975.h>
#include <AK8963.h>
//#include <L3G.h>
//#include <SFE_LSM9DS0.h>
//#include <BaroSensor.h>
#include <AP_Baro_MS5611.h>  //Uncomment for APM2.5


//These are mandatory
#include <AP_Math_freeimu.h>
#include <Butter.h>    // Butterworth filter
#include <iCompass.h>
#include <MovingAvarageFilter.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "DCM.h"
#include "FilteringScheme.h"
#include "RunningAverage.h"
#include "FreeIMU.h"

//Intel Edison, Arduino 101, Arduino Due, Arduino Zero: no eeprom 
#if defined(__SAMD21G18A__) || defined(_VARIANT_ATSAMR21E18A_) || defined(__SAM3X8E__) || defined(__ARDUINO_ARC__) || defined(__SAMD21G18A__)
  #define HAS_EEPPROM 0
#else
  #include <EEPROM.h>
  #define HAS_EEPPROM 1
#endif

int raw_values[11];     // hold raw values, from FreeIMU::getRawValues()
float cal_values[11];    // hold calibrated values, from FreeIMU:getValues()
char str[512];
float eulers[3];

// Set the default object
FreeIMU my3IMU = FreeIMU();

// FemtoBeacon FSYNC pin is PA18 (not PA19, which is mislabeled in the silkscreen of FemtoBeacon rev 2.0.0)
// Must connect to GND if FSYNC is unused.
byte PIN_FSYNC = 4;

// FemtoBeacon INT pin is PA19 (not PA18, which is mislabeled in the silkscreen of FemtoBeacon r2.0.0)
byte PIN_INT = 3;

void setup() {
  
  pinMode(PIN_INT, INPUT);
  pinMode(PIN_FSYNC, OUTPUT);
  digitalWrite(PIN_FSYNC, HIGH);
  delay(10);
  digitalWrite(PIN_FSYNC, LOW);
  while (!Serial);
  
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Begin IMU...");
  
  delay(100);
  Serial.println("Here...");
  my3IMU.init();
  Serial.println("Here again...");
  delay(100);
  Serial.println("Ok!");
}


void loop() {
    //my3IMU.getRawValues(raw_values);
    //my3IMU.getEuler(eulers);
    my3IMU.getYawPitchRoll180(eulers);
    //my3IMU.getValues(cal_values);

    //float heading = (atan2(cal_values[6], cal_values[7]) * 180.f) / M_PI;
    //Serial.println(heading);

    // without baro
    //sprintf(str, "%d\t%d\t%d\t%d\t%d\t%d", raw_values[6], raw_values[7], raw_values[8], cal_values[6], cal_values[7], cal_values[8]);
    //Serial.println(str);
   
   /* 
    Serial.print(cal_values[0]);
    Serial.print("\t");
    Serial.print(cal_values[1]);
    Serial.print("\t");
    Serial.print(cal_values[2]);
    Serial.print("\t");
    Serial.print(cal_values[3]);
    Serial.print("\t");
    Serial.print(cal_values[4]);
    Serial.print("\t");
    Serial.print(cal_values[5]);
    Serial.print("\t");
   */ 

    /*
    Serial.print(raw_values[6]);
    Serial.print("\t");
    Serial.print(raw_values[7]);
    Serial.print("\t");
    Serial.println(raw_values[8]);
    */

    Serial.print(eulers[0]);
    Serial.print("\t");
    Serial.print(eulers[1]);
    Serial.print("\t");
    Serial.println(eulers[2]); 
}
