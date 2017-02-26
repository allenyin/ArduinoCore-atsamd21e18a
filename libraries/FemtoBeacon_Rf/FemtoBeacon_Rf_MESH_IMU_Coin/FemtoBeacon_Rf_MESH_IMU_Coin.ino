/**
  FemtoBeacon wirless IMU and LPS platform.
  Mesh networked IMU demo.

  @author A. Alibno <aalbino@femtoduino.com>
  @version 1.0.1

  Modified by Allen Yin, 2/27/2017 
  */

/**
Requires:
https://github.com/femtoduino/FreeIMU-Updates libraries, 
https://github.com/femtoduino/libraries-atmel-lwm,
https://github.com/femtoduino/RTCZero
avr/dtostrf header

This sketch assumes the following FreeIMU.h values:

- MARG should be 4 (DCM)
- MAG_DEC needs to be set to your location's magnetic declination (degrees)
- Calibrate your IMU using the FreeIMU GUI tool (should generate a calibration.h file, include alongside this sketch)
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <Arduino.h>
#include <Wire.h>
//#include <avr/dtostrf.h>  Does not work..
#include <SPI.h>

#include <RTCZero.h>

#define Serial SERIAL_PORT_USBVIRTUAL

//#define DEBUG
//#define OUTPUT_SERIAL


/** BEGIN Atmel's LightWeight Mesh stack. **/
#include "lwm.h"
#include "lwm/sys/sys.h"
#include "lwm/nwk/nwk.h"
/** END Atmel's LightWeight Mesh stack. **/

/** BEGIN mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/
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

#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "DCM.h"
#include "FilteringScheme.h"
#include "RunningAverage.h"
#include "FreeIMU.h"

// Arduino Zero: no eeprom
#define HAS_EEPPROM 0
/** END mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/

/** BEGIN Networking vars **/
extern "C" {
    void                      println(char *x) {
        Serial.println(x);
        Serial.flush();
    }
}

#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE        // 109 bytes
#endif

// Address must be set to 1 for the first device, and to 2 for the second one.
#define APP_ADDRESS         2 // Each coin (node) should have a unique integer address > 1
#define DEST_ADDRESS        1 // The RF Dongle node uses Address 1 (this is of course, for simplicity's sake)
#define APP_ENDPOINT        1 // What callback endpoint number we are using.
#define APP_PANID           0x01
#define APP_SECURITY_KEY    "TestSecurityKey0"
#define APP_CHANNEL         0x1a

char                        bufferData[APP_BUFFER_SIZE];
static NWK_DataReq_t        sendRequest;
static void                 sendMessage(void);
static void                 sendMessageConfirm(NWK_DataReq_t *req);
static bool                 receiveMessage(NWK_DataInd_t *ind);

static bool                 send_message_busy = false;

byte pingCounter            = -2147483648; // Minimum value as a starting point
long tickCounter            = 0;
/** END Networking vars **/

/** BEGIN Sensor vars **/
// Sensor reading.
float ypr[3]; // Hold Yaw-Pitch-Roll (YPR) data.
float eulers[3]; // Hold euler angles (360 deg).
float baro; // Hold Barometer Altitude data
float temp; // Hold Temperature data
float pressure; // Hold Pressure data

// Set the FreeIMU object
FreeIMU sensors = FreeIMU();

// FemtoBeacon FSYNC pin is PA18 (not PA19, which is mislabeled in the silkscreen of FemtoBeacon rev 2.0.0)
// Must connect to GND if FSYNC is unused.
byte PIN_FSYNC = 4;

// FemtoBeacon INT pin is PA19 (not PA18, which is mislabeled in the silkscreen of FemtoBeacon r2.0.0)
byte PIN_INT = 3;
/** END Sensor vars **/


volatile bool is_sensor_on = 1;
volatile bool is_timestamp_on = 1;
volatile bool is_wireless_ok = 1;

/* Create an RTC object */
RTCZero rtc;
volatile unsigned long milliseconds = 0;
volatile unsigned long last_ms = 0;
volatile unsigned long current_ms = 0;

byte seconds = 0;
byte minutes = 0;
byte hours = 0;

byte day = 1; // Day 1
byte month = 1; // January
byte year = 17; // 2017


char delimeter = ',';
char filler = (char) 0;

unsigned long start, finish, elapsed;

void setup() {

    current_ms = millis();
    last_ms = current_ms;

    pinMode(PIN_INT, INPUT);
    pinMode(PIN_FSYNC, OUTPUT);
    digitalWrite(PIN_FSYNC, HIGH);
    delay(10);
    digitalWrite(PIN_FSYNC, LOW);

#ifdef OUTPUT_SERIAL
    setupSerialComms();
    Serial.print("Starting LwMesh...");
#endif

    setupMeshNetworking();

#ifdef OUTPUT_SERIAL
    Serial.println("OK.");
    Serial.print("Starting Sensors...");
#endif

    // RTC initialization
    rtc.begin();

    // Sensor initialization
    setupSensors();

#ifdef OUTPUT_SERIAL
    Serial.println("OK.");
    Serial.print("App buffer size is ");
    Serial.println(APP_BUFFER_SIZE);
#endif

    // REQUIRED! calls to dtostrf will otherwise fail (optimized out?)
    char cbuff[7];
    dtostrf(123.4567, 6, 2, cbuff);

#ifdef OUTPUT_SERIAL
    Serial.print("cbuff test is ");
    Serial.println(cbuff);
    Serial.println("OK, ready!");
#endif
}

void setupSerialComms() {
    while (!Serial);


    Serial.begin(115200);
    Serial.print("LWP Ping Demo. Serial comms started. ADDRESS is ");
    Serial.println(APP_ADDRESS);
}

void setupMeshNetworking() {
    SPI.usingInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ));

    SPI.beginTransaction(
            SPISettings(
                MODULE_AT86RF233_CLOCK,
                MSBFIRST,
                SPI_MODE0
                )
            );

    attachInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ), HAL_IrqHandlerSPI, RISING);
    /*  wait for SPI to be ready  */
    delay(10);

    SYS_Init();

    // Set TX Power for internal at86rf233, default is 0x0 (+4 dbm)
    // TX_PWR  0x0 ( +4   dBm)
    // TX_PWR  0x1 ( +3.7 dBm)
    // TX_PWR  0x2 ( +3.4 dBm)
    // TX_PWR  0x3 ( +3   dBm)
    // TX_PWR  0x4 ( +2.5 dBm)
    // TX_PWR  0x5 ( +2   dBm)
    // TX_PWR  0x6 ( +1   dBm)
    // TX_PWR  0x7 (  0   dBm)
    // TX_PWR  0x8 ( -1   dBm)
    // TX_PWR  0x9 ( -2   dBm)
    // TX_PWR  0xA ( -3   dBm)
    // TX_PWR  0xB ( -4   dBm)
    // TX_PWR  0xC ( -6   dBm)
    // TX_PWR  0xD ( -8   dBm)
    // TX_PWR  0xE (-12   dBm)
    // TX_PwR  0xF (-17   dBm)

    // Example:
    PHY_SetTxPower(0x00); // Set to +4 dBm

    NWK_SetAddr(APP_ADDRESS);
    NWK_SetPanId(APP_PANID);
    PHY_SetChannel(APP_CHANNEL);
    PHY_SetRxState(true);
    NWK_OpenEndpoint(APP_ENDPOINT, receiveMessage);
}

void setupSensors() {
    Wire.begin();

    delay(10);

    sensors.init(true); // the parameter enable or disable fast mode
    delay(10);
}


void loop() {

#ifdef OUTPUT_SERIAL
    //  Serial.print("*");
#endif
    //digitalWrite(PIN_FSYNC, HIGH);
    //digitalWrite(PIN_FSYNC, LOW);

    handleNetworking();

}

void handleNetworking()
{
    SYS_TaskHandler();

    if (APP_ADDRESS > 1 && !send_message_busy) {

        // We are reading the sensors only when we can send data.
        // @TODO implement FIFO Stack of sensor data to transmit.
        if (is_sensor_on == true) {
            handleSensors();
        }
        sendMessage();
    }
}

void handleSensors()
{
    byte bufferIndex = 0;

    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
        Serial.println("handleSensors()");
    #endif
    #endif
    current_ms = millis();

    sensors.getYawPitchRoll180(ypr);
    sensors.getEuler360deg(eulers);

    baro = sensors.getBaroAlt();
    temp = sensors.getBaroTemperature();
    pressure = sensors.getBaroPressure();


    /// Use dtostrf?
    // Copy ypr to buffer.
    resetBuffer();

    // ...We need a wide enough size (8 should be enough to cover negative symbol and decimal).
    // ...Precision is 2 decimal places.

    // Timestamp YYYY-MM-DDTHH:II:SS.sss
    /*char tstamp[23];
      sprintf(tstamp,
      "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
      rtc.getYear(), rtc.getMonth(), rtc.getDay(),
      rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(),
      ((int) milliseconds * .001));
    //String timestamp = sprintf("%04d", rtc.getYear()); // + "-" + sprintf("%02d", rtc.getMonth()) + "-" + sprintf("%02d", rtc.getDay()) + "T" + sprintf("%02d", rtc.getHours())( + ":" sprintf("%02d", rtc.getMinutes()) + ":" + sprintf("%02d", rtc.getSeconds()) + "." + sprintf("%03d", (int)milliseconds * .001);
    */
   
    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
        Serial.print("Time=");
        Serial.print(current_ms);
        Serial.print(",");
        Serial.print("(");
        Serial.print(ypr[0]);
        Serial.print(",");
        Serial.print(ypr[1]);
        Serial.print(",");
        Serial.print(ypr[2]); 
        Serial.println(")");
    #endif
    #endif

    // First 10 char is current_ms
    dtostrf(current_ms, 16, 0, &bufferData[bufferIndex]);
    bufferIndex += 16;
    bufferData[bufferIndex] = delimeter;

    // Sensor data:
    // ...Yaw
    ++bufferIndex;
    dtostrf(ypr[0], 16, 6, &bufferData[bufferIndex]);
    bufferIndex += 16;
    bufferData[bufferIndex] = delimeter;

    // ...Pitch
    ++bufferIndex;
    dtostrf(ypr[1], 16, 6, &bufferData[bufferIndex]);
    bufferIndex += 16;
    bufferData[bufferIndex] = delimeter;

    // ...Roll
    ++bufferIndex;
    dtostrf(ypr[2], 16, 6, &bufferData[bufferIndex]);
    bufferIndex += 16;
    bufferData[bufferIndex] = delimeter;

    #ifdef OUTPUT_SERIAL
        Serial.print("TX data: ");
        Serial.println(bufferData);
    #endif
}

void resetBuffer() {
    memset(bufferData, filler, APP_BUFFER_SIZE);
    bufferData[APP_BUFFER_SIZE] = '\0';
}


static void sendMessage(void) {

    if (send_message_busy) {
    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
        Serial.println("...sendMessage() busy");
    #endif
    #endif
        return;
    }

    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
        Serial.println("sendMessage()");
    #endif
    #endif

    sendRequest.dstAddr       = DEST_ADDRESS;
    sendRequest.dstEndpoint   = APP_ENDPOINT; // Endpoint number on destination device
    sendRequest.srcEndpoint   = APP_ENDPOINT; // Local Endpoint number
    sendRequest.options       = NWK_IND_OPT_BROADCAST_PAN_ID; // Broadcast to the PAN ID group.
    sendRequest.data          = (uint8_t*)&bufferData;
    sendRequest.size          = sizeof(bufferData);
    sendRequest.confirm       = sendMessageConfirm;

    NWK_DataReq(&sendRequest);

    send_message_busy = true;
}

static void sendMessageConfirm(NWK_DataReq_t *req)
{
    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
    Serial.println("sendMessageConfirm() req->status is ");
    #endif
    #endif

    send_message_busy = false;
    is_wireless_ok = false;

    switch (req->status) {
        case NWK_SUCCESS_STATUS:
            { 
                is_wireless_ok = true;
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_SUCCESS_STATUS: wireless link ok!");
                #endif
                break;
            }
        case NWK_ERROR_STATUS:
            {
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_ERROR_STATUS");
                #endif
                break;
            }
        case NWK_OUT_OF_MEMORY_STATUS:
            {
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_OUT_OF_MEMORY_STATUS");
                #endif
                break;
            }
        case NWK_NO_ACK_STATUS:
            {
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_NO_ACK_STATUS");
                #endif;
                break;
            }
        case NWK_NO_ROUTE_STATUS:
            {
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_NO_ROUTE_STATUS");
                #endif
                break;
            }
        case NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS:
            {
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS");
                #endif
                break;
            }
        case NWK_PHY_NO_ACK_STATUS:
            {
                #ifdef OUTPUT_SERIAL
                Serial.println("NWK_PHY_NO_ACK_STATUS");
                #endif
                break;
            }
        default:
            break;
    }
    (void) req;
}

static bool receiveMessage(NWK_DataInd_t *ind) {
    //char sensorData[5];
#ifdef OUTPUT_SERIAL
    Serial.print("receiveMessage() ");
    Serial.print("lqi: ");
    Serial.print(ind->lqi, DEC);

    Serial.print("  ");

    Serial.print("rssi: ");
    Serial.print(ind->rssi, DEC);
    Serial.print("  "); 

    Serial.print("data: ");
#endif;

    String str((char*)ind->data);

    if (str.length() > 0)
    {
        if (str.equals("RESET")) {
            // Reset
            sensors.RESET();
            sensors.RESET_Q();
            sensors.init(true);

        } else if (str.equals("SON")) {
            is_sensor_on = true;
        } else if (str.equals("SOFF")) {
            is_sensor_on = false;
        } else if (str.equals("TON")) {
            is_timestamp_on = true;
        } else if (str.equals("TOFF")) {
            is_timestamp_on = false;
        } else if (str.equals("TSET")) {
            // @TODO parse incomming timestamp string
            // and set day, month, year, hours, minutes, seconds, milliseconds.
            int spaceIndex = str.indexOf("T");
            String dmy = str.substring(0, spaceIndex);
            String hms = str.substring(spaceIndex + 1);

            int hyphenIndex = dmy.indexOf("-");
            int secondHyphenIndex = dmy.indexOf("-", hyphenIndex + 1);

            year = (byte)dmy.substring(0, hyphenIndex).toInt();
            month = (byte)dmy.substring(hyphenIndex, secondHyphenIndex).toInt();
            day = (byte)dmy.substring(secondHyphenIndex + 1).toInt();

            int colonIndex = hms.indexOf(":");
            int secondColonIndex = hms.indexOf(":", colonIndex + 1);

            hours = (byte)hms.substring(0, colonIndex).toInt();
            minutes = (byte)hms.substring(colonIndex, secondColonIndex).toInt();

            String secondFractional = hms.substring(secondColonIndex + 1);

            int dotIndex = secondFractional.indexOf(".");

            if (dotIndex > 0) {
                seconds = (byte)secondFractional.substring(0, dotIndex).toInt();

                // Add the board's millis() return value to this.
                milliseconds = secondFractional.substring(dotIndex + 1).toInt();
            } else {
                seconds = (byte)secondFractional.toInt();
            }

            rtc.setHours(hours);
            rtc.setMinutes(minutes);
            rtc.setSeconds(seconds);

            rtc.setDay(day);
            rtc.setMonth(month);
            rtc.setYear(year);

        } else {
            // Ignore...
        }
    }

#ifdef OUTPUT_SERIAL
    Serial.println(str);
#endif

    return true;
}

char *dtostrf(double val, int width, unsigned int prec, char *sout)
{
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvtf(val, prec, &decpt, &sign);
  if (prec == 0 && decpt == 0) {
  s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = sout;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && prec > 0) {
    *p++ = '0';
    *p++ = '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }    
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = '.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return sout;
}

