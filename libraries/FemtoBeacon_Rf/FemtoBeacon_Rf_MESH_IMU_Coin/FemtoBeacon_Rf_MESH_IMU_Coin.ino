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

//#include <RTCZero.h>

#define Serial SERIAL_PORT_USBVIRTUAL

//#define DEBUG
//#define OUTPUT_SERIAL
#define READ_YPR
//#define READ_RAW_MAG


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
int raw_values[11]; // sensor raw value readings

// Set the FreeIMU object
FreeIMU sensors = FreeIMU();

// FemtoBeacon FSYNC pin is PA18 (not PA19, which is mislabeled in the silkscreen of FemtoBeacon rev 2.0.0)
// Must connect to GND if FSYNC is unused.
byte PIN_FSYNC = 4;

// FemtoBeacon INT pin is PA19 (not PA18, which is mislabeled in the silkscreen of FemtoBeacon r2.0.0)
byte PIN_INT = 3;
/** END Sensor vars **/


char delimeter = ',';
char filler = (char) 0;

// Timestamp tracking
volatile unsigned long current_ms;

volatile bool is_wireless_ok = 1;


void setup() {

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
    Serial.print("Rf_MESH_IMU_Coin: Serial comms started. ADDRESS is ");
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
    handleNetworking();
}

void handleNetworking()
{
    SYS_TaskHandler();

    if (APP_ADDRESS > 1 && !send_message_busy) {

        // We are reading the sensors only when we can send data.
        // @TODO implement FIFO Stack of sensor data to transmit.
        handleSensors();
        sendMessage();
    }
}

void handleSensors()
{
    byte bufferIndex = 0;

    #ifdef OUTPUT_SERIAL
        Serial.println("handleSensors()");
    #endif
    
    current_ms = millis();
    //sensors.getYawPitchRoll180(ypr);
    sensors.getYawPitchRoll180_stable(ypr);
    //sensors.getEuler360deg(eulers);
    //sensors.getEuler(eulers);
    //sensors.getRawValues(raw_values);

    /// Use dtostrf?
    // Copy ypr to buffer.
    resetBuffer();

    // ...We need a wide enough size (8 should be enough to cover negative symbol and decimal).
    // ...Precision is 2 decimal places.

    #if defined(OUTPUT_SERIAL)
        //Serial.print("Time=");
        //Serial.print(current_ms);
        //Serial.print("\t");
        Serial.print(eulers[0]);
        Serial.print("\t");
        Serial.print(eulers[1]);
        Serial.print("\t");
        Serial.println(eulers[2]); 
    #endif

    // First 10 char is current_ms
    dtostrf(current_ms, 16, 0, &bufferData[bufferIndex]);
    bufferIndex += 16;
    bufferData[bufferIndex] = delimeter;
    
    #ifdef READ_YPR
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
    #endif

    #ifdef READ_RAW_MAG
        // mag
        ++bufferIndex;
        sprintf(&bufferData[bufferIndex], "%016d", raw_values[6]);
        bufferIndex += 16;
        bufferData[bufferIndex] = delimeter; 

        ++bufferIndex;
        sprintf(&bufferData[bufferIndex], "%016d", raw_values[7]);
        bufferIndex += 16;
        bufferData[bufferIndex] = delimeter; 

        ++bufferIndex;
        sprintf(&bufferData[bufferIndex], "%016d", raw_values[8]);
        bufferIndex += 16;
        bufferData[bufferIndex] = delimeter; 
    #endif
    
    #if defined(OUTPUT_SERIAL) && defined(DEBUG)
        //Serial.print("TX data: ");
        //Serial.println(bufferData);
    #endif
}

void resetBuffer() {
    memset(bufferData, filler, APP_BUFFER_SIZE);
    bufferData[APP_BUFFER_SIZE] = '\0';
}


static void sendMessage(void) {

    if (send_message_busy) {
    #if defined(OUTPUT_SERIAL) && defined(DEBUG)
        Serial.println("...sendMessage() busy");
    #endif
        return;
    }

    #if defined(OUTPUT_SERIAL) && defined(DEBUG)
        Serial.println("sendMessage()");
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
    #if defined(OUTPUT_SERIAL) && defined(DEBUG)
        Serial.println("sendMessageConfirm() req->status is ");
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

