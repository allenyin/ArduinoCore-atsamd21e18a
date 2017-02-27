/**
 * FemtoBeacon wirless IMU and LPS platform.
 * Mesh networked IMU demo. Uses Atmel's LWM library (ported for use w/ Arduino)
 *
 * @author A. Alibno <aalbino@femtoduino.com>
 * @version 1.0.1
 */

#include <stdio.h>

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define Serial SERIAL_PORT_USBVIRTUAL
//#define DEBUG
//#define DEBUG_DATA
#define SERIAL_PLOTTER

/** BEGIN Atmel's LightWeight Mesh stack. **/
    #include "lwm.h"
    #include "lwm/sys/sys.h"
    #include "lwm/nwk/nwk.h"
/** END Atmel's LightWeight Mesh stack. **/


/** BEGIN Networking vars **/
extern "C" {
  void                      println(char *x) { Serial.println(x); Serial.flush(); }
}

#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

// Address must be set to 1 for the first device, and to 2 for the second one.
#define APP_ADDRESS         1
#define DEST_ADDRESS        1
#define APP_ENDPOINT        1
#define APP_PANID           0x01
#define APP_SECURITY_KEY    "TestSecurityKey0"
#define APP_CHANNEL         0x1a

static char                 bufferData[APP_BUFFER_SIZE];
static NWK_DataReq_t        sendRequest;
static void                 sendMessage(void);
static void                 sendMessageConfirm(NWK_DataReq_t *req);
static bool                 receiveMessage(NWK_DataInd_t *ind);

static bool                 send_message_busy = false;

byte pingCounter            = 0;
/** END Networking vars **/

/** BEGIN receiving packet processing vars **/
int splitIndex;
int secondSplitIndex;
int thirdSplitIndex;
int fourthSplitIndex;
unsigned long timestamp;
    
float yaw_value;
float pitch_value;
float roll_value;
/** END packet processing vars **/

#ifdef DEBUG
unsigned long start = millis();
#endif

void setup() {
  // put your setup code here, to run once:
  setupSerialComms();
  Serial.print("Starting LwMesh...");
  setupMeshNetworking();
  Serial.println("OK.");
  delay(500);
  
}

void setupSerialComms() {
    //while(!Serial);
    
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

void loop() {
  
  handleNetworking();
  
  //Serial.println("----");
  
  //delay(1000);
}

void handleNetworking()
{
    SYS_TaskHandler();
   
    #ifdef DEBUG 
    if (millis() - start > 2000) {    
        Serial.print("Node #");
        Serial.print(APP_ADDRESS);
        Serial.println(" handleNetworking()");
        start = millis();
    }
    #endif
}

static bool receiveMessage(NWK_DataInd_t *ind) {
    /* Data
       Data is a string with format:
       "Timestamp,Yaw,Pitch,Roll"
       Timestamp is in ms (since Coin starts)
       Yaw,Pitch,Roll in radians...I think?
    */
    
    char* data = (char*) ind->data;

    String str(data);

    splitIndex = str.indexOf(',');
    secondSplitIndex = str.indexOf(',', splitIndex + 1);
    thirdSplitIndex = str.indexOf(',', secondSplitIndex + 1);
    fourthSplitIndex = str.indexOf(',', thirdSplitIndex + 1);
           
    timestamp = str.substring(0, splitIndex).toInt();
    yaw_value = str.substring(splitIndex + 1, secondSplitIndex).toFloat();
    pitch_value = str.substring(secondSplitIndex + 1, thirdSplitIndex).toFloat();
    roll_value = str.substring(thirdSplitIndex + 1).toFloat();

    /* Output format is a string:
     * NETWORK_ID,CHANNEL_#,RECEIVER_ID,SENDER_ID,TIMESTAMP,YAW,PITCH,ROW
     */

#ifdef DEBUG_DATA
    Serial.print(APP_PANID);    Serial.print(",");
    Serial.print(APP_CHANNEL);  Serial.print(",");
    Serial.print(APP_ADDRESS);  Serial.print(",");
    Serial.print(ind->srcAddr); Serial.print(",");
    Serial.print(timestamp);    Serial.print(",");
    Serial.print(yaw_value);    Serial.print(",");
    Serial.print(pitch_value);  Serial.print(",");
    Serial.print(roll_value);   Serial.print(",");
#endif

#ifdef SERIAL_PLOTTER
    Serial.print(yaw_value);    Serial.print("\t");
    Serial.print(pitch_value);  Serial.print("\t");
    Serial.println(roll_value);   Serial.print("\t");
#endif

    
#ifdef DEBUG
    // network info
    Serial.print("Node #");
    Serial.print(APP_ADDRESS);
    Serial.print(",");
    
    Serial.print("Network #");
    Serial.print(APP_PANID);
    Serial.print(",");
    
    Serial.print("Channel #");
    Serial.print(APP_CHANNEL);
    Serial.print(",");
    
    Serial.print(" receiveMessage() from Node #");
    Serial.print(ind->srcAddr);
    Serial.print(",");

    Serial.print("lqi=");
    Serial.print(ind->lqi, DEC);
    Serial.print(",");

    Serial.print("rssi=");
    Serial.print(ind->rssi, DEC);
    Serial.print(",");

    Serial.print("data: ");
    Serial.println(str);
#endif
    
    return true;
}
