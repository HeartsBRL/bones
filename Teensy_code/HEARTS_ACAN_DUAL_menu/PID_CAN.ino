/* Edited By Faizan 
 *  Date 9/26/2022
 *  Test Code Version 1
 */



//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 Demo in loopback mode, using hardware SPI, with an external interrupt
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 connections: adapt theses settings to your design
//  This sketch is designed for a Teensy 3.5, using SPI0 (named SPI)
//  But standard Teensy 3.5 SPI0 pins are not used
//    SCK input of MCP2515 is connected to pin #27
//    SI input of MCP2515 is connected to pin #28
//    SO output of MCP2515 is connected to pin #39
//  User code should configure MCP2515_IRQ pin as external interrupt
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK = 13 ; // SCK input of MCP2515
static const byte MCP2515_SI  = 11 ; // SI input of MCP2515
static const byte MCP2515_SO  = 12 ; // SO output of MCP2515

static const byte MCP2515_CS  = 9 ; // CS input of MCP2515 // CAN0
static const byte MCP2515_INT = 2 ; // INT output of MCP2515

static const byte MCP2515_CS_2  = 10 ; // CS input of MCP2515 // CAN1
static const byte MCP2515_INT_2 = 3 ; // INT output of MCP2515

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;
ACAN2515 can2 (MCP2515_CS_2, SPI, MCP2515_INT_2) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————


//----------------------------------------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

static uint32_t gBlinkLedDate2 = 0 ;
static uint32_t gReceivedFrameCount2 = 0 ;
static uint32_t gSentFrameCount2 = 0 ;

//----------------------------------------------------------------------------------------------------------------------
long GenPos = 0;
uint16_t GenVel = 1000;//was 400
unsigned int EstPos = 0;

//----------------- Actuator ID's-------------//

int FRight_Hip = 0x142;
int FRight_Shoulder = 0x142;
int FRight_Knee = 0x142;

int FLeft_Hip = 0x142;
int FLeft_Shoulder = 0x142;
int FLeft_Knee = 0x142;

int BRight_Hip = 0x142;
int BRight_Shoulder = 0x142;
int BRight_Knee = 0x142;

int BLeft_Hip = 0x142;
int BLeft_Shoulder = 0x142;
int BLeft_Knee = 0x142;


int RMD_ID = 0x142;
int RMD_ID2 = 0x142;

//——————————————————————————————————————————————————————————————————————————————
int RMD_ID_Select = 0;
int Delay_time = 1000;

CANMessage frameRx;
CANMessage frameTx;

CANMessage frameTx2;
CANMessage frameRx2;

//-----------------------------------------------------------------

uint8_t currKp;
uint8_t currKi;
uint8_t speedKp;
uint8_t speedKi;
uint8_t PosKp;
uint8_t PosKi;

void setup() 
{
            
            Serial.begin (38400) ;
            //--- Wait for serial (blink led at 10 Hz during waiting)
            while (!Serial) {
              delay (50) ;
              digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
            }
            //--- Define alternate pins for SPI0 (see https://www.pjrc.com/teensy/td_libs_SPI.html)
            //    These settings are defined by Teensyduino for Teensy 3.x
            Serial.print ("Using pin #") ;
            Serial.print (MCP2515_SI) ;
            Serial.print (" for MOSI: ") ;
            Serial.println (SPI.pinIsMOSI (MCP2515_SI) ? "yes" : "NO!!!") ;
            Serial.print ("Using pin #") ;
            Serial.print (MCP2515_SO) ;
            Serial.print (" for MISO: ") ;
            Serial.println (SPI.pinIsMISO (MCP2515_SO) ? "yes" : "NO!!!") ;
            Serial.print ("Using pin #") ;
            Serial.print (MCP2515_SCK) ;
            Serial.print (" for SCK: ") ;
            Serial.println (SPI.pinIsSCK (MCP2515_SCK) ? "yes" : "NO!!!") ;
            SPI.setMOSI (MCP2515_SI) ;
            SPI.setMISO (MCP2515_SO) ;
            SPI.setSCK (MCP2515_SCK) ;
            //--- Configure SPI
            SPI.begin () ;
            //--- Configure ACAN2515
            Serial.println ("Configure ACAN2515") ;
            ACAN2515Settings settings (QUARTZ_FREQUENCY, 1000 * 1000) ; // CAN bit rate 125 kb/s 125 * 1000
            settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select loopback mode LoopBackMode
            const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
          
            if (errorCode == 0) {
              Serial.print ("Bit Rate prescaler: ") ;
              Serial.println (settings.mBitRatePrescaler) ;
              Serial.print ("Propagation Segment: ") ;
              Serial.println (settings.mPropagationSegment) ;
              Serial.print ("Phase segment 1: ") ;
              Serial.println (settings.mPhaseSegment1) ;
              Serial.print ("Phase segment 2: ") ;
              Serial.println (settings.mPhaseSegment2) ;
              Serial.print ("SJW:") ;
              Serial.println (settings.mSJW) ;
              Serial.print ("Triple Sampling: ") ;
              Serial.println (settings.mTripleSampling ? "yes" : "no") ;
              Serial.print ("Actual bit rate: ") ;
              Serial.print (settings.actualBitRate ()) ;
              Serial.println (" bit/s") ;
              Serial.print ("Exact bit rate ? ") ;
              Serial.println (settings.exactBitRate () ? "yes" : "no") ;
              Serial.print ("Sample point: ") ;
              Serial.print (settings.samplePointFromBitStart ()) ;
              Serial.println ("%") ;
            } else {
              Serial.print ("Configuration error 0x") ;
              Serial.println (errorCode, HEX) ;
            }
          
            //--------------bus 2------------------//
            Serial.println ("Configure ACAN2515 2") ;
            settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select loopback mode LoopBackMode
            const uint32_t errorCode_2 = can2.begin (settings, [] { can2.isr () ; }) ;
          
            if (errorCode_2 == 0) {
              Serial.print ("Bit Rate prescaler: ") ;
              Serial.println (settings.mBitRatePrescaler) ;
              Serial.print ("Propagation Segment: ") ;
              Serial.println (settings.mPropagationSegment) ;
              Serial.print ("Phase segment 1: ") ;
              Serial.println (settings.mPhaseSegment1) ;
              Serial.print ("Phase segment 2: ") ;
              Serial.println (settings.mPhaseSegment2) ;
              Serial.print ("SJW:") ;
              Serial.println (settings.mSJW) ;
              Serial.print ("Triple Sampling: ") ;
              Serial.println (settings.mTripleSampling ? "yes" : "no") ;
              Serial.print ("Actual bit rate: ") ;
              Serial.print (settings.actualBitRate ()) ;
              Serial.println (" bit/s") ;
              Serial.print ("Exact bit rate ? ") ;
              Serial.println (settings.exactBitRate () ? "yes" : "no") ;
              Serial.print ("Sample point: ") ;
              Serial.print (settings.samplePointFromBitStart ()) ;
              Serial.println ("%") ;
            } else {
              Serial.print ("Configuration error 0x") ;
              Serial.println (errorCode_2, HEX) ;
            }
          
            //CANMessage frameRx2;
            //CANMessage frameTx2;
            frameTx.len = 8;
            frameTx.id = RMD_ID;
            //request initial position
            frameTx.data[0] = 0x94;// uint8_t{0xA6, 0, 0, 0, 0, 0, 0, 0};
            frameTx.data[1] = 0x00;
            frameTx.data[2] = 0x00;
            frameTx.data[3] = 0x00;
            frameTx.data[4] = 0x00;
            frameTx.data[5] = 0x00;
            frameTx.data[6] = 0x00;
            frameTx.data[7] = 0x00;

            //--------------Back Legs -----------------//

            frameTx2.len = 8;
            frameTx2.id = RMD_ID2;
            //request initial position
            frameTx2.data[0] = 0x94;// uint8_t{0xA6, 0, 0, 0, 0, 0, 0, 0};
            frameTx2.data[1] = 0x00;
            frameTx2.data[2] = 0x00;
            frameTx2.data[3] = 0x00;
            frameTx2.data[4] = 0x00;
            frameTx2.data[5] = 0x00;
            frameTx2.data[6] = 0x00;
            frameTx2.data[7] = 0x00;

}

void loop() {
              
              Serial.println("Enter The ID: ");
              while(!Serial.available());
              int RMD_ID_Select = Serial.parseInt();

              if (RMD_ID_Select == 1) {
                RMD_ID = 0x141;
              }
              else if (RMD_ID_Select == 2) {
                RMD_ID = 0x142;
              }
              else if (RMD_ID_Select == 3) {
                RMD_ID = 0x143;
              }
              else if (RMD_ID_Select == 4) {
                RMD_ID = 0x144;
              }
              else if (RMD_ID_Select == 5) {
                RMD_ID = 0x145;
              }
              else if (RMD_ID_Select == 6) {
                RMD_ID = 0x146;
              }
              else {
                Serial.println("Bus 1, Invalid ID");
              }
          
              frameTx.id = RMD_ID;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();
              receive_bus_1();

              
              currKp = frameRx.data[2];
              currKi = frameRx.data[3];
              speedKp = frameRx.data[4];
              speedKi = frameRx.data[5];
              PosKp = frameRx.data[6];
              PosKi = frameRx.data[7];

              //Writng new values to the actuator

              frameTx.id = 0x146;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              frameTx.id = 0x146;
              frameTx.data[0] = 0x31;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = currKp;
              frameTx.data[3] = currKi;
              frameTx.data[4] = speedKp;
              frameTx.data[5] = speedKi;
              frameTx.data[6] = PosKp;
              frameTx.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_1();

              frameTx.id = 0x146;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              
              receive_bus_1();

              frameTx.id = 0x144;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              frameTx.id = 0x144;
              frameTx.data[0] = 0x31;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = currKp;
              frameTx.data[3] = currKi;
              frameTx.data[4] = speedKp;
              frameTx.data[5] = speedKi;
              frameTx.data[6] = PosKp;
              frameTx.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_1();

              frameTx.id = 0x144;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();
              receive_bus_1();



              //------------------Right Leg------------------------------//


              frameTx.id = 0x141;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              frameTx.id = 0x141;
              frameTx.data[0] = 0x31;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = currKp;
              frameTx.data[3] = currKi;
              frameTx.data[4] = speedKp;
              frameTx.data[5] = speedKi;
              frameTx.data[6] = PosKp;
              frameTx.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_1();

              frameTx.id = 0x141;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              
              receive_bus_1();

              frameTx.id = 0x142;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              frameTx.id = 0x142;
              frameTx.data[0] = 0x31;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = currKp;
              frameTx.data[3] = currKi;
              frameTx.data[4] = speedKp;
              frameTx.data[5] = speedKi;
              frameTx.data[6] = PosKp;
              frameTx.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_1();

              frameTx.id = 0x142;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();
              receive_bus_1();


              frameTx.id = 0x143;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();

              frameTx.id = 0x143;
              frameTx.data[0] = 0x31;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = currKp;
              frameTx.data[3] = currKi;
              frameTx.data[4] = speedKp;
              frameTx.data[5] = speedKi;
              frameTx.data[6] = PosKp;
              frameTx.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_1();

              frameTx.id = 0x143;
              frameTx.data[0] = 0x30;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = 0x00;
              frameTx.data[3] = 0x00;
              frameTx.data[4] = 0x00;
              frameTx.data[5] = 0x00;
              frameTx.data[6] = 0x00;
              frameTx.data[7] = 0x00;
          
              send_bus_1();
              receive_bus_1();



              //----------------------------Back Legs----------------------------//


              frameTx2.id = 0x141;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              frameTx2.id = 0x141;
              frameTx2.data[0] = 0x31;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = currKp;
              frameTx2.data[3] = currKi;
              frameTx2.data[4] = speedKp;
              frameTx2.data[5] = speedKi;
              frameTx2.data[6] = PosKp;
              frameTx2.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_2();

              frameTx2.id = 0x141;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              
              receive_bus_2();


              frameTx2.id = 0x142;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              frameTx2.id = 0x142;
              frameTx2.data[0] = 0x31;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = currKp;
              frameTx2.data[3] = currKi;
              frameTx2.data[4] = speedKp;
              frameTx2.data[5] = speedKi;
              frameTx2.data[6] = PosKp;
              frameTx2.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_2();

              frameTx2.id = 0x142;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              
              receive_bus_2();

              frameTx2.id = 0x143;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              frameTx2.id = 0x143;
              frameTx2.data[0] = 0x31;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = currKp;
              frameTx2.data[3] = currKi;
              frameTx2.data[4] = speedKp;
              frameTx2.data[5] = speedKi;
              frameTx2.data[6] = PosKp;
              frameTx2.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_2();

              frameTx2.id = 0x143;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              
              receive_bus_2();

         //--------------------------Left Legs-------------------//
         
              frameTx2.id = 0x144;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              frameTx2.id = 0x144;
              frameTx2.data[0] = 0x31;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = currKp;
              frameTx2.data[3] = currKi;
              frameTx2.data[4] = speedKp;
              frameTx2.data[5] = speedKi;
              frameTx2.data[6] = PosKp;
              frameTx2.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_2();

              frameTx2.id = 0x144;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              
              receive_bus_2();

              frameTx2.id = 0x145;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              frameTx2.id = 0x145;
              frameTx2.data[0] = 0x31;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = currKp;
              frameTx2.data[3] = currKi;
              frameTx2.data[4] = speedKp;
              frameTx2.data[5] = speedKi;
              frameTx2.data[6] = PosKp;
              frameTx2.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_2();

              frameTx2.id = 0x145;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              
              receive_bus_2();
              

              frameTx2.id = 0x146;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              frameTx2.id = 0x146;
              frameTx2.data[0] = 0x31;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = currKp;
              frameTx2.data[3] = currKi;
              frameTx2.data[4] = speedKp;
              frameTx2.data[5] = speedKi;
              frameTx2.data[6] = PosKp;
              frameTx2.data[7] = PosKi;  // potentially copying ki dont know why?
             
             
          
              send_bus_2();

              frameTx2.id = 0x146;
              frameTx2.data[0] = 0x30;
              frameTx2.data[1] = 0x00;
              frameTx2.data[2] = 0x00;
              frameTx2.data[3] = 0x00;
              frameTx2.data[4] = 0x00;
              frameTx2.data[5] = 0x00;
              frameTx2.data[6] = 0x00;
              frameTx2.data[7] = 0x00;
          
              send_bus_2();

              
              receive_bus_2();
              
}


void send_bus_1() {
  
  const bool ok = can.tryToSend (frameTx) ;
  delay(2);  //delay not optimized
  if (ok) {
    gSentFrameCount += 1 ;
    Serial.print ("Sent on bus 1: ") ;
    Serial.println (gSentFrameCount) ;
    Serial.println(frameTx.data[0], HEX);
  } else {
    Serial.println ("Send failure") ;
  }

}

void send_bus_2() {
  delay(2);
  const bool ok = can2.tryToSend (frameTx2) ;
  if (ok) {
    gSentFrameCount2 += 1 ;
    Serial.print ("Sent on bus 2: ") ;
    Serial.println (gSentFrameCount2) ;
    Serial.println(frameTx2.data[0], HEX);
  } else {
    Serial.println ("Send failure") ;
  }
}


void receive_bus_1() {
        
  if (can.available ()) {
    can.receive (frameRx) ;
    gReceivedFrameCount ++ ;
      
      Serial.print ("Received on bus 1: ") ;
      Serial.println (gReceivedFrameCount) ;
      Serial.print ("RMD ID: ") ;
      Serial.println (frameRx.id, HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[0], HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[1], HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[2], HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[3], HEX);
      Serial.print("\t");

      Serial.print(frameRx.data[4], HEX);
      Serial.print("\t");

      Serial.print(frameRx.data[5], HEX);
      Serial.print("\t");

      Serial.print(frameRx.data[6], HEX);
      Serial.print("\t");

      Serial.print(frameRx.data[7], HEX);
      Serial.println("\t");

  }

  else {
    delay (5);

    if (can.available ()) {
          
      can.receive (frameRx) ;
      gReceivedFrameCount ++ ;
      
      Serial.print ("Received on bus 1: ") ;
      Serial.println (gReceivedFrameCount) ;
      Serial.print ("RMD ID: ") ;
      Serial.println (frameTx.id, HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[0], HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[1], HEX);
      Serial.print("\t");
  
      Serial.print(frameRx.data[2], HEX);
      Serial.print("\t");
 
      Serial.print(frameRx.data[3], HEX);
      Serial.print("\t");
  
      Serial.print(frameRx.data[4], HEX);
      Serial.print("\t");
    
      Serial.print(frameRx.data[5], HEX);
      Serial.print("\t");
     
      Serial.print(frameRx.data[6], HEX);
      Serial.print("\t");
  
      Serial.print(frameRx.data[7], HEX);
      Serial.println("\t");
    }
  }
}


void receive_bus_2() {
        
  if (can2.available ()) {
    can2.receive (frameRx2) ;
    gReceivedFrameCount ++ ;
      
      Serial.print ("Received on bus 1: ") ;
      Serial.println (gReceivedFrameCount) ;
      Serial.print ("RMD ID: ") ;
      Serial.println (frameRx2.id, HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[0], HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[1], HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[2], HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[3], HEX);
      Serial.print("\t");

      Serial.print(frameRx2.data[4], HEX);
      Serial.print("\t");

      Serial.print(frameRx2.data[5], HEX);
      Serial.print("\t");

      Serial.print(frameRx2.data[6], HEX);
      Serial.print("\t");

      Serial.print(frameRx2.data[7], HEX);
      Serial.println("\t");

  }

  else {
    delay (5);

    if (can2.available ()) {
          
      can2.receive (frameRx2) ;
      gReceivedFrameCount ++ ;
      
      Serial.print ("Received on bus 1: ") ;
      Serial.println (gReceivedFrameCount) ;
      Serial.print ("RMD ID: ") ;
      Serial.println (frameTx2.id, HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[0], HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[1], HEX);
      Serial.print("\t");
  
      Serial.print(frameRx2.data[2], HEX);
      Serial.print("\t");
 
      Serial.print(frameRx2.data[3], HEX);
      Serial.print("\t");
  
      Serial.print(frameRx2.data[4], HEX);
      Serial.print("\t");
    
      Serial.print(frameRx2.data[5], HEX);
      Serial.print("\t");
     
      Serial.print(frameRx2.data[6], HEX);
      Serial.print("\t");
  
      Serial.print(frameRx2.data[7], HEX);
      Serial.println("\t");
    }
  }
}
