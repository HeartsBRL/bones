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
uint16_t GenVel = 65000;//was 400
unsigned int EstPos = 0;
int RMD_ID = 0x142;
int RMD_ID2 = 0x142;

//——————————————————————————————————————————————————————————————————————————————
int RMD_ID_Select = 0;
int Delay_time = 1000;

CANMessage frameRx;
CANMessage frameTx;

CANMessage frameTx2;
CANMessage frameRx2;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

          void setup () {
            //--- Switch on builtin led
            pinMode (LED_BUILTIN, OUTPUT) ;
            digitalWrite (LED_BUILTIN, HIGH) ;
            //--- Start serial
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  //
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    
    ///////////--------Menu-------------/////////////////////--------Menu-------------//////////

    input_command_bus_1_gait();

    ///////////--------Menu-------------/////////////////////--------Menu-------------//////////
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



            void input_command_bus_1_gait() {
            
            
            
            int Angles_to_pos = 903;
            int joint_angles_1[5] = {10, -10, -20, -10, 10};   // Go towards the negative!!!!! dont go positive  // testing right leg
            int joint_angles_2[5] = {10, 20, 30, 20, 10};      // Keep it Positive
            int joint_angles_3[5] = {10, -10, -20, -10, 10};   // Go towards the negative!!!!! dont go positive  // testing right leg
            int gait_steps = 5;
          
          
          
            Serial.println("Bus 1, Input gait steps as Int: ");
            // Actuator id i.e define 1 = 0x141, velocity(check max), position
            while (!Serial.available()) {
            }; //remove this blocking function later, its just for a testing
          
          
            if (Serial.available()) {
              gait_steps = Serial.parseInt();
              Serial.println("Gait steps 5 max: ");
              Serial.println(gait_steps);
              Serial.println("--end");
            }
          
            IK_xyz(2, 4, 6); //contoh x,y,z
            Serial.print("gama= ");
            Serial.print(gama);
            Serial.print(", alpha= ");
            Serial.print(alpha);
            Serial.print(", beta= ");
            Serial.print(beta);
            Serial.println();
          
          
            for (int i = 0; i < gait_steps; i++) {
          
              GenPos = (903 * joint_angles_1[i]);
              RMD_ID = 0x141;
              frameTx.id = RMD_ID;
              GenVel = 400;
          
          
              /////////////
              frameTx.data[0] = 0xA4;
              frameTx.data[1] = 0x00;
              frameTx.data[2] = GenVel;
              frameTx.data[3] = GenVel >> 8;
              frameTx.data[4] = GenPos;
              frameTx.data[5] = GenPos >> 8;
              frameTx.data[6] = GenPos >> 16;
              frameTx.data[7] = GenPos >> 24;
          
              send_bus_1();
              delay(3000);
              
            }
          }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void send_bus_1() {
  
  const bool ok = can.tryToSend (frameTx) ;
  if (ok) {
    gSentFrameCount += 1 ;
    Serial.print ("Sent on bus 1: ") ;
    Serial.println (gSentFrameCount) ;
    Serial.println(frameTx.data[0], HEX);
  } else {
    Serial.println ("Send failure") ;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IK_xyz(float x, float y, float z)
{
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
