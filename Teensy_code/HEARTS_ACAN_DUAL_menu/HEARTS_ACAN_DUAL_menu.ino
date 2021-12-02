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

static const byte MCP2515_CS  = 9 ; // CS input of MCP2515
static const byte MCP2515_INT = 2 ; // INT output of MCP2515

static const byte MCP2515_CS_2  = 10 ; // CS input of MCP2515
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


void loop () {







  //
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    /////////////
    ///////////--------Menu-------------/////////////////////--------Menu-------------//////////
    //comment out unused functions
    //input_command_bus_1();
    //input_command_bus_2();
    //taps_test();
    leg_test();
    ///////////--------Menu-------------/////////////////////--------Menu-------------//////////
  }

  // put in function with 3 tries or error message
  ///////////--------Bus 1 receive -------------//////////
  if (can.available ()) {
    can.receive (frameRx) ;
    gReceivedFrameCount ++ ;
    Serial.print ("Received on bus 1: ") ;
    Serial.println (gReceivedFrameCount) ;
    Serial.print ("RMD ID: ") ;
    Serial.println (frameTx2.id, HEX);
    Serial.print((frameRx.data[6] << 8) | frameRx.data[7]);
    Serial.print("\t");
    EstPos = (frameRx.data[7] << 8) | frameRx.data[6];
    if (EstPos > 18000) {
      GenPos = -36000 + EstPos;
    } else {
      GenPos = EstPos;
    }
    Serial.print(frameRx.data[6], HEX);
    Serial.print("\t");
    Serial.print(frameRx.data[7], HEX);
    Serial.print("\t");
    Serial.print(EstPos);
    Serial.print("\t");
    Serial.println(GenPos);
  }
  ///////////--------Bus 2 receive-------------//////////
  receive_bus_2();
}
//——————————————————————————————————————————————————————————————————————————————
//——————————————————————————————————————————————————————————————————————————————
//——————————————————————————————————————————————————————————————————————————————

void zero_pos_1() {
  GenPos = 0;
  GenVel = 200;
  ////
  RMD_ID = 0x141;
  frameTx.id = RMD_ID;
  frameTx.data[0] = 0xA4;
  frameTx.data[1] = 0x00;
  frameTx.data[2] = GenVel;
  frameTx.data[3] = GenVel >> 8;
  frameTx.data[4] = GenPos;
  frameTx.data[5] = GenPos >> 8;
  frameTx.data[6] = GenPos >> 16;
  frameTx.data[7] = GenPos >> 24;
  send_bus_1();
  delay(10);
  receive_bus_1();
  ////
  RMD_ID = 0x142;
  frameTx.id = RMD_ID;
  frameTx.data[0] = 0xA4;
  frameTx.data[1] = 0x00;
  frameTx.data[2] = GenVel;
  frameTx.data[3] = GenVel >> 8;
  frameTx.data[4] = GenPos;
  frameTx.data[5] = GenPos >> 8;
  frameTx.data[6] = GenPos >> 16;
  frameTx.data[7] = GenPos >> 24;
  send_bus_1();
  delay(10);
  receive_bus_1();
  ////
  RMD_ID = 0x143;
  frameTx.id = RMD_ID;
  frameTx.data[0] = 0xA4;
  frameTx.data[1] = 0x00;
  frameTx.data[2] = GenVel;
  frameTx.data[3] = GenVel >> 8;
  frameTx.data[4] = GenPos;
  frameTx.data[5] = GenPos >> 8;
  frameTx.data[6] = GenPos >> 16;
  frameTx.data[7] = GenPos >> 24;
  send_bus_1();
  delay(10);
  receive_bus_1();
  ////
}

void leg_test(){
int not_used = 0;
  RMD_ID = 0x141;
  frameTx.id = RMD_ID;
  GenPos = 0;
Serial.println("Ready? Press a key and enter to continue... ");
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing
  not_used = Serial.parseInt();
  zero_pos_1();

  Serial.println("Input velocity as int: ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    GenVel = Serial.parseInt();
    Serial.println("GenVel: ");
    Serial.println(GenVel);
    Serial.println("--end");
  }
  //-----
  ////
  GenPos = 90000;
  RMD_ID = 0x141;
  frameTx.id = RMD_ID;
  frameTx.data[0] = 0xA4;
  frameTx.data[1] = 0x00;
  frameTx.data[2] = GenVel;
  frameTx.data[3] = GenVel >> 8;
  frameTx.data[4] = GenPos;
  frameTx.data[5] = GenPos >> 8;
  frameTx.data[6] = GenPos >> 16;
  frameTx.data[7] = GenPos >> 24;
  send_bus_1();
  delay(10);
  receive_bus_1();
  ////
  GenPos = 90000;
  RMD_ID = 0x142;
  frameTx.id = RMD_ID;
  frameTx.data[0] = 0xA4;
  frameTx.data[1] = 0x00;
  frameTx.data[2] = GenVel;
  frameTx.data[3] = GenVel >> 8;
  frameTx.data[4] = GenPos;
  frameTx.data[5] = GenPos >> 8;
  frameTx.data[6] = GenPos >> 16;
  frameTx.data[7] = GenPos >> 24;
  send_bus_1();
  delay(10);
  receive_bus_1();
  ////
  GenPos = 90000;
  RMD_ID = 0x143;
  frameTx.id = RMD_ID;
  frameTx.data[0] = 0xA4;
  frameTx.data[1] = 0x00;
  frameTx.data[2] = GenVel;
  frameTx.data[3] = GenVel >> 8;
  frameTx.data[4] = GenPos;
  frameTx.data[5] = GenPos >> 8;
  frameTx.data[6] = GenPos >> 16;
  frameTx.data[7] = GenPos >> 24;
  send_bus_1();
  delay(10);
  receive_bus_1();
  ////



}

void taps_test() {
  RMD_ID = 0x141;
  frameTx2.id = RMD_ID;
  GenPos = 0;

  Serial.println("Input velocity as int: ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    GenVel = Serial.parseInt();
    Serial.println("GenVel: ");
    Serial.println(GenVel);
    Serial.println("--end");
  }
  //-----
  Serial.println("Input time delay as Int: ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    Delay_time = Serial.parseInt();
    Serial.println("Delay_time: ");
    Serial.println(Delay_time);
    Serial.println("--end");
  }

  for (int i = 0; i <= 3; i++) {
    //lets go forwards and back with the motor
    GenPos = 10000;

    frameTx2.data[0] = 0xA4;
    frameTx2.data[1] = 0x00;
    frameTx2.data[2] = GenVel;
    frameTx2.data[3] = GenVel >> 8;
    frameTx2.data[4] = GenPos;
    frameTx2.data[5] = GenPos >> 8;
    frameTx2.data[6] = GenPos >> 16;
    frameTx2.data[7] = GenPos >> 24;

    send_bus_2();
    delay(10);
    receive_bus_2();
    delay(Delay_time); //Delays are gross and just for prototype, timer/interrrupts are important.
    //-----
    GenPos = 0;

    frameTx2.data[0] = 0xA4;
    frameTx2.data[1] = 0x00;
    frameTx2.data[2] = GenVel;
    frameTx2.data[3] = GenVel >> 8;
    frameTx2.data[4] = GenPos;
    frameTx2.data[5] = GenPos >> 8;
    frameTx2.data[6] = GenPos >> 16;
    frameTx2.data[7] = GenPos >> 24;

    send_bus_2();
    //delay(Delay_time); //Delays are gross and just for prototype, timer/interrrupts are important.
    delay(10);
    receive_bus_2();
    while (GenPos > 1000) {

      delay(10);
      send_bus_2();
      delay(10);
      receive_bus_2();
      Serial.print("GenPos: ");
      Serial.println(GenPos);
    }
    delay(10);
    //-----

    GenPos = 30000;

    frameTx2.data[0] = 0xA4;
    frameTx2.data[1] = 0x00;
    frameTx2.data[2] = GenVel;
    frameTx2.data[3] = GenVel >> 8;
    frameTx2.data[4] = GenPos;
    frameTx2.data[5] = GenPos >> 8;
    frameTx2.data[6] = GenPos >> 16;
    frameTx2.data[7] = GenPos >> 24;
    send_bus_2();
    delay(Delay_time); //Delays are gross and just for prototype, timer/interrrupts are important.

    receive_bus_2();

  }

}

void receive_bus_2() {
  if (can2.available ()) {
    can2.receive (frameRx2) ;
    gReceivedFrameCount2 ++ ;
    Serial.print ("Received on bus 2: ") ;
    Serial.println (gReceivedFrameCount2) ;
    Serial.print ("RMD ID: ") ;
    Serial.println (frameTx2.id, HEX);
    Serial.print((frameRx2.data[6] << 8) | frameRx2.data[7]);
    Serial.print("\t");
    EstPos = (frameRx2.data[7] << 8) | frameRx2.data[6];
    if (EstPos > 18000) {
      GenPos = -36000 + EstPos;
    } else {
      GenPos = EstPos;
    }
    Serial.print(frameRx2.data[6], HEX);
    Serial.print("\t");
    Serial.print(frameRx2.data[7], HEX);
    Serial.print("\t");
    Serial.print(EstPos);
    Serial.print("\t");
    Serial.println(GenPos);
  }

  else {
    delay (5);

    if (can2.available ()) {
      can2.receive (frameRx2) ;
      gReceivedFrameCount2 ++ ;
      Serial.print ("Received on bus 2: ") ;
      Serial.println (gReceivedFrameCount2) ;
      Serial.print ("RMD ID: ") ;
      Serial.println (frameTx2.id, HEX);
      Serial.print((frameRx2.data[6] << 8) | frameRx2.data[7]);
      Serial.print("\t");
      EstPos = (frameRx2.data[7] << 8) | frameRx2.data[6];
      if (EstPos > 18000) {
        GenPos = -36000 + EstPos;
      } else {
        GenPos = EstPos;
      }
      Serial.print(frameRx2.data[6], HEX);
      Serial.print("\t");
      Serial.print(frameRx2.data[7], HEX);
      Serial.print("\t");
      Serial.print(EstPos);
      Serial.print("\t");
      Serial.println(GenPos);
    }
  }

}
void receive_bus_1() {
  if (can.available ()) {
    can.receive (frameRx) ;
    gReceivedFrameCount ++ ;
    Serial.print ("Received on bus 1: ") ;
    Serial.println (gReceivedFrameCount) ;
    Serial.print ("RMD ID: ") ;
    Serial.println (frameTx.id, HEX);
    Serial.print((frameRx.data[6] << 8) | frameRx.data[7]);
    Serial.print("\t");
    EstPos = (frameRx.data[7] << 8) | frameRx.data[6];
    if (EstPos > 18000) {
      GenPos = -36000 + EstPos;
    } else {
      GenPos = EstPos;
    }
    Serial.print(frameRx.data[6], HEX);
    Serial.print("\t");
    Serial.print(frameRx.data[7], HEX);
    Serial.print("\t");
    Serial.print(EstPos);
    Serial.print("\t");
    Serial.println(GenPos);
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
      Serial.print((frameRx.data[6] << 8) | frameRx.data[7]);
      Serial.print("\t");
      EstPos = (frameRx.data[7] << 8) | frameRx.data[6];
      if (EstPos > 18000) {
        GenPos = -36000 + EstPos;
      } else {
        GenPos = EstPos;
      }
      Serial.print(frameRx.data[6], HEX);
      Serial.print("\t");
      Serial.print(frameRx.data[7], HEX);
      Serial.print("\t");
      Serial.print(EstPos);
      Serial.print("\t");
      Serial.println(GenPos);
    }
  }

}

void input_command_bus_1() {
  Serial.println("Bus 1, Input Position as Int: ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    GenPos = Serial.parseInt();
    Serial.println("GenPos: ");
    Serial.println(GenPos);
    Serial.println("--end");
  }

  Serial.println("Bus 1, Input ID (1-6): ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    RMD_ID_Select = Serial.parseInt();

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


    Serial.println("Bus 1, ID selected: ");
    Serial.println(RMD_ID, HEX);
    Serial.println("--end");
  }
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

}

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
//------------------------------------------
void input_command_bus_2() {

  /////////////
  Serial.println("Bus 2, Input Position as Int: ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    GenPos = Serial.parseInt();
    Serial.println("GenPos: ");
    Serial.println(GenPos);
    Serial.println("--end");
  }

  Serial.println("Bus 2, Input ID (1-6): ");
  // Actuator id i.e define 1 = 0x141, velocity(check max), position
  while (!Serial.available()) {
  }; //remove this blocking function later, its just for a testing

  if (Serial.available()) {
    RMD_ID_Select = Serial.parseInt();

    if (RMD_ID_Select == 1) {
      RMD_ID2 = 0x141;
    }
    else if (RMD_ID_Select == 2) {
      RMD_ID2 = 0x142;
    }
    else if (RMD_ID_Select == 3) {
      RMD_ID2 = 0x143;
    }
    else if (RMD_ID_Select == 4) {
      RMD_ID2 = 0x144;
    }
    else if (RMD_ID_Select == 5) {
      RMD_ID2 = 0x145;
    }
    else if (RMD_ID_Select == 6) {
      RMD_ID2 = 0x146;
    }
    else {
      Serial.println("Bus 2, Invalid ID");
    }
    frameTx2.id = RMD_ID2;


    Serial.println("Bus 2, ID selected: ");
    Serial.println(RMD_ID2, HEX);
    Serial.println("--end");
  }
  /////////////
  frameTx2.data[0] = 0xA4;
  frameTx2.data[1] = 0x00;
  frameTx2.data[2] = GenVel;
  frameTx2.data[3] = GenVel >> 8;
  frameTx2.data[4] = GenPos;
  frameTx2.data[5] = GenPos >> 8;
  frameTx2.data[6] = GenPos >> 16;
  frameTx2.data[7] = GenPos >> 24;

  send_bus_2();

}
void send_bus_2() {
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

//——————————————————————————————————————————————————————————————————————————————
