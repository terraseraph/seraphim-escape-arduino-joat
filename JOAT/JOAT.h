

///////////////////////////////
//  header starts here
//////////////////////////////
#ifndef DEVICES_H     
#define DEVICES_H

// temporary to enable the correct device ids and peripherals
//#define MASTER
#define SLAVE
const char *spacer = "====================";

// Define Devices for this sketc1h
#define RADIO 
#ifdef SLAVE
   #define MY_DEVICE_ID 23         //id of this device.
   #define HUB_DEVICE_ID 255       //hub id
   //#define RFID_CARD
   //#define BUTTON
   #define KEY_PAD
   //#define BUZZER
   //#define MAG_SWITCH
   //#define RELAY
   //#define SONIC_RANGE
   //#define STEPPER_MOTOR
   //#define MP3_PLAYER
   //#define TELEPHONE
   #define MAX_RECEIVED_MESSAGES 5
   #define MAX_MESSAGES 5
   #define MAX_ACKS 5
   #define DEV_DEBUG
#elif defined MASTER
   #define MY_DEVICE_ID 255      //id of this device.
   #define SERIAL_COMMS
   #define MAX_RECEIVED_MESSAGES 1
   #define MAX_MESSAGES 6
   #define MAX_ACKS 6 
   #define DEV_DEBUG 
   //#define LCD_DEBUG             // I2C address 0x27
#endif

#ifdef LCD_DEBUG
  #include <Wire.h>
  #include "LiquidCrystal_I2C.h"
#endif
#if defined(KEY_PAD) || defined(TELEPHONE)
  #include <Key.h>        // keypad
  #include <Keypad.h>     // keypad
#endif
//#include <MemoryFree.h>
#if defined(MP3_PLAYER) || defined(TELEPHONE)
  #include "MP3.h"
#endif
#include <SPI.h>
#include "RF24.h"         // transceiver 
#ifdef RFID_CARD
 //#include <MFRC522.h>    // RF ID reader
 #include "rfid.h"
 #define RFID_CARD_LAPSE_MS 1000
 #define RFID_READ_DELAY 200
#endif
#ifdef DEV_DEBUG
  #define PRINT Serial.print
  #define PRINTLN Serial.println
#else
  #define PRINT //
  #define PRINTLN //
#endif

#define BAUD_RATE 115200
#define MAX_MESSAGE_ID 0x7FFF
#define MESSAGE_PIPE_BASE_ADDRESS 0x59493929
#define ACK_MS_TIMEOUT 600
#define ACK_RETRIES 10

//#define RFID_SS_PIN 8           //RFID cable select
//#define RF24_SS_PIN 9           //RF24 cable select
//#define RST_PIN 10              //shared reset
#define RFID_SS_PIN 8          //RFID cable select
#define RFID_RS_PIN 7           //shared reset
#define RF24_SS_PIN 10           //RF24 cable select
#define RF24_SE_PIN 9           //shared reset

#if defined(KEY_PAD) && defined(TELEPHONE)
  #error "KEY_PAD and TELEPHONE can not be defined at the same time"
#endif
/////////////////////////////
// button pins 0 1 2 3 4 5
/////////////////////////////
#define BTN_PIN0 A0              //button pin
#define BTN_PIN1 A1
#define BTN_PIN2 A2
#define BTN_PIN3 A3
#define BTN_PIN4 A4
#define BTN_PIN5 A5
#define BTN_PIN6 4
#define BTN_PIN7 5
#define BTN_PIN8 6
#define BTN_PIN9 7

///////////////////////////////
// relay pins  4 5 6 7 8
//////////////////////////////
#define REL_PIN1 4              // relay pin
#define REL_PIN2 5
#define REL_PIN3 6
#define REL_PIN4 7
#define REL_PIN5 8
#define REL_PIN6 A0
#define REL_PIN7 A1
#define REL_PIN8 A2

////////////////////////////
// keypad/telephone
///////////////////////////
#define LEFT_PIN1 A1              // Left pin on keyp pad pin1
#define LEFT_PIN2 A2
#define LEFT_PIN3 A3
#define LEFT_PIN4 A4
#define LEFT_PIN5 A5
#define LEFT_PIN6 2
#define LEFT_PIN7 3
#define LEFT_PIN8 4

#if defined(KEY_PAD) || defined(TELEPHONE)
  uint8_t keypad_digits[11];    // number of digits to send, i.e pin number
  uint8_t digit_count = 0;
  /////////////////////////
  // define Telephone keypad
  ////////////////////////
  #ifdef TELEPHONE
    
    // phone constants
    #define ON_HOOK  1            // 0000 0001
    #define OFF_HOOK  2           // 0000 0010
    #define INCOMMING_CALL 4      // 0000 0100
    #define OUTGOING_CALL 8       // 0000 1000
    #define ANSWER 16             // 0001 0000
    #define RINGING 32            // 0010 0000
    #define PENDING_CALL 64       // 0100 0000
    
    #define PHONE_DIGITS 3       // number of digits to dial
    #define POLL_MS 200            // 200 ms between polling for changes
    #define HOOK_PIN 7            // pin number of hangup hook
    #define VOLUME_PIN A0         // pin for volume control
    #define VOLUME_POT_RANGE 1020  // range of volume potentiometer
    #define VOLUME_POT_UNIT (VOLUME_POT_RANGE/30)
    #define INCOMMING_RING_TIMEOUT  // duration of incomming ring
    
    // state variables
    uint8_t phone_volume = 15;        // initialise phone volume at 50%, i.e. 15/30
    uint8_t phone_state = ON_HOOK;    // initialise phone state on-hook
    uint16_t call_duration = 0;        // the minimum number of seconds the call is expected to go for.
    long call_start_time = 0;
    uint8_t mp3_to_play = 0;
    long poll_counter = 0;

  
    // configure the keypad
    const uint8_t KEYPAD_ROWS = 3;
    const uint8_t KEYPAD_COLS = 4;
    char keypadKeys[KEYPAD_ROWS][KEYPAD_COLS] = {
      {'#','9','6','3'},
      {'0','8','5','2'},
      {'*','7','4','1'}
    };
    uint8_t KEYPAD_ROW_PINS[] = {LEFT_PIN1,LEFT_PIN2,LEFT_PIN3,LEFT_PIN4};
    uint8_t KEYPAD_COL_PINS[] = {LEFT_PIN5,LEFT_PIN6,LEFT_PIN7,LEFT_PIN8};
  #endif

  ////////////////////////
  // define keypad
  ////////////////////////
  #ifdef KEY_PAD
    #define KEYPAD_DIGITS 6
    const uint8_t KEYPAD_ROWS = 4;
    const uint8_t KEYPAD_COLS = 4;
    char keypadKeys[KEYPAD_ROWS][KEYPAD_COLS] = {
      {'1','2','3','A'},
      {'4','5','6','B'},
      {'7','8','9','C'},
      {'*','0','#','D'} 
    };
    uint8_t KEYPAD_ROW_PINS[] = {LEFT_PIN1,LEFT_PIN2,LEFT_PIN3,LEFT_PIN4};
    uint8_t KEYPAD_COL_PINS[] = {LEFT_PIN5,LEFT_PIN6,LEFT_PIN7,LEFT_PIN8};
  #endif
#endif

/////////////////////////////////
// BUZZER 
/////////////////////////////////
#define BUZZER_PIN 6

#ifdef BUZZER 
const uint16_t BUZZER_SOUNDS[][4] = {
                              {1000, 100,500, 100}   // Hz and duration, repeat
                              ,{2000, 500,1000, 200}   // Hz and duration, repeat
};

#endif


///////////////////////////////////
// Magnetic Switch
///////////////////////////////////

#ifdef MAG_SWITCH
  bool SWITCH_STATE[] = {false,false,false,false,false,false,false,false,false,false};
#endif

////////////////////////////////////
// distance sensor
////////////////////////////////////
#ifdef SONIC_RANGE
  #define PER_CM 58
  #define TRIGGER 4
  #define ECHO 5
  #define FILTER_SIZE 4

  long distanceFilter[FILTER_SIZE];
  uint8_t distanceHead = 0;
  uint8_t distanceCount = 0;
  bool objectInRange = false;
  int closeRangeThreshold = 20;
  int farRangeThreshold = 30;
#endif

////////////////////////////////////
// Stepper Motor
////////////////////////////////////
#ifdef STEPPER_MOTOR
  #define STEPPER_PIN1 4
  #define STEPPER_PIN2 5
  #define STEPPER_PIN3 6
  #define STEPPER_PIN4 7
  uint8_t stepsPerRevolution = 180;
  uint8_t stepperSpeed = 30;
  uint16_t stepperCurAngle = 0;

#endif

/////////////////////////////////////////////
// Radio Message data structures
// NOTE: addding a new enum
//  1.  Add to enum here
//  2.  Update ParseJson and PacketToJs
//  3.  Approriate ActionTypeInt,etc. Method
//  4.  Add to String array here.
/////////////////////////////////////////////
char enumBuff[16];      // to temp store enum text for debug messages
enum ActionType
{
   noneAT = 0,
   laser,
   relay,
   buzzer,
   distanceSensor,
   stepperMotor,
   mp3,
   phoneAT          
};

const char ActionTypenoneAT[] PROGMEM = "noneAT";
const char ActionTypelaser[] PROGMEM = "laser";
const char ActionTyperelay[] PROGMEM = "relay";
const char ActionTypebuzzer[] PROGMEM = "buzzer";
const char ActionTypedistanceSensor[] PROGMEM = "distanceSensor";
const char ActionTypestepperMotor[] PROGMEM = "stepperMotor";
const char ActionTypemp3[] PROGMEM = "mp3";
const char ActionTypephoneAT[] PROGMEM = "phoneAT";
const char* const ActionTypeString[] PROGMEM = {ActionTypenoneAT,ActionTypelaser,ActionTyperelay,ActionTypebuzzer,ActionTypedistanceSensor,ActionTypestepperMotor,ActionTypemp3, ActionTypephoneAT};

enum Action
{
   noneA = 0,
   start,
   stop,
   configureRange,            
   stepAction,                 // data = speed, absolute, revolutions, angle.  If absolute is 0, then revolutions and angel are used.  If absolute is 1 or -1, motor continues in that direction.
   play,
   volume,
   inCommingCall,                     // incomming call.  data[0] = track to play on mp3, data[1] = length of track in seconds.
   outGoingCall,                       // outgoing call.  data[0] = track to play on mp3, data[1] = length of track in seconds.
   toggleA
};

const char ActionStringnoneA[] PROGMEM = "noneA";
const char ActionStringstart[] PROGMEM = "start";
const char ActionStringstop[] PROGMEM = "stop";
const char ActionStringconfigureRange[] PROGMEM = "configureRange";
const char ActionStringstepAction[] PROGMEM = "stepAction";
const char ActionStringplay[] PROGMEM = "play";
const char ActionStringvolume[] PROGMEM = "volume";
const char ActionStringinCommingCall[] PROGMEM = "inCommingCall";
const char ActionStringoutGoingCall[] PROGMEM = "outGoingCall";
const char ActionStringtoggleA[] PROGMEM = "toggleA";
const char* const ActionString[] PROGMEM = {ActionStringnoneA,ActionStringstart,ActionStringstop,ActionStringconfigureRange,ActionStringstepAction,ActionStringplay,ActionStringvolume, ActionStringinCommingCall,ActionStringoutGoingCall, ActionStringtoggleA};

enum EventType
{
   noneET = 0,
   button,
   keyPad,
   rfId,
   photoRes,
   thermometer,
   potentiometer,
   magnetSwitch,
   distanceDetector,
   stepperDone,              // event to say when the stepper motor has finished moving to requested location.
   phoneET
};

const char EventTypeStringnoneET[] PROGMEM = "noneET";
const char EventTypeStringbutton[] PROGMEM = "button";
const char EventTypeStringkeyPad[] PROGMEM = "keyPad";
const char EventTypeStringrfId[] PROGMEM = "rfId";
const char EventTypeStringphotoRes[] PROGMEM = "photoRes";
const char EventTypeStringthermometer[] PROGMEM = "thermometer";
const char EventTypeStringpotentiometer[] PROGMEM = "potentiometer";
const char EventTypeStringmagnetSwitch[] PROGMEM = "magnetSwitch";
const char EventTypeStringdistanceDetector[] PROGMEM = "distanceDetector";
const char EventTypeStringstepperDone[] PROGMEM = "stepperDone";
const char EventTypeStringphoneET[] PROGMEM = "phoneET";
const char* const EventTypeString[] PROGMEM = {EventTypeStringnoneET,EventTypeStringbutton,EventTypeStringkeyPad,EventTypeStringrfId,EventTypeStringphotoRes,EventTypeStringpotentiometer,EventTypeStringpotentiometer, EventTypeStringmagnetSwitch,EventTypeStringdistanceDetector,EventTypeStringstepperDone,EventTypeStringphoneET};

enum Event
{
   noneE = 0,
   toggle,
   code,
   on,
   off,
   rangeTrigger,            // 
   stepperResult,            // I suppose, result of event is request complete 0, reset position -1, end position 1
   callOut,                    // outgoing call. (char*) data = phone number
   callReceived               // outcome of the call, (char*) data = yes or no
};
const char EventStringnoneET[] PROGMEM = "noneET";
const char EventStringtoggle[] PROGMEM = "toggle";
const char EventStringcode[] PROGMEM = "code";
const char EventStringon[] PROGMEM = "on";
const char EventStringoff[] PROGMEM = "off";
const char EventStringrangeTrigger[] PROGMEM = "rangeTrigger";
const char EventStringstepperResult[] PROGMEM = "stepperResult";
const char EventStringcallOut[] PROGMEM = "callOut";
const char EventStringcallReceived[] PROGMEM = "callReceived";
const char* const EventString[] PROGMEM = {EventStringnoneET,EventStringtoggle,EventStringcode,EventStringon,EventStringoff,EventStringrangeTrigger,EventStringstepperResult,EventStringcallOut,EventStringcallReceived};

struct Ack
{
  uint16_t chksum = 0;
  uint16_t msgId = 0;
  uint8_t fromId = 0;
  uint8_t toId = 0;  
};

struct Packet
{
  uint16_t chksum = 0;        // " + 6 + ": + 5 chars = 14
  uint16_t msgId = 0;         // " + 5 + ": + 5 chars = 13                                        if the left most bit is on, the message is an acknowledement.
  uint8_t fromId = 0;         // " + 6 + ": + 3 chars = 12
  uint8_t toId = 0;           // " + 5 + ": + 3 chars = 11                                       - 50 to here
  uint16_t wait = 0;          // " + 6 + ": + 5 chars = 14
  Action action;              // " + 6 + ":" + 14 chars = 24 or " + 6 + ":" + 5 chars = 15
  ActionType actionType;      // " + 10 + ":" + 14 chars = 28 or " + 10 + ":" + 6 chars = 20     - 52 or 35
  Event event;                // " + 5 + ":" + 13 chars = 22 or " + 5 + ":" + 5 chars = 14                        - which is either 85 or 86
  EventType eventType;        // " + 9 + ":" + 16 chars = 29 or " + 9 + ":" + 6 chars = 19       - 51 or 33   
  uint8_t data[14];           // either 14 chars or 36 digits
                              // { + 8, + [ + ] + }= 12
};

struct Message
{
  uint8_t sentTimes = 0;
  uint32_t lastSentTime = 0;
  Packet packet;
};

struct ReceivedMessage
{
  uint32_t receivedTime = 0;
  Packet packet;
};

//////////////////////////////////////
// Functions
/////////////////////////////////////
void loop() ;
void WriteData(Packet *packet);
bool ReadData();
char inString[150];
bool SerialReadEvent();
void MagneticSwtichEvent();
bool ProcessKeyPad();
bool ProcessTelephone();
void ButtonEvent();
  void FormatCardID(uint8_t *buffer, byte* uidByte, byte bufferSize);
    void ProcessCardReader();
    void ProcessRelayRequest(Packet *packet);
    void ProcessBuzzerRequest(Packet *packet);
    void ProcessTelephoneRequest(Packet *packet);
    void ProcessConfigureRangeRequest(Packet *packet);
    void ProcessMP3Request(Packet *packet);
    void ProcessCardReaderEvent(Packet *packet);
    void ProcessButtonEvent(Packet *packet);
    void ProcessKeyPadEvent(Packet *packet);
    void ProcessTelephoneEvent(Packet *packet);
    void ProcessPacket(Packet *packet);
#ifdef MASTER   
      void JsonToPacket(Packet *packet, char* json);
      char* PacketToJsonStr(Packet *packet);
#endif
    void check_radio(void) ;
    ActionType ActionTypeInt(char* actionType);
    Action ActionInt(char* action)  ;
    Event EventInt(char* event);
    EventType EventTypeInt(char *eventType);
    void ProcessDistanceSensor();
uint16_t calculateChecksum(Packet *packet);
uint8_t ackSentAlready(uint16_t msgId, uint8_t fromId);
bool resendPacketsNotAlreadyAck();
void printPacket(Packet *packet);
#endif

