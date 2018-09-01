#include "JOAT.h"
#include <SPI.h>


// Hardware configuration
//RF24 radio(RST_PIN, RF24_SS_PIN);             // create the radio instance on SPI bus plus pins 7 & 8
RF24 radio(RF24_SE_PIN, RF24_SS_PIN);             // create the radio instance on SPI bus plus pins 7 & 8

#ifdef RFID_CARD
//  MFRC522 mfrc522(RFID_SS_PIN, RST_PIN);        // Create the RFID MFRC522 instance.
//  MFRC522 mfrc522(RFID_SS_PIN, RFID_RS_PIN);        // Create the RFID MFRC522 instance.
  RFID mfrc522(RFID_SS_PIN, RFID_RS_PIN); 
#endif

#ifdef LCD_DEBUG
  LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16,2);
#endif

#if defined(KEY_PAD) || defined(TELEPHONE)
  Keypad keypad = Keypad(makeKeymap(keypadKeys), KEYPAD_ROW_PINS, KEYPAD_COL_PINS, KEYPAD_ROWS, KEYPAD_COLS);
#endif
                                        
// Use the same address for both devices
uint64_t readingPipe = MESSAGE_PIPE_BASE_ADDRESS + MY_DEVICE_ID;
#ifndef MASTER
  uint64_t writingPipe = MESSAGE_PIPE_BASE_ADDRESS + HUB_DEVICE_ID;
#endif

//debounce timer
unsigned long lastDebounceTime=millis();
unsigned long btnLastDebounceTime=millis();

//for testing count packets sent to received
long pSent = 0;
long pReceived = 0; 
long pAcked = 0;
long pResent = 0;
long pAborted = 0;

ReceivedMessage receivedMessage[MAX_RECEIVED_MESSAGES];
uint8_t receivedMessageIndex = 0;
//Packet receivedPacket;
Message sentMessage[MAX_MESSAGES];
uint8_t sentMessageHead = 0;
uint8_t lastSentMessageHead = 0;
Ack sentAck[MAX_ACKS];
uint8_t sentAckHead = 0;

long lastPrintStats ;
long lastSentPacket;
bool lcdLoaded = false;


/********************** Setup *********************/
void setup(){
  // Setup and configure rf radio
  radio.begin();
  radio.setChannel(108);
  radio.setAutoAck(false);
  //radio.setRetries(15,15);
  radio.enableDynamicPayloads();
  #ifndef MASTER
    radio.openWritingPipe(writingPipe);         // only open writing pipe if this device is not the MASTER.  Master will write to whichever device reqired.
  #endif        
  radio.openReadingPipe(1,readingPipe); 
  radio.setDataRate(RF24_2MBPS);
  radio.txDelay = 4000;
  radio.startListening();

  Serial.begin(BAUD_RATE);
  PRINT(F("My Device ID: "));
  PRINTLN(MY_DEVICE_ID); 
  PRINT(F("PA Level: "));
  PRINTLN(radio.getPALevel());
  PRINT(F("Packet Size: "));
  PRINTLN((long) sizeof(Packet));
  PRINT(F("Channel #: "));
  PRINTLN(radio.getChannel());
  
    
  #ifdef RFID_CARD
  // configure RFID Reader
     delay(500);
     PRINTLN(F("Using Card Reader Events"));
     //mfrc522.PCD_Init(); // Init MFRC522 card 
     //  SPI.begin(); 
     mfrc522.init();
  #endif

  //configure button
  #ifdef BUTTON
     PRINTLN("Using Button Events");
     pinMode(BTN_PIN0, INPUT_PULLUP);
     pinMode(BTN_PIN1, INPUT_PULLUP);
     pinMode(BTN_PIN2, INPUT_PULLUP);
     pinMode(BTN_PIN3, INPUT_PULLUP);
     pinMode(BTN_PIN4, INPUT_PULLUP);
     pinMode(BTN_PIN5, INPUT_PULLUP);
  #endif

  // configure magnetic switch
  #ifdef MAG_SWITCH
     PRINTLN("Using Mag Switch Events");
     pinMode(BTN_PIN0, INPUT_PULLUP);
     pinMode(BTN_PIN1, INPUT_PULLUP);
     pinMode(BTN_PIN2, INPUT_PULLUP);
     pinMode(BTN_PIN3, INPUT_PULLUP);
     pinMode(BTN_PIN4, INPUT_PULLUP);
     pinMode(BTN_PIN5, INPUT_PULLUP);
     pinMode(BTN_PIN6, INPUT_PULLUP);
     pinMode(BTN_PIN7, INPUT_PULLUP);
     pinMode(BTN_PIN8, INPUT_PULLUP);
     pinMode(BTN_PIN9, INPUT_PULLUP);
  #endif 
  
  //configure relay
  #ifdef RELAY
     PRINTLN(F("Using Relay Requests"));
     pinMode(REL_PIN1, INPUT_PULLUP);
     pinMode(REL_PIN1, OUTPUT);

     pinMode(REL_PIN2, INPUT_PULLUP);
     pinMode(REL_PIN2, OUTPUT);

     pinMode(REL_PIN3, INPUT_PULLUP);
     pinMode(REL_PIN3, OUTPUT);

     pinMode(REL_PIN4, INPUT_PULLUP);
     pinMode(REL_PIN4, OUTPUT);

     pinMode(REL_PIN5, INPUT_PULLUP);
     pinMode(REL_PIN5, OUTPUT);

     pinMode(REL_PIN6, INPUT_PULLUP);
     pinMode(REL_PIN6, OUTPUT);

     pinMode(REL_PIN7, INPUT_PULLUP);
     pinMode(REL_PIN7, OUTPUT);

     pinMode(REL_PIN8, INPUT_PULLUP);
     pinMode(REL_PIN8, OUTPUT);
  #endif

  // configure key pad
  #ifdef KEY_PAD
      PRINTLN(F("Using Keypad Events"));
      memset(keypad_digits, 0, sizeof(keypad_digits));
  #endif

  #ifdef TELEPHONE
      PRINTLN(F("Using Telephone Events"));
      memset(keypad_digits, 0, sizeof(keypad_digits));
      pinMode(HOOK_PIN, INPUT_PULLUP);
      pinMode(VOLUME_PIN, INPUT);
      MP3_Init();
      MP3_SetVolume(21);
      long poll_counter = millis();      
  #endif

  #ifdef BUZZER
      PRINTLN(F("Using Buzzer"));
      pinMode(BUZZER_PIN, OUTPUT);
  #endif

  #ifdef SONIC_RANGE
      PRINTLN(F("Using Distance Sensor"));
      pinMode(TRIGGER, OUTPUT);
      pinMode(ECHO, INPUT);
  #endif

  #ifdef MP3_PLAYER
     PRINTLN(F("Using MP3 Player"));
     MP3_Init();
     MP3_SetVolume(21);
  #endif

  #ifdef LCD_DEBUG
    PRINTLN(F("Using LCD Debugging"));
    lcd.backlight();
    lcd.init();
    lcd.clear();
  #endif
   
  #ifdef SERIAL_COMMS
     PRINTLN(F("Using Serial Port for Json Requests and Events"));
     Serial.println(F("{\"ready\":\"true\"}"));
  #endif

  PRINT("Packet Size: ");
  PRINTLN(sizeof(Packet));
  PRINT("Ack Size: ");
  PRINTLN(sizeof(Ack));
  
  // setup the packet data
  for (int i = 0; i < MAX_RECEIVED_MESSAGES; i++) memset(&receivedMessage[i], 0, sizeof(ReceivedMessage));
  for (int i = 0; i < MAX_MESSAGES; i++)  memset(&sentMessage[i], 0, sizeof(Message));
  for (int i = 0; i < MAX_ACKS; i++) memset(&sentAck[i], 0, sizeof(Ack));
  
  lastPrintStats = millis();
  lastSentPacket = millis();
}



/********************** Main Loop *********************/
void loop() {

    // Process a packet received from the radio
    if (ReadData())
    {
      PRINT(F("Received fromId: "));
      PRINTLN(receivedMessage[receivedMessageIndex].packet.fromId);
      if (ProcessAcknowledgement(&receivedMessage[receivedMessageIndex].packet))
      {
        pReceived++;    // count the message just received
        if (receivedMessage[receivedMessageIndex].packet.event != noneE || receivedMessage[receivedMessageIndex].packet.wait == 0) 
        {
          PRINTLN(F("Processing Event Request"));
          ProcessPacket(&receivedMessage[receivedMessageIndex].packet);
          receivedMessage[receivedMessageIndex].packet.toId = 0;
          memset(&receivedMessage[receivedMessageIndex].packet, 0, sizeof(Packet));
        }
      }
      else
      {
        receivedMessage[receivedMessageIndex].packet.toId = 0;
        memset(&receivedMessage[receivedMessageIndex].packet, 0, sizeof(Packet));
      }
    }
    else if (resendPacketsNotAlreadyAck())
    {
       return;
    }

    // Process any packets waiting to be processed.
    for (int i = 0; i < MAX_RECEIVED_MESSAGES; i++)
    {
      if (receivedMessage[i].packet.action != noneA && receivedMessage[i].receivedTime + (receivedMessage[i].packet.wait*100) <= millis()) 
      {
        ProcessPacket(&receivedMessage[i].packet);
        memset(&receivedMessage[i].packet, 0, sizeof(Packet));
        return;
      }
    }

    #ifdef LCD_DEBUG
      if (lcdLoaded && millis() - lastSentPacket > 5000)
      {
         lcd.clear();
         lastSentPacket = millis();
         lcdLoaded = false;
      }
    #endif
        
    /////////////////////////////
    // Process card reader Event
    /////////////////////////////
    #ifdef RFID_CARD
       if (sentMessage[sentMessageHead].packet.toId == 0) ProcessCardReader();
    #endif

    ///////////////////////
    // Process button event
    ///////////////////////
    #ifdef BUTTON
       if (sentMessage[sentMessageHead].packet.toId == 0) ButtonEvent();
    #endif

    ///////////////////////////////
    // Process Key Pad Events
    ///////////////////////////////
    #ifdef KEY_PAD
       if (sentMessage[sentMessageHead].packet.toId == 0) ProcessKeyPad();
    #endif

    ///////////////////////////////
    // Process Telephone Events
    ///////////////////////////////
    #ifdef TELEPHONE
       if (sentMessage[sentMessageHead].packet.toId == 0) ProcessTelephone();
    #endif

    #ifdef BUZZER
       
    #endif

    #ifdef MAG_SWITCH
       if (sentMessage[sentMessageHead].packet.toId == 0) MagneticSwtichEvent();
       
    #endif
    
    #ifdef SONIC_RANGE
      if (sentMessage[sentMessageHead].packet.toId == 0) ProcessDistanceSensor();
    #endif


    
    ///////////////////////
    // Process serial incomming data
    ///////////////////////
    #ifdef SERIAL_COMMS
         if (sentMessage[sentMessageHead].packet.toId == 0) 
         {
            if (SerialReadEvent())
            {
               Serial.println(F("{\"ready\":\"true\"}"));
               //Serial.flush();
            }
         }
    #endif
    
    if (sentMessage[sentMessageHead].packet.toId != 0)
    {
        // send the message
        PRINT(F("Sending ToId: "));
        PRINTLN(sentMessage[sentMessageHead].packet.toId);
        lastSentMessageHead = sentMessageHead;
        SendMessage(&sentMessage[sentMessageHead]);

        // prepare the next message
        sentMessageHead = (++sentMessageHead) % MAX_MESSAGES;
        memset(&sentMessage[sentMessageHead], 0, sizeof(Message));
        
        // send the message and 
        #ifdef RFID_CARD 
         //delay(100);
         //mfrc522.PCD_Init();    // Init MFRC522 card  
        #endif 
    }    
}

void SendMessage(Message *message)
{
    // if this packet has not already been sent, set the checksum and message id
    PRINTLN(F("Sending Message"));
    if (message->sentTimes == 0)
    {
      pSent++;
      // look at (uint16_t) millis() & MAX_MESSAGE_ID;
      message->packet.msgId = createMessageId();
      message->packet.chksum = calculateChecksum(((uint8_t*) &message->packet)+2, sizeof(Packet)-2);
      PRINTLN(F("Sending Packet: "));
      #ifdef DEV_DEBUG
        printPacket(&message->packet);
      #endif
      PRINTLN(spacer);
    }
    message->lastSentTime = millis();
    message->sentTimes++;

    #ifdef MASTER
      radio.openWritingPipe(MESSAGE_PIPE_BASE_ADDRESS + message->packet.toId);
    #endif  
    
    radio.stopListening();
    for(uint32_t i=0; i<130;i++){
       __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }    
    radio.write((uint8_t*) &(message->packet),sizeof(Packet),0);
    radio.startListening();
}

bool messageExistsAlready(uint16_t messageId)
{
    for (uint8_t i = 0; i < MAX_MESSAGES; i++)
    {
      // for messages that have been sent, check whether the messageId provided doesn't clash
      if (sentMessage[i].packet.msgId == messageId)
      {
        return true;
      }
    }
    return false;
}

uint16_t createMessageId()
{
    uint16_t messageId =  (uint16_t) millis() & MAX_MESSAGE_ID;
    while (messageExistsAlready(messageId))
    {
      PRINTLN(F("Message Exists Already"));
      messageId =  (uint16_t) millis() & MAX_MESSAGE_ID;
    }
    return messageId;
}

// send the given ack to caller.
void SendAck(Ack *ack)
{
    PRINT(F("Sending Ack MsgId "));
    PRINTLN(ack->msgId & 0x7FFF );

    #ifdef MASTER
      radio.openWritingPipe(MESSAGE_PIPE_BASE_ADDRESS + ack->toId);
    #endif
    pAcked++;
    radio.stopListening();
    for(uint32_t i=0; i<130;i++){
       __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }    
    radio.write((uint8_t*) ack,sizeof(Ack),0);
    radio.startListening();
}

bool resendPacketsNotAlreadyAck()
{
    uint32_t currentMs = millis();
    for (int i = 0; i < MAX_MESSAGES; i++)
    {
      // for messages that have been sent, check whether they need to be sent again
      if (sentMessage[i].sentTimes > 0)
      {
        if ((currentMs - sentMessage[i].lastSentTime) > ACK_MS_TIMEOUT)
        {
          if (sentMessage[i].sentTimes >= ACK_RETRIES)
          {
            PRINT(F("Max Ack Att Reached, aborting: "));
            PRINTLN(sentMessage[i].packet.msgId);
            memset(&sentMessage[i],0,sizeof(Message));
            pAborted++;
          }
          else
          {
            pResent++;
            SendMessage(&sentMessage[i]);
            return true;
          }
        }
      }
    }
    return false;
}
/*************************************************************** 
 * Serial Read Event from Pi
 ******************************************************** *******/
#ifdef SERIAL_COMMS
  bool SerialReadEvent(){
    int inStringLen = 0;
    inString[0] = '\0';
    char inChar = '\0';
    if (Serial.available()) {
      PRINTLN(F("Serial data available."));

      int timeoutCount = 5;
      while (timeoutCount > 0 && inChar != '}')
      {
        int dataVal = Serial.read();
        if (dataVal == -1)
        {
          delay(20);
          timeoutCount--;
        }
        else 
        {
          inChar = (char) dataVal;
          
          // add it to the inputString:
          if ((int) sizeof(inString) > (inStringLen + 1)) {
            inString[inStringLen] = inChar;
            inStringLen++;
          }        
        }
      }
    }

/*
    // Serial is USB and Serial1 is pins 0 or 1.
    while (Serial.available() && inChar != '}') {
      // get the new byte:
      inChar = (char)Serial.read();
      
      // add it to the inputString:
      if ((int) sizeof(inString) > (inStringLen + 1)) {
        inString[inStringLen] = inChar;
        inStringLen++;
      }
      //str = Serial.readStringUntil('}');
      delay(5);
    }
    */
    inString[inStringLen] = '\0';
    
    if (inStringLen <= 1) return false;
    if (inString[0] != '{' && inString[inStringLen-1] != '}') {
      PRINT(F("Incomplete Json: "));
      Serial.println(F("{\"ready\":\"true\"}"));
      PRINTLN(inString);
      return false;
    }
    
   // output the string recieved
    PRINT(F("Serial Received: "));
    PRINT(inStringLen);
    PRINTLN(F(" Bytes"));
    PRINTLN(spacer);
    //PRINTLN(inString);
    PRINTLN(spacer);

    // parse json to a packet.
    JsonToPacket(&sentMessage[sentMessageHead].packet, inString);
    sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
    
    #ifdef DEV_DEBUG
      printPacket(&sentMessage[sentMessageHead].packet);
    #endif
    #ifdef LCD_DEBUG
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("To-"));
      lcd.print(sentMessage[sentMessageHead].packet.toId);
      if (sentMessage[sentMessageHead].packet.action != noneA)
      {
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&ActionTypeString[sentMessage[sentMessageHead].packet.actionType]));
          lcd.print(enumBuff);
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&ActionString[sentMessage[sentMessageHead].packet.action]));
          lcd.print(enumBuff);
      }
      else if (sentMessage[sentMessageHead].packet.event != noneE)
      {
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&EventTypeString[sentMessage[sentMessageHead].packet.eventType]));
          lcd.print(enumBuff);
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&EventString[sentMessage[sentMessageHead].packet.event]));
          lcd.print(enumBuff);   
      }
    #endif
    return true;
  }
#endif

#ifdef MAG_SWITCH
  void MagneticSwtichEvent()
  {
    int button0 = digitalRead(BTN_PIN0);
    int button1 = digitalRead(BTN_PIN1);
    int button2 = digitalRead(BTN_PIN2);
    int button3 = digitalRead(BTN_PIN3);
    int button4 = digitalRead(BTN_PIN4);
    int button5 = digitalRead(BTN_PIN5);
    int button6 = digitalRead(BTN_PIN6);
    int button7 = digitalRead(BTN_PIN7);
    int button8 = digitalRead(BTN_PIN8);
    int button9 = digitalRead(BTN_PIN9);
    int switchNo = -1;

    // LOW is on
    if (button0 == LOW && !SWITCH_STATE[0] || button0 == HIGH && SWITCH_STATE[0])  switchNo = 0;
    else if(button1 == LOW && !SWITCH_STATE[1] || button1 == HIGH && SWITCH_STATE[1])  switchNo = 1;
    else if(button2 == LOW && !SWITCH_STATE[2] || button2 == HIGH && SWITCH_STATE[2])  switchNo = 2;
    else if(button3 == LOW && !SWITCH_STATE[3] || button3 == HIGH && SWITCH_STATE[3])  switchNo = 3;
    else if(button4 == LOW && !SWITCH_STATE[4] || button4 == HIGH && SWITCH_STATE[4])  switchNo = 4;
    else if(button5 == LOW && !SWITCH_STATE[5] || button5 == HIGH && SWITCH_STATE[5])  switchNo = 5;
    else if(button6 == LOW && !SWITCH_STATE[6] || button6 == HIGH && SWITCH_STATE[6])  switchNo = 6;
    else if(button7 == LOW && !SWITCH_STATE[7] || button7 == HIGH && SWITCH_STATE[7])  switchNo = 7;
    else if(button8 == LOW && !SWITCH_STATE[8] || button8 == HIGH && SWITCH_STATE[8])  switchNo = 8;
    else if(button9 == LOW && !SWITCH_STATE[9] || button9 == HIGH && SWITCH_STATE[9])  switchNo = 9;
      
    if (switchNo >= 0)
    {
        PRINTLN(F("Magnet Switch Event: "));

        SWITCH_STATE[0] = button0 == LOW;
        SWITCH_STATE[1] = button1 == LOW;
        SWITCH_STATE[2] = button2 == LOW;
        SWITCH_STATE[3] = button3 == LOW;
        SWITCH_STATE[4] = button4 == LOW;
        SWITCH_STATE[5] = button5 == LOW;
        SWITCH_STATE[6] = button6 == LOW;
        SWITCH_STATE[7] = button7 == LOW;
        SWITCH_STATE[8] = button8 == LOW;
        SWITCH_STATE[9] = button9 == LOW;
        //prepare packet for button
        sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
        sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
        sentMessage[sentMessageHead].packet.eventType = magnetSwitch;
        sentMessage[sentMessageHead].packet.event = toggle;
        if (button0 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[0] = 1;
        if (button1 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[1] = 1;
        if (button2 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[2] = 1;
        if (button3 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[3] = 1;
        if (button4 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[4] = 1;
        if (button5 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[5] = 1;
        if (button6 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[6] = 1;
        if (button7 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[7] = 1;
        if (button8 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[8] = 1;
        if (button9 == LOW) ((uint8_t*) sentMessage[sentMessageHead].packet.data)[9] = 1;
    }
  }
#endif

#ifdef KEY_PAD
  bool ProcessKeyPad()
  { 
      char key = keypad.getKey();
  
      if (key) {
        keypad_digits[digit_count] = key;
        PRINT("Key Pad digit buffer: ");
        PRINTLN((char*) keypad_digits);
        digit_count++;
      
        if (digit_count == KEYPAD_DIGITS) 
        {
           // package code and Key Pad event
          sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
          sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
          sentMessage[sentMessageHead].packet.event = code;
          sentMessage[sentMessageHead].packet.eventType = keyPad;   
          
          // null terminate the digit string
          ((uint8_t*) sentMessage[sentMessageHead].packet.data)[KEYPAD_DIGITS] = 0;
          strcpy((char*) sentMessage[sentMessageHead].packet.data, (const char*)keypad_digits);
          
          // clear the pad
          digit_count = 0;
          memset(keypad_digits, 0, sizeof(keypad_digits));
  
          #ifdef BUZZER
             tone(BUZZER_PIN, 1000);
             delay(100);
             noTone(BUZZER_PIN);
          #endif
        }
        else 
        {
          #ifdef BUZZER
            tone(BUZZER_PIN, 500);
            delay(100);
            noTone(BUZZER_PIN);
          #endif
        }
        return true;
      }
      return false;
  }
#endif

#ifdef TELEPHONE
  bool ProcessTelephone()
  {
    // only process hook and volumn every 200 ms
    if (poll_counter + POLL_MS < millis())
    {
      int pinRead = analogRead(VOLUME_PIN);
      pinRead = pinRead  / VOLUME_POT_UNIT;
      if (pinRead > 22) pinRead = 22;
      
      if (phone_volume != pinRead)
      {
        PRINT(F("Set Volume: "));
        Serial.println(pinRead);
        MP3_SetVolume((uint8_t) phone_volume);   
        phone_volume = pinRead;     
      }

      pinRead = digitalRead(HOOK_PIN);
      // if handset just hungup, clear everything
      if (pinRead == LOW && phone_state != ON_HOOK) {
        Serial.println("On Hook");
        // stop any audio
        MP3_Stop();
        // if phone is in outgoing or incomming call, check duration and send appropriate callReceived
        if ((phone_state & OUTGOING_CALL) == OUTGOING_CALL || (phone_state & INCOMMING_CALL) == INCOMMING_CALL)
        {
          sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
          sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
          sentMessage[sentMessageHead].packet.event = callReceived;
          sentMessage[sentMessageHead].packet.eventType = phoneET;  
          PRINT(F("call_start_time "));
          PRINTLN(call_start_time);
          PRINT(F("call_duration "));
          PRINTLN(call_duration);
          PRINT(F("millis "));
          PRINTLN(millis());
          PRINT(F("addition "));
          PRINTLN(call_start_time + call_duration);
          if (call_start_time > 0 && (call_start_time + call_duration) <= millis()) 
          {
            strcpy((char*) sentMessage[sentMessageHead].packet.data, "yes");        
          }
          else
          {
            strcpy((char*) sentMessage[sentMessageHead].packet.data, "no");  
          }
        }
        phone_state = ON_HOOK;
        
        // clear the phone number entered.
        digit_count = 0;
        call_start_time = 0;
        call_duration = 0;
        memset(keypad_digits, 0, sizeof(keypad_digits));        
      }
      // if handset just pickedup, and not an incomming call, play dialtone
      else if (pinRead == HIGH && (phone_state & ON_HOOK) == ON_HOOK) 
      {
        Serial.println("off hook");
        // if incomming call, play the mp3
        if ((phone_state & INCOMMING_CALL) == INCOMMING_CALL)
        {
            call_start_time = millis();
            MP3_PlayWithFolderFilename(1,mp3_to_play);
            phone_state = INCOMMING_CALL | OFF_HOOK | ANSWER;
        }
        else
        {
          phone_state = OFF_HOOK | OUTGOING_CALL;
          MP3_PlayWithFolderFilename(1,1);          // play dial tone
        }
      }
      poll_counter = millis();
    }  
       
      char key = keypad.getKey();
      if (key) {
        if (phone_state = (OFF_HOOK | OUTGOING_CALL))
        {
          keypad_digits[digit_count] = key;
          
          PRINT("Key Pad digit buffer: ");
          PRINTLN((char*) keypad_digits);
          digit_count++;
        
          if (digit_count == PHONE_DIGITS) 
          {
             // package code and Key Pad event
            sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
            sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
            sentMessage[sentMessageHead].packet.event = callOut;
            sentMessage[sentMessageHead].packet.eventType = phoneET;      
            memcpy((void*) sentMessage[sentMessageHead].packet.data, (void*)keypad_digits, sizeof(keypad_digits));
            // clear the pad
            digit_count = 0;
            memset(keypad_digits, 0, sizeof(keypad_digits));
  
            MP3_PlayWithFolderFilename(1,6);          // play ring tone
            phone_state = (OFF_HOOK | OUTGOING_CALL | RINGING);
          }
          else 
          {
            // play pulse dial sound on phone
            MP3_PlayWithFolderFilename(1,(uint8_t) key);
          }
          return true;
        }
      }
      return false;
  }
#endif

/*************************************************************** 
 * Button Event code
 ***************************************************************/
#ifdef BUTTON
void ButtonEvent()
{
    int button0 = digitalRead(BTN_PIN0);
    int button1 = digitalRead(BTN_PIN1);
    int button2 = digitalRead(BTN_PIN2);
    int button3 = digitalRead(BTN_PIN3);
    int button4 = digitalRead(BTN_PIN4);
    int button5 = digitalRead(BTN_PIN5);
    uint16_t buttonNo = 0;

    
    if (button0 == LOW || button1 == LOW || button2 == LOW || button3 == LOW || button4 == LOW || button5 == LOW){
      if ((millis()-btnLastDebounceTime) < 125)  //if 50 milliseconds has passed since last bounce
      {
           btnLastDebounceTime=millis();
           return;  //read value again now that bouncing is over
      }    
      btnLastDebounceTime=millis();
  
      PRINTLN("Button Event: ");

      //prepare packet for button
      sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
      sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
      sentMessage[sentMessageHead].packet.event = toggle;
      sentMessage[sentMessageHead].packet.eventType = button;
  
      //prepare message to be sent
      if(button0 == LOW) buttonNo = 0;
      if(button1 == LOW) buttonNo = 1;
      if(button2 == LOW) buttonNo = 2;
      if(button3 == LOW) buttonNo = 3;
      if(button4 == LOW) buttonNo = 4;
      if(button5 == LOW) buttonNo = 5;
     
      memcpy((void*) sentMessage[sentMessageHead].packet.data, (void*)&buttonNo, sizeof(uint16_t));
    }  
}
#endif

/*************************************************************** 
 * RFID Reader Event code
 ***************************************************************/
#ifdef RFID_CARD
    int serNum0;
    int serNum1;
    int serNum2;
    int serNum3;
    int serNum4;
    long clearRFIDCodeLapse = 0;
    long rfidReadLapse = 0;
    
  void FormatCardID(uint8_t *buffer, byte* uidByte, byte bufferSize) {
      for (byte i = 0; i < bufferSize; i++) {
          sprintf((char*) buffer+(i*2), "%0X", uidByte[i]);
      }
  }

  void ProcessCardReader()
  {
        if (mfrc522.isCard()) {
          if (rfidReadLapse + RFID_READ_DELAY > millis())
          {
            rfidReadLapse = millis();
            return;
          }
          rfidReadLapse = millis();
          if (clearRFIDCodeLapse + RFID_CARD_LAPSE_MS <= millis())
          {
            serNum0 = 0;
            serNum1 = 0;
            serNum2 = 0;
            serNum3 = 0;
            serNum4 = 0;
            clearRFIDCodeLapse = millis();
           }
          
          if (mfrc522.readCardSerial()) {
              if (mfrc522.serNum[0] != serNum0
                  && mfrc522.serNum[1] != serNum1
                  && mfrc522.serNum[2] != serNum2
                  && mfrc522.serNum[3] != serNum3
                  && mfrc522.serNum[4] != serNum4
              ) {
                  /* With a new cardnumber, show it. */
                  Serial.println(" ");
                  Serial.println("Card found");
                  serNum0 = mfrc522.serNum[0];
                  serNum1 = mfrc522.serNum[1];
                  serNum2 = mfrc522.serNum[2];
                  serNum3 = mfrc522.serNum[3];
                  serNum4 = mfrc522.serNum[4];
  
                  //Serial.println(" ");
                  Serial.println("Cardnumber:");
                  Serial.print("Dec: ");
                  Serial.print(mfrc522.serNum[0],DEC);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[1],DEC);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[2],DEC);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[3],DEC);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[4],DEC);
                  Serial.println(" ");
  
                  Serial.print("Hex: ");
                  Serial.print(mfrc522.serNum[0],HEX);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[1],HEX);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[2],HEX);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[3],HEX);
                  Serial.print(", ");
                  Serial.print(mfrc522.serNum[4],HEX);
                  Serial.println(" ");
  
                  sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
                  sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
                  sentMessage[sentMessageHead].packet.event = code;
                  sentMessage[sentMessageHead].packet.eventType = rfId;
                  FormatCardID((uint8_t*) sentMessage[sentMessageHead].packet.data, mfrc522.serNum, 5);
               } else {
                 /* If we have the same ID, just write a dot. */
                 Serial.print(".");
               }
          }
    }

    mfrc522.halt();
/*
      //SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE1));
      if ( ! mfrc522.PICC_IsNewCardPresent())
      {
          PRINTLN(F("no card present"));
          return;
      }
      // Select one of the cards
      if ( ! mfrc522.PICC_ReadCardSerial())
      {
          PRINTLN("Not Read Card Searial");   
          return;
      }
  
      if ((millis()-lastDebounceTime) < 300)  //if 300 milliseconds has passed since last bounce
      {  
           lastDebounceTime=millis();
           return;  //read value again now that bouncing is over
      }    
      lastDebounceTime=millis();
  
      PRINTLN(F("Card Reader Event: "));
      
      // send card Id via radio here
      sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
      sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
      sentMessage[sentMessageHead].packet.event = code;
      sentMessage[sentMessageHead].packet.eventType = rfId;
      FormatCardID((uint8_t*) sentMessage[sentMessageHead].packet.data, mfrc522.uid.uidByte, mfrc522.uid.size);
 /*     
      SPI.endTransaction();
      radio.begin();
      radio.setAutoAck(false);
      radio.enableDynamicPayloads();
      radio.openReadingPipe(1,readingPipe); 
      radio.setDataRate(RF24_2MBPS);
      radio.txDelay = 4000;
      radio.startListening();
 */
  }
#endif

/*************************************************************** 
 * Distance Sensor Event
 ***************************************************************/
#ifdef SONIC_RANGE
unsigned int distance = 0;
uint8_t distance_low_count = 0;

void ProcessDistanceSensor()
{
   if (distance_low_count < 100 && distance < 2) {
     distance_low_count++;
     digitalWrite(TRIGGER, HIGH );
     delayMicroseconds(50);
     digitalWrite(TRIGGER, LOW ); 
     distance = 2;
     return;
   }
   else if (distance_low_count < 100 && distance == 2)
   {
     distance_low_count++;
     return;
   }
   distance_low_count=0;
   
   digitalWrite(TRIGGER, LOW ); 
   delayMicroseconds(10);
   digitalWrite(TRIGGER, HIGH );
   delayMicroseconds(12);
   digitalWrite(TRIGGER, LOW ); 
   distance = pulseIn(ECHO, HIGH, 200000) / PER_CM;

   if (distance < 2) {
     delay(200);
     PRINTLN("Distance less than 2");
     return;
   }

   distanceFilter[distanceHead] = distance;
   distanceHead = (distanceHead+1) % FILTER_SIZE;
   if (distanceCount < FILTER_SIZE) distanceCount++;

   if (objectInRange && allOutRange(distanceFilter,distanceCount))
   {
      objectInRange = false;
      strcpy((char*) sentMessage[sentMessageHead].packet.data, "outside" );
      
   }
   else if (!objectInRange && allInRange(distanceFilter,distanceCount))
   { 
      objectInRange = true;
      strcpy((char*) sentMessage[sentMessageHead].packet.data, "inside" );
   }
   else return;

   PRINTLN("Distance Sensor Threshold exceeded Event: ");

    // send card Id via radio here
    sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
    sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
    sentMessage[sentMessageHead].packet.action = noneA;
    sentMessage[sentMessageHead].packet.actionType = noneAT;
    sentMessage[sentMessageHead].packet.event = rangeTrigger;
    sentMessage[sentMessageHead].packet.eventType = distanceDetector;
}

bool allInRange(long filter[], uint8_t count) 
{
    for(int i = 0; i < count; i++)
    {
        if (filter[i] < closeRangeThreshold || filter[i] > farRangeThreshold) return false;
    }
    return true;
}
bool allOutRange(long filter[], uint8_t count)
{
    for(int i = 0; i < count; i++)
    {
        if (filter[i] >= closeRangeThreshold && filter[i] <= farRangeThreshold) return false;
    }
    return true;
}
#endif

/*************************************************************************
 * Radio Processing Events and Request Code here
 *************************************************************************/
void ProcessRelayRequest(Packet *packet)
{
  #ifdef RELAY
   // received info from Master Arduino.
    PRINTLN(F("Relay Request: "));
    
    // &packet->data has the pin to switch
   uint8_t pinNo = *(uint16_t*) packet->data;
   // packet->action = either start or stop
   // temp - turn on relay 1.
   if (packet->action == toggleA) {
    if(digitalRead(pinNo) == HIGH){
      digitalWrite(pinNo, LOW); 
      delay(100);
      digitalWrite(pinNo, HIGH);
    }
    else{
      digitalWrite(pinNo, HIGH);
      delay(100);
      digitalWrite(pinNo, LOW);
    }
    
   }
   else if (packet->action == stop) {
      PRINTLN(F("On"));
      digitalWrite(pinNo, HIGH); 
   } else if (packet->action == start) {
      PRINTLN(F("Off"));
      digitalWrite(pinNo, LOW); 
   }     

  #endif
}

void ProcessBuzzerRequest(Packet *packet)
{
#ifdef BUZZER  
    PRINTLN("Buzzer Request: ");

    tone(6,700);
    delay(200);
    noTone(6);
    
    uint8_t soundNo = *(unsigned int*) packet->data;
    PRINT(F("Request No: "));
    PRINTLN(soundNo);
    
    if (soundNo > 0 && soundNo <= 2)
    {    
        PRINTLN(F("Playing Buzzer"));
        const uint16_t *melody = BUZZER_SOUNDS[soundNo-1];
        for (int i = 0; i < 4; i+=2)
        {
          PRINT(melody[i]);
          PRINT(", ");
          PRINTLN(melody[i+1]);
           tone(BUZZER_PIN, melody[i]);
           delay(melody[i+1]);
        }
        noTone(BUZZER_PIN);
        PRINTLN(F("Done"));
    }
#endif 
}


void ProcessTelephoneRequest(Packet *packet)
{
  #ifdef TELEPHONE
    PRINTLN("Telephone Request: ");
  
    // if action is incomming call, inCommingCall
    if (packet->action == inCommingCall)
    {
        // phone is off the hook so cant receive the incomming call
       if (phone_state != ON_HOOK) 
       {  
          // error, phone is not on hook to receive the incomming call.
          // send message not received
          sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
          sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
          sentMessage[sentMessageHead].packet.event = callReceived;
          sentMessage[sentMessageHead].packet.eventType = phoneET;  
          strcpy((char*) sentMessage[sentMessageHead].packet.data, "no");
          return;
       }
       else
       {
          // set the phone state to ringing
          phone_state = ON_HOOK | INCOMMING_CALL;
          // play the ring mp3
          MP3_PlayWithFolderFilename(1, ((uint8_t*) packet->data)[0]);
          mp3_to_play = ((uint8_t*) packet->data)[0];
          call_duration = ((uint8_t*) packet->data)[1] * 1000; // convert to ms
          call_start_time  = 0;
       }
    }
    // else if action is outGoingCall
    else if (packet->action == outGoingCall)
    {
        // they've hung up before getting this message.
        if (phone_state == ON_HOOK)
        {
            // error, phone is on the hook so outgoing call not receieved
            // send message not received.
            sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
            sentMessage[sentMessageHead].packet.toId = HUB_DEVICE_ID;
            sentMessage[sentMessageHead].packet.event = callReceived;
            sentMessage[sentMessageHead].packet.eventType = phoneET;  
            strcpy((char*) sentMessage[sentMessageHead].packet.data, "no");
            return;
        }
        else
        {
            PRINTLN(F("change state to:  OFF_HOOK | OUTGOING_CALL | ANSWER"));
            // set the phone state to
            phone_state = OFF_HOOK | OUTGOING_CALL | ANSWER;
            mp3_to_play = ((uint8_t*) packet->data)[0];
            call_duration = ((uint8_t*) packet->data)[1] * 1000;   // convert to ms
            call_start_time = millis();
            // play the mp3
            PRINT(F("Playing: "));
            PRINTLN(mp3_to_play);
            MP3_PlayWithFolderFilename(1, mp3_to_play);
        }
    }
   
  #endif
}


void ProcessConfigureRangeRequest(Packet *packet)
{
  #ifdef SONIC_RANGE
    closeRangeThreshold = ((int*) packet->data)[0];
    farRangeThreshold = ((int*) packet->data)[1];
    PRINT(F("close range set : "));
    PRINTLN(closeRangeThreshold);
    PRINT(F("far range set: "));
    PRINTLN(farRangeThreshold);
  #endif
}

void ProcessMP3Request(Packet *packet)
{
  #ifdef MP3_PLAYER
    PRINTLN(F("MP3 Request: "));
    #ifdef DEV_DEBUG
      printPacket(packet);
    #endif 
    
    uint16_t data = ((uint8_t*) packet->data)[0];
    PRINT(F("Data is: "));
    PRINTLN(data);
    if (packet->action == play)
    {
        PRINTLN(F("Play Index: "));
        MP3_PlayWithFolderFilename(1,data);
    }
    else if (packet->action == volume)
    {
        PRINTLN(F("Set Volume: "));
        MP3_SetVolume((uint8_t) data);
    }
    else if (packet->action == stop)
    {
        PRINTLN(F("Stopping: "));
        MP3_Stop();
    }
  #endif
}

////////////////////////////////////////////
// Events comming into Master
////////////////////////////////////////////
void ProcessCardReaderEvent(Packet *packet)
{  

    PRINTLN(F("Card Reader Tagged: "));

    // send info to Raspberry Pi here.
    SendToPie(packet);

    // write to lcd   
    WriteLCDDebugMsgReceived(packet);
}

void ProcessButtonEvent(Packet *packet)
{
   // send info to Raspberry Pi here.
    PRINTLN(F("Button toggled: "));
    
    // send info to Raspberry Pi here.
    SendToPie(packet);

    // write to lcd   
    WriteLCDDebugMsgReceived(packet);
}

void ProcessKeyPadEvent(Packet *packet)
{
   // send info to Raspberry Pi here.
    PRINTLN(F("Key Pad Code: "));


    // send info to Raspberry Pi here.
    SendToPie(packet);

    // write to lcd   
    WriteLCDDebugMsgReceived(packet);}

void ProcessDistanceDetectorEvent(Packet *packet)
{
    PRINTLN(F("Distance Threshold smashed: "));


    // send info to Raspberry Pi here.
    SendToPie(packet);

    // write to lcd   
    WriteLCDDebugMsgReceived(packet);}

void ProcessMagneticSwitchEvent(Packet *packet)
{
    PRINTLN(F("Magnetic Switch: "));

    // send info to Raspberry Pi here.
    SendToPie(packet);

    // write to lcd   
    WriteLCDDebugMsgReceived(packet);
}

void ProcessTelephoneEvent(Packet *packet)
{
    PRINTLN(F("Telephone: "));    

    if (packet->event == callOut)
    {
      // todo: if outgoing call, send action to play outgoing call mp3
      sentMessage[sentMessageHead].packet.fromId = MY_DEVICE_ID;
      sentMessage[sentMessageHead].packet.toId = packet->fromId;
      sentMessage[sentMessageHead].packet.wait = packet->wait = 30;
      sentMessage[sentMessageHead].packet.action = outGoingCall;
      sentMessage[sentMessageHead].packet.actionType = phoneAT; 
      ((uint8_t*) sentMessage[sentMessageHead].packet.data)[0] = 9;
      ((uint8_t*) sentMessage[sentMessageHead].packet.data)[1] = 3;
      
    }       
    SendToPie(packet);

    WriteLCDDebugMsgReceived(packet);
}

void WriteLCDDebugMsgReceived(Packet *packet)
{
    #ifdef LCD_DEBUG
      lcd.setCursor(0,1);
      lcd.print(F("msg from - "));
      lcd.print(packet->fromId);
      if (packet->action != noneA)
      {
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&ActionTypeString[sentMessage[sentMessageHead].packet.actionType]));
          lcd.print(ActionTypeString[packet->actionType]);
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&ActionString[sentMessage[sentMessageHead].packet.action]));
          lcd.print(enumBuff);
      }
      else if (packet->event != noneE)
      {
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&EventTypeString[sentMessage[sentMessageHead].packet.eventType]));
          lcd.print(enumBuff);
          lcd.print(" ");
          strcpy_P(enumBuff, (const char*) pgm_read_word(&EventString[sentMessage[sentMessageHead].packet.event]));
          lcd.print(enumBuff);   
      }
    #endif   
}
void WriteLCDDebugMsgAck(Ack *ack)
{
    #ifdef LCD_DEBUG
      lcd.setCursor(0,1);
      lcd.print(F("ack from - "));
      lcd.print(ack->fromId);
      lastSentPacket = millis();
      lcdLoaded = true;
    #endif   
}
void SendToPie(Packet *packet)
{
#ifdef MASTER   
      PRINTLN(F("Convert Packet to Json string"));
      char * jsont = PacketToJsonStr(packet);
      Serial.println(jsont);
#endif  
}

bool ProcessAcknowledgement(Packet *packet)
{
    PRINTLN(spacer);
    PRINTLN(F("Processing Packet Acknowledement"));
    // if I have seen this packet before, resend ack and ignore it;
    uint8_t sentAckIndex = ackSentAlready(packet->msgId, packet->fromId);
    if (sentAckIndex != 255)
    {
      PRINT(F("I already Acknowledged this, resending"));
      PRINTLN(sentAckIndex);
      SendAck(&sentAck[sentAckIndex]);
      memset(packet, 0, sizeof(Packet));
      return false;
    }

    // create the ack and send it
    sentAckHead = (++sentAckHead) % MAX_ACKS;

    // calculate the checksum, set the ack bit and swap the to and from ids
    sentAck[sentAckHead].msgId =  packet->msgId | 0x8000;
    sentAck[sentAckHead].toId = packet->fromId;
    sentAck[sentAckHead].fromId = MY_DEVICE_ID;
    sentAck[sentAckHead].chksum =  calculateChecksum(((uint8_t*)&sentAck[sentAckHead])+2, sizeof(Ack)-2);
    SendAck(&sentAck[sentAckHead]);
    return true;
}

void ProcessPacket(Packet *packet)
{
    if (packet->action != noneA)
    {
       switch (packet->actionType)
       {
         case laser:
         break;
         case relay:
            ProcessRelayRequest(packet);
            break;
         case buzzer:
            ProcessBuzzerRequest(packet);
            break;
         case mp3:
            ProcessMP3Request(packet);
            break;
         case phoneAT:
            ProcessTelephoneRequest(packet);
            break;
         case distanceSensor:
            ProcessConfigureRangeRequest(packet);
            break;
         default:
         break;
       }
       memset(packet, 0, sizeof(Packet));
    }

    if (packet->event != noneE)
    {
       switch (packet->eventType)
       {
         case button:
            ProcessButtonEvent(packet);
            break;
         case keyPad:
            ProcessKeyPadEvent(packet);
            break;
         case rfId:
            ProcessCardReaderEvent(packet);
            break;
         case photoRes:
            break;
         case phoneET:
            ProcessTelephoneEvent(packet);
            break;
         case thermometer:
            break;
         case potentiometer:
            break;
         case magnetSwitch:
            ProcessMagneticSwitchEvent(packet);
            break;
         default:
            break;
       }
       memset(packet, 0, sizeof(Packet));
    }    
    
}

#ifdef MASTER
char jsonBufferStr[128];

char* PacketToJsonStr(Packet *packet)
{
    int nChars = 0;
    jsonBufferStr[0] = '{';
    jsonBufferStr[1] = '\0';
    strcat(jsonBufferStr, (const char*) "\"msgId\":");
    sprintf(jsonBufferStr+strlen(jsonBufferStr), "%u", packet->msgId);
    strcat(jsonBufferStr, (const char*) ",\"chksum\":");
    sprintf(jsonBufferStr+strlen(jsonBufferStr), "%u", packet->chksum);
    strcat(jsonBufferStr, (const char*) ",\"fromId\":");
    sprintf(jsonBufferStr+strlen(jsonBufferStr), "%u", packet->fromId);
    strcat(jsonBufferStr, (const char*) ",\"toId\":");
    sprintf(jsonBufferStr+strlen(jsonBufferStr), "%u", packet->toId);
    strcat(jsonBufferStr, (const char*) ",\"wait\":");
    sprintf(jsonBufferStr+strlen(jsonBufferStr), "%u", packet->wait);
    strcat(jsonBufferStr, (const char*) ",\"action\":\"");
    strcat_P(jsonBufferStr, (const char*) pgm_read_word(&ActionString[(uint8_t) packet->action]));
    strcat(jsonBufferStr, (const char*) "\",\"actionType\":\"");
    strcat_P(jsonBufferStr, (const char*) pgm_read_word(&ActionTypeString[(uint8_t) packet->actionType]));
    strcat(jsonBufferStr, (const char*) "\",\"event\":\"");
    strcat_P(jsonBufferStr, (const char*) pgm_read_word(&EventString[(uint8_t) packet->event]));
    strcat(jsonBufferStr, (const char*) "\",\"eventType\":\"");
    strcat_P(jsonBufferStr, (const char*) pgm_read_word(&EventTypeString[(uint8_t) packet->eventType]));
    strcat(jsonBufferStr, (const char*) "\",\"data\":");
    nChars = strlen(jsonBufferStr);
    
    if (packet->eventType != noneET)
    {
         if (packet->eventType == button) {
            nChars += sprintf(jsonBufferStr+nChars, "%u", ((uint16_t*) packet->data)[0]);
         }
         else if (packet->eventType == magnetSwitch)
         {
            nChars += sprintf(jsonBufferStr+nChars, "[%u", ((uint8_t*) packet->data)[0]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[1]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[2]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[3]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[4]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[5]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[6]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[7]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u", ((uint8_t*) packet->data)[8]);
            nChars += sprintf(jsonBufferStr+nChars, ",%u]", ((uint8_t*) packet->data)[9]);
         }
         else {
            //strcpy(jsonBufferStr+nChars, (const char*) packet->data);
            nChars += sprintf(jsonBufferStr+nChars, "\"%s\"", (char*) packet->data);
            //nChars = strlen(jsonBufferStr);
         }
    } else if (packet->actionType != noneAT)
    {
        if (packet->actionType == relay || packet->actionType == buzzer || packet->actionType == mp3) {
            nChars += sprintf(jsonBufferStr+nChars, "%u}", ((uint16_t*) packet->data)[0]);
        }
        else if (packet->actionType == distanceSensor || packet->actionType == stepperMotor) {
            nChars += sprintf(jsonBufferStr+nChars, "[%u,", ((uint16_t*) packet->data)[0]);
            nChars += sprintf(jsonBufferStr+nChars, "%u]", ((uint16_t*) packet->data)[1]);
        }
        else if (packet->actionType == stepperMotor) {
            nChars += sprintf(jsonBufferStr+nChars, "[%u,", ((uint16_t*) packet->data)[0]);
            nChars += sprintf(jsonBufferStr+nChars, "%u,", ((uint16_t*) packet->data)[1]);
            nChars += sprintf(jsonBufferStr+nChars, "%u,", ((uint16_t*) packet->data)[2]);
            nChars += sprintf(jsonBufferStr+nChars, "%u]", ((uint16_t*) packet->data)[3]);
        }
        else if (packet->actionType == phoneAT)
        {
            nChars += sprintf(jsonBufferStr+nChars, "[%u,", ((uint8_t*) packet->data)[0]);
            nChars += sprintf(jsonBufferStr+nChars, "%u]", ((uint8_t*) packet->data)[1]);
        }
        else {
          //strcpy(jsonBufferStr+nChars, (const char*) packet->data);
           nChars += sprintf(jsonBufferStr+nChars, "\"%s\"", (char*) packet->data);
          //nChars = strlen(jsonBufferStr);
        }
    }
    jsonBufferStr[nChars] = '}';
    jsonBufferStr[nChars+1] = '\0';
    return jsonBufferStr;
}

//{"msgId":xxxx,"chksum":xxxx,"fromId":20,"toId":255,"action":"noneA","actionType":"noneAT","event":"code","eventType":"keyPad","data":"123"}
void JsonToPacket(Packet *packet, char* jsonString)
{
    memset(packet, 0, sizeof(Packet));
    char *field = strtok(jsonString," \":{,}\n");  
    while (field != 0)
    {
       if (!ParseJsonField(packet, field)) return;
       field = strtok(0, " \":{,}\n");
    }
}

bool ParseJsonField(Packet *packet, char *field)
{
    char *value = strtok(0, " \":{,}[\n");     // "fromId":20
    if (value == 0) return false;
    
    if (strcmp(field, "fromId") == 0) packet->fromId = atoi(value);
    else if (strcmp(field, "toId") == 0) packet->toId = atoi(value);
    else if (strcmp(field, "msgId") == 0) packet->msgId = atoi(value);
    else if (strcmp(field, "chksum") == 0) packet->chksum = atoi(value);
    else if (strcmp(field, "wait") == 0) packet->wait = atoi(value);
    else if (strcmp(field, "action") == 0)  packet->action = ActionInt(value);
    else if (strcmp(field, "actionType") == 0) packet->actionType = ActionTypeInt(value);
    else if (strcmp(field, "event") == 0) packet->event = EventInt(value);
    else if (strcmp(field, "eventType") == 0) packet->eventType = EventTypeInt(value);
    else if (strcmp(field, "data") == 0)
    {
        PRINT(F("Value is: "));
        PRINTLN(value);

        if (packet->eventType != noneET)
        {
            if (packet->eventType == button )
            {
                ((uint16_t*) packet->data)[0] = atoi(value);
            }
            else if (packet->eventType == magnetSwitch)
            {
                ((uint8_t*) packet->data)[0] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[1] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[2] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[3] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[4] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[5] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[6] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[7] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[8] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[9] = atoi(value);         
            }
            else {
              strcpy((char*)packet->data, (char*) value);
            }
        }
        else if (packet->actionType != noneAT)
        {
            if (packet->actionType == relay || packet->actionType == buzzer || packet->actionType == mp3)
            {
                ((uint16_t*) packet->data)[0] = atoi(value);
            }
            else if (packet->actionType == distanceSensor || packet->actionType == stepperMotor)
            {
                ((uint16_t*) packet->data)[0] = atoi(value);
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint16_t*) packet->data)[1] = atoi(value);

                if (packet->actionType == stepperMotor)
                {
                  value = strtok(0, " ,]}\n"); 
                  if (value == 0) return false;
                  ((uint16_t*) packet->data)[2] = atoi(value);
                  value = strtok(0, " ,]}\n"); 
                  if (value == 0) return false;
                  ((uint16_t*) packet->data)[3] = atoi(value);                               
                }
            }
            else if (packet->actionType = phoneAT)
            {
                ((uint8_t*) packet->data)[0] = atoi(value);   // mp3 track to play
                value = strtok(0, " ,]}\n"); 
                if (value == 0) return false;
                ((uint8_t*) packet->data)[1] = atoi(value);   // length of track in seconds
            }
            else {
                strcpy((char*)packet->data, (char*) value);
            }
        }  
    }
    return true;
}

#endif

bool ReadData()
{
  uint8_t payloadSize = 0;
  if (radio.available())
  {
    PRINTLN(F("Received Packet on radio"));
    payloadSize = radio.getDynamicPayloadSize() ;
    if(payloadSize < sizeof(Ack))
    {
      PRINTLN(F("Payload too small"));
      return false; 
    }
    receivedMessageIndex = findAvailableReceivedMessage();
    
    // Read in the data
    if (payloadSize == sizeof(Ack))
    {
      // message is an acknowledment
      radio.read((void*)&receivedMessage[receivedMessageIndex].packet,sizeof(Ack));
      if (receivedMessage[receivedMessageIndex].packet.msgId & 0x8000)
      {
        uint16_t chksum =  calculateChecksum(((uint8_t*)&receivedMessage[receivedMessageIndex].packet)+2, sizeof(Ack)-2);
        if (chksum != receivedMessage[receivedMessageIndex].packet.chksum || receivedMessage[receivedMessageIndex].packet.toId != MY_DEVICE_ID)
        {
          memset(&receivedMessage[receivedMessageIndex].packet, 0, sizeof(Packet));
          return false;
        }
  
        // clear acknowledged message by search through sentMessages
        for (int i = 0; i < MAX_MESSAGES; i++)
        {
          if ((sentMessage[i].packet.msgId == (receivedMessage[receivedMessageIndex].packet.msgId & 0x7FFF)) && (sentMessage[i].packet.toId == receivedMessage[receivedMessageIndex].packet.fromId))
          {
            WriteLCDDebugMsgAck((Ack*) &receivedMessage[receivedMessageIndex].packet);
            memset(&sentMessage[i],0,sizeof(Message));
            memset(&receivedMessage[receivedMessageIndex].packet, 0, sizeof(Packet));
            return false;
          }
        }
      } else {
        PRINTLN(F("Expected Ack Payload"));
      }
      // clear the receieved ack/packet
      memset(&receivedMessage[receivedMessageIndex].packet, 0, sizeof(Packet));
      return false;
    }
    else if (payloadSize == sizeof(Packet))
    {
      radio.read((void*)&receivedMessage[receivedMessageIndex].packet, sizeof(Packet));
      uint16_t chksum =  calculateChecksum(((uint8_t*)&receivedMessage[receivedMessageIndex].packet)+2, sizeof(Packet)-2);
      // if this packet is corrupt or not for me, ignore
      if (chksum != receivedMessage[receivedMessageIndex].packet.chksum || receivedMessage[receivedMessageIndex].packet.toId != MY_DEVICE_ID)
      {
        PRINTLN(F("Checksum missmatch, packet corrupt?"));
        memset(&receivedMessage[receivedMessageIndex].packet, 0, sizeof(Packet));
        return false;
      }
      receivedMessage[receivedMessageIndex].receivedTime = millis();
      printPacket(&receivedMessage[receivedMessageIndex].packet);
      PRINTLN(spacer);  
      return true;
    } else {
      PRINTLN(F("Expected Packet Payload"));
    }

    // Print the packet just recieved.
    PRINTLN(spacer);  
   return false;
  }
  return false;
}

ActionType ActionTypeInt(char* actionType)
{
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[0])) == 0) return noneAT;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[1])) == 0) return laser;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[2])) == 0) return relay;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[3])) == 0) return buzzer;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[4])) == 0) return distanceSensor;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[5])) == 0) return stepperMotor;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[6])) == 0) return mp3;
   if (strcasecmp_P(actionType, (const char*) pgm_read_word(&ActionTypeString[7])) == 0) return phoneAT;
   return noneAT;
}

Action ActionInt(char* action)
{
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[0])) == 0) return noneA;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[1])) == 0) return start;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[2])) == 0) return stop;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[3])) == 0) return configureRange;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[4])) == 0) return stepAction;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[5])) == 0) return play;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[6])) == 0) return volume;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[7])) == 0) return inCommingCall;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[8])) == 0) return outGoingCall;
   if (strcasecmp_P(action, (const char*) pgm_read_word(&ActionString[9])) == 0) return toggleA;
   return noneA;
}
EventType EventTypeInt(char *eventType)
{
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[0])) == 0) return noneET;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[1])) == 0) return button;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[2])) == 0) return keyPad;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[3])) == 0) return rfId;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[4])) == 0) return photoRes;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[5])) == 0) return thermometer;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[6])) == 0) return potentiometer;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[7])) == 0) return magnetSwitch;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[8])) == 0) return distanceDetector;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[9])) == 0) return stepperDone;
   if (strcasecmp_P(eventType, (const char*) pgm_read_word(&EventTypeString[10])) == 0) return phoneET;
   return  noneET;
}


Event EventInt(char* event)
{
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[0])) == 0) return noneE;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[1])) == 0) return toggle;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[2])) == 0) return code;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[3])) == 0) return on;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[4])) == 0) return off;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[5])) == 0) return rangeTrigger;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[6])) == 0) return stepperResult;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[7])) == 0) return callOut;
   if (strcasecmp_P(event, (const char*) pgm_read_word(&EventString[8])) == 0) return callReceived;
   return noneE;
}

uint16_t calculateChecksum(uint8_t *packet, uint8_t cnt)
{
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;

   for(int index = 0; index < cnt; index++ )
   {
      sum1 = (sum1 + packet[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

uint8_t ackSentAlready(uint16_t msgId, uint8_t fromId)
{
  for (int i = 0; i < MAX_ACKS; i++)
  { 
    // note the recorded Ack already has the ack bit set so compare with message id
    if (sentAck[i].msgId != 0 && sentAck[i].msgId == (msgId | 0x8000) && sentAck[i].toId == fromId) {
      return  i;
    }
  }
  return 255;
}

// if no buffer available, return the first one.
uint8_t findAvailableReceivedMessage()
{
  for (int i = 0; i < MAX_RECEIVED_MESSAGES; i++)
  {
    if (receivedMessage[i].packet.msgId == 0) {
      PRINT(F("available message buffer: "));
      PRINTLN(i);
      return i;
    }
  }
  PRINT(F("available message buffer: zero"));
  return 0;
}

void printPacket(Packet *packet)
{
    Serial.print(F("msgId: "));
    Serial.print(packet->msgId);
    Serial.print(F(", chksum: "));
    Serial.print(packet->chksum);
    Serial.print(F(", fromId: "));
    Serial.print(packet->fromId);
    Serial.print(F(", toId: "));
    Serial.print(packet->toId);
    Serial.print(F(", wait: "));
    Serial.print(packet->wait);
    Serial.print(F(", action: "));
    strcpy_P(enumBuff, (const char*) pgm_read_word(&ActionString[(int)packet->action]));
    Serial.print(enumBuff);
    Serial.print(F(", actionType: "));
    strcpy_P(enumBuff, (const char*) pgm_read_word(&ActionTypeString[(int)packet->actionType]));
    Serial.print(enumBuff);
    Serial.print(F(", event: "));
    strcpy_P(enumBuff, (const char*) pgm_read_word(&EventString[(int)packet->event]));
    Serial.print(enumBuff);
    Serial.print(F(", eventType: "));
    strcpy_P(enumBuff, (const char*) pgm_read_word(&EventTypeString[(int)packet->eventType]));
    Serial.print(enumBuff);
    Serial.print(F(", data: "));
    
    if (packet->eventType == button)
    {
        Serial.print(((uint16_t*)packet->data)[0]);
    }
    else if (packet->eventType == magnetSwitch)
    {
        Serial.print(((uint8_t*)packet->data)[0]);
        Serial.print(((uint8_t*)packet->data)[1]);
        Serial.print(((uint8_t*)packet->data)[2]);
        Serial.print(((uint8_t*)packet->data)[3]);
        Serial.print(((uint8_t*)packet->data)[4]);
        Serial.print(((uint8_t*)packet->data)[5]);
        Serial.print(((uint8_t*)packet->data)[6]);
        Serial.print(((uint8_t*)packet->data)[7]);
        Serial.print(((uint8_t*)packet->data)[8]);
        Serial.print(((uint8_t*)packet->data)[9]);
    }
    else if (packet->actionType == relay || packet->actionType == buzzer || packet->actionType == mp3)
    {
        Serial.print(((uint16_t*)packet->data)[0]);
    }
    else if (packet->actionType == distanceSensor || packet->actionType == stepperMotor)
    {
        Serial.print(((uint16_t*)packet->data)[0]);
        Serial.print(F(","));
        Serial.print(((uint16_t*)packet->data)[1]);      
        if (packet->actionType == stepperMotor)
        {
          Serial.print(F(","));
          Serial.print(((uint16_t*)packet->data)[2]);             
          Serial.print(F(","));
          Serial.print(((uint16_t*)packet->data)[3]);    
        }
    }
    else if (packet->actionType == phoneAT)
    {
        Serial.print(((uint8_t*)packet->data)[0]);
        Serial.print(F(","));
        Serial.print(((uint8_t*)packet->data)[1]);      
    }
    else 
    {
         Serial.print((char*) packet->data); 
    }
    Serial.println();
}



