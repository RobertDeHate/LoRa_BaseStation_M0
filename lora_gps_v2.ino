// LoRa GPS
// -*- mode: C++ -*-
//
// v1.2 - Changed the decode of $GNGGA sentences
// V2 added remote control to change frequency and increased channels to 15

#include <SPI.h>
#include <RH_RF95.h>
#include <EEPROM.h>

// for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

//#define SERIAL_CONSOLE 1

// Frequency Channels:  First channel of each subband for North America
#define RF95_FREQ_0 902.3
#define RF95_FREQ_1 903.9
#define RF95_FREQ_2 905.5
#define RF95_FREQ_3 907.1
#define RF95_FREQ_4 908.7
#define RF95_FREQ_5 910.3
#define RF95_FREQ_6 911.9
#define RF95_FREQ_7 913.5
#define RF95_FREQ_8 903.1
#define RF95_FREQ_9 904.7
#define RF95_FREQ_10 906.3
#define RF95_FREQ_11 907.9
#define RF95_FREQ_12 909.5
#define RF95_FREQ_13 911.1
#define RF95_FREQ_14 912.7

float RF95_FREQ;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Reserve a portion of flash memory 
int FLASHchannel = 0;

// Blinky on receipt
#define LED 13

int gpsBytes;
int radio_channel;

void setup(){
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  /* Set up HW Serial port for GPS */
  Serial1.begin(9600);
  Serial1.setTimeout(500);

  #ifdef SERIAL_CONSOLE
    Serial.begin(115200);
    while (!Serial) {
      delay(1);
    }
    delay(100);
    Serial.println("LoRa TX Test!");
  #endif

  // Get radio_channel from flash
  radio_channel = EEPROM[FLASHchannel];
  if(radio_channel>14) radio_channel = 0;

  switch(radio_channel){
    case 0:
      RF95_FREQ = RF95_FREQ_0;
      break;
    case 1:
      RF95_FREQ = RF95_FREQ_1;
      break;
    case 2:
      RF95_FREQ = RF95_FREQ_2;
      break;
    case 3:
      RF95_FREQ = RF95_FREQ_3;
      break;
    case 4:
      RF95_FREQ = RF95_FREQ_4;
      break;
    case 5:
      RF95_FREQ = RF95_FREQ_5;
      break;
    case 6:
      RF95_FREQ = RF95_FREQ_6;
      break;
    case 7:
      RF95_FREQ = RF95_FREQ_7;
      break;
    case 8:
      RF95_FREQ = RF95_FREQ_8;
      break;
    case 9:
      RF95_FREQ = RF95_FREQ_9;
      break;
    case 10:
      RF95_FREQ = RF95_FREQ_10;
      break;
    case 11:
      RF95_FREQ = RF95_FREQ_11;
      break;
    case 12:
      RF95_FREQ = RF95_FREQ_12;
      break;
    case 13:
      RF95_FREQ = RF95_FREQ_13;
      break;
    case 14:
      RF95_FREQ = RF95_FREQ_14;
      break;
    default:
      RF95_FREQ = RF95_FREQ_0;
      break;
  }

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while(1);
  }
  #ifdef SERIAL_CONSOLE
    Serial.println("LoRa radio init OK!");
  #endif
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while(1);
  }
  #ifdef SERIAL_CONSOLE
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  #endif
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  //Check for PING packets
  setTxChannel();
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

bool bValidGGA = false;
char sentenceIdentifier[7] = "$GNGGA"; //Global Positioning System Fix Data

char radioPKT[RH_RF95_FIFO_SIZE ];
bool LED_trigger = true;

void loop()
{
  unsigned long timeout ;
  int inIndex = 0;
  radioPKT[inIndex]='\0';
  bValidGGA = false;
  
  //Read in GPS sentence one char at a time.
  //Find the start of the sentence the $. This aligns the buffer to the beginning of the sentence
  timeout = millis() + 1000;
  while((millis() < timeout) && radioPKT[inIndex]!='$'){
    if (Serial1.available() > 0) {
        // read the incoming byte:
        radioPKT[inIndex] = Serial1.read();
    }
  }
  inIndex++; //Found $ so increment
  timeout = millis() + 1000;
  //Now we have the start of a sentence read in the remainder
  while ((millis() < timeout) && (inIndex < RH_RF95_FIFO_SIZE)) {
    if (Serial1.available() > 0) { //Rememeber Serial buffer is 64 bytes long
      // read the incoming byte:
      radioPKT[inIndex] = Serial1.read();
      //Compare each incoming byte to the matching string and exit if it doesn't match
      if((inIndex>2) && (inIndex<6)) { //Its in the testing window
        if(radioPKT[inIndex] != sentenceIdentifier[inIndex]){ //Walking pointer testing for GNGGA message
          radioPKT[0] = '\0';
          bValidGGA = false;
          break;
        }        
      }
      //Identify end of GPS sentence and move on
      if (radioPKT[inIndex] == '\r') {
        bValidGGA = true;        
        inIndex++;
        break;
      }
      inIndex++;
    }
  }
  /*
  //Add identification string
  strcat(radioPKT,"DEHATE");
  inIndex += 6;
  */
  radioPKT[inIndex] = '\0'; // put a null delimiter on string

  if (bValidGGA) {
    #ifdef SERIAL_CONSOLE
      Serial.write(radioPKT, inIndex);
    #endif
    // Now write out through the radio
    delay(10);
    rf95.send((uint8_t *)radioPKT, inIndex);
    #ifdef SERIAL_CONSOLE
        Serial.println("\nWaiting for packet to complete...");
    #endif
    delay(10);
    rf95.waitPacketSent();
    if (LED_trigger) {
      digitalWrite(LED, HIGH);
      LED_trigger = false;
    }
    else {
      digitalWrite(LED, LOW);
      LED_trigger = true;
    }
  }
}


// Allow selection of transmitter channel
void setTxChannel(void) {
  char* PING = "PING";//PING test
  char* ACK = "ACK";//ACK test"Channel:"
  char* CHANNEL = "Channel:";//CHANNEL test
  bool done = false;
  uint8_t len; 
  uint8_t radioBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t testBuf[RH_RF95_MAX_MESSAGE_LEN];
  digitalWrite(LED, LOW);
  unsigned long start = millis() + 5000;// 5sec delay
  while(!done){
    if(millis() > start) return; //Timeout return
    //////////////////////////////////////////////
    // Wait for message over radio
    if (rf95.available()){
      len = sizeof(radioBuf);
      if (rf95.recv(radioBuf, &len)){
        digitalWrite(LED, HIGH);
        #ifdef SERIAL_CONSOLE
          Serial.print("\nGot: ");
          Serial.write((char*)radioBuf, len-1);
          Serial.print("\n");
        #endif
        radioBuf[4] = 0;
        if(!strcmp((char *)radioBuf, PING)) done = true; //strcmp returns zero if a match
      }
    }
  }
  digitalWrite(LED, LOW);
  //Send ACK
  delay(10);
  rf95.send((uint8_t *)ACK, 3);
  #ifdef SERIAL_CONSOLE
    Serial.println("Wait for reply");
  #endif
  delay(10);
  rf95.waitPacketSent();
  done=false;
  while(!done){
    //Wait for Radio channel
    if (rf95.available()){
      len = 11;
      if (rf95.recv(radioBuf, &len)){
        digitalWrite(LED, HIGH);
        #ifdef SERIAL_CONSOLE
          Serial.print("\nReceived: ");
          Serial.write((char*)radioBuf,10);
        #endif
        testBuf[0] = 0;
        strcat((char *)testBuf,(char *)radioBuf);
        testBuf[8] = 0;
        if(!memcmp((char *)testBuf, CHANNEL, 8)) done = true; //Is it the magic word?  //strcmp returns zero if a match
      }
    }
  }
    //Extract the radio_channel
    char CH[3];
    CH[0] = radioBuf[8]; CH[1] = radioBuf[9]; CH[2] = '\0';
    radio_channel = atoi(CH); //convert from ASCII
    //Send channel back as ACK then set channel
    delay(10);
    len = sizeof(radioBuf);
    #ifdef SERIAL_CONSOLE
      Serial.println("\nChannel sent back");
      Serial.println(radio_channel);
    #endif
    sprintf((char *)radioBuf,"Channel:%02d",radio_channel);
    radioBuf[10] = 0;
    rf95.send((uint8_t *)radioBuf, 10); //Fixed length string 10 characters
    delay(10);
    rf95.waitPacketSent();
    // Set channel
    #ifdef SERIAL_CONSOLE
      Serial.print("\nNew ");
      Serial.println((char *)radioBuf);
    #endif
    // Write the new value into EEPROM
    EEPROM[FLASHchannel] = radio_channel; 
    while(1);    
}