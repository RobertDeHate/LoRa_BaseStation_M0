// v1.1 - first fielded version
// v1.2 - Changed SF = 10
// v1.3 - Changed for SN#2 and beyond - fixed color bitmap
// v1.4 - Added SD card logging, flash storage of channel
// v1.5 - Added Pulling last coordinates from SD card on power on
//        Last packet timer on remote display & AGL on tracker display.
// v1.6 - Added compass calibration. Fixed bug in flash storage of channel, it was writing every iteration of the loop
//        Added setup menu
// V1.7 - Added communication to tracker to allow changing tracker channel
//        Added Scan channels option to look for used channels. Added channel display on remote page
// V1.8 - Added support for SD cards for data logging and storage of Last Know Good GPS data.
//		Support for 1 wire EEPROM to store Last Known Good received GPS message. 
//		Allows changing the transmitter channel. 
//		Scanning of channels to see which are in use. 
//		Calibrating the compass module. 
//		Large arrow support for tracking screen. Display of Maximum speed. Display of Maximum altitude. Display of battery status on basestation display.
#include <Button.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <RH_RF95.h>
#include <QMC5883LCompass.h>
#include <SD.h>
#include <FlashAsEEPROM.h>
#include <OneWire.h>

#include "Arrows.h"
#include "data.h"

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//#define SERIAL_CONSOLE 1
#define LCD_DISPLAY 1
#define COMPASS 1
#define SD_CARD 1
#define OneWireEE 1 // Use 1wire EEPROM for LKG GPS
#define OneWirePin 14
#define OneWirePower 15
#define LOCAL_GPS 1
#define directionalArrow 1 //Use big arrow for naviagation

const char version_str[] = {"Version: 1.8  SN: xx"};

// The TinyGPS++ object
TinyGPSPlus gps;
TinyGPSPlus gpsBS;   //basestation local GPS object

// last known good Remote Lat/long
typedef struct
{
  double Lat;
  double Long;
} GPS_Pos_struct;

GPS_Pos_struct  LKG_Position;

int displayIndex = 0, olddisplayIndex=0;

// These pins will also work for the 1.8" TFT shield.
#define TFT_CS  6 // Adafruit Feather
#define TFT_RST 5 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC  10
#define TFT_MOSI  11 // Data out
#define TFT_SCLK  12 // Clock out

// For 1.44" and 1.8" TFT with ST7735 use:
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);   // SW SPI

#ifdef COMPASS
  QMC5883LCompass compass;
#endif

// Button object
Button button(A5); //Button might have to be modified depending upon what library you have

// Menu definitions
#define RADIO_GPS_MENU  0
#define TRACKER_MENU  1
#define LOCAL_GPS_MENU 2

// Distance conversion
#define METERS_2_FEET 3.291

float RF95_FREQ;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// Battery voltage input
#define VBATPIN A7
float Vmn=3.2; //Battery minimum voltage for battery display
float Vmx=4.2; //Battery maximum voltage for battery display


//SD file
File myFile;

//EEPROM
OneWire ds(OneWirePin); //attach signal pin of EEPROM

//define compass calibration structure
typedef struct {
  int x0;
  int x1;
  int y0;
  int y1;
  int z0;
  int z1;
} Compass_Calibration;

//Reserve a portion of flash memory 
FlashStorage(FLASHchannel, int);
FlashStorage(FLASHCompass_Calibration, Compass_Calibration);

//Create global variable
Compass_Calibration Compass_cal;
int radio_channel;
int LastPackettime = 144000; //Set timer initial value high to indicate no packet has come in
//max indicators
int max_altitude_feet = 0;
int max_speed_mph = 0;
uint8_t gps_len; 
uint8_t radioBuf[RH_RF95_MAX_MESSAGE_LEN];
bool first_fix = true;
uint8_t display_mode = RADIO_GPS_MENU;
int heading = 0;
bool updatedGPSFix = true;
int updateDisplayCnt = 0;
uint8_t msg_len = 0, msg_cnt = 0; 
uint8_t maxAGLlen=0;

void setup() {
  pinMode(LED, OUTPUT);
	button.begin();

  #ifdef SERIAL_CONSOLE
    // Open serial communications to console
    Serial.begin(115200);
    while (!Serial) { // Blocks execution if NO USB cable connected
      delay(1);
    }
    delay(100);
    Serial.println("Initialized Console");
  #endif

  #ifdef LOCAL_GPS
    // Initial GPS Port
    Serial1.begin(9600);
  #endif

  #ifdef SD_CARD
    // Initialize SD card
    bool SDreturn = SD.begin(18);//Initialize SD card
    #ifdef SERIAL_CONSOLE
      Serial.println("Initializing SD card...");
      if (!SDreturn) {
        Serial.println("SD initialization failed!");
      }
      else{
        Serial.println("SD initialization done.");
      }
    #endif
  #endif

  #ifdef OneWireEE
    //Set 1wire pins
    pinMode(OneWirePin, OUTPUT);
    digitalWrite(OneWirePin, HIGH);
    pinMode(OneWirePower, OUTPUT);
    digitalWrite(OneWirePower, HIGH);
  #endif

  #ifdef COMPASS
    bool compassNOTcalibrated = 0;
    //init compass
    compass.init();
    //Calibrate compass
    //Get stored value and if zero force calibration
    Compass_cal = FLASHCompass_Calibration.read();
    if (Compass_cal.x0+Compass_cal.x1+Compass_cal.y0+Compass_cal.y1+Compass_cal.z0+Compass_cal.z1 == 0) {
      compassNOTcalibrated = 1;
    }
    else{
      compass.setCalibration(Compass_cal.x0,Compass_cal.x1,Compass_cal.y0,Compass_cal.y1,Compass_cal.z0,Compass_cal.z1);
    }
  #endif

  #ifdef SERIAL_CONSOLE
    Serial.println("Start Listening for GPS\n");
  #endif

  #ifdef LCD_DISPLAY
  // Use this initializer if using a 1.8" TFT screen:
    tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  #endif
  
  #ifdef OneWireEE
    // Get the last Remote GPS data
    getLastGPS();
  #else
    #ifdef SD_CARD
      // Get the last Remote GPS data
      getLastGPS(); //This can take some time if the log file is over 200k
    #endif
  #endif

  // Get radio_channel from flash
  radio_channel = FLASHchannel.read();

  #ifdef SERIAL_CONSOLE
    Serial.print("Read Radio Chan: ");
    Serial.println(radio_channel);
  #endif
    
  RF95_FREQ = FreqLookup(radio_channel);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    #ifdef SERIAL_CONSOLE
      Serial.println("LoRa radio init failed");
      Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    #endif
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    #ifdef SERIAL_CONSOLE
      Serial.println("setFrequency failed");
    #endif
    while (1);
  }
  #ifdef SERIAL_CONSOLE
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  #endif

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on, modulation GFSK_Rb250Fd250, +13dbM
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);   // set defaults in RH_RF95.cpp

  #ifdef LCD_DISPLAY
    unsigned long timeout;
    //init display
    #ifdef SERIAL_CONSOLE
      Serial.println("Init display");
    #endif
    tft.fillScreen(ST77XX_BLACK);   // clear display
    // Display initial screen - waiting for GPS from Radio
    tft.setRotation(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setTextSize(1);
    tft.setCursor(0, 30);
    tft.println(version_str);
    tft.println("Waiting for GPS...");
    tft.print("...on Channel ");
    tft.print(radio_channel);

    if(compassNOTcalibrated){
      tft.println("\n\nCOMPASS NOT CALIBRATED!!");
      tft.println("\nCalibrating compass now");
      delay(3000);
      CalibrateCompass();
    }
    else{// Check for button press to enter setup
      tft.println("\n\nPress button to enter \nSetup");
      timeout = millis() + 3000;// 3sec timeout
      while((timeout > millis())){
        if(button.pressed()){
          SetupMenu();
          timeout = 0;
        }
      }
    }
  #endif
}

void loop() {
  /////////////////////////////////////////////
  //   Time base for packet receive timer
  LastPackettime ++;
  ////////////////////////////////////////////
  //  check the button
  // and see if we should switch screens
  if(button.pressed()){
     if(++display_mode > LOCAL_GPS_MENU){
       display_mode = RADIO_GPS_MENU;
     }
     olddisplayIndex = -1; //Force redraw of arrow
    #ifdef LCD_DISPLAY
        tft.fillScreen(ST77XX_BLACK);   // clear display
    #endif
  }

  //////////////////////////////////////////////
  //  Monitor the remote GPS
  // Wait for message over radio
  if (rf95.available()){
    // Message received
    gps_len = sizeof(radioBuf);
    if (rf95.recv(radioBuf, &gps_len)){
      digitalWrite(LED, HIGH);
        // Encode GPS object
        while(msg_len--) gps.encode(radioBuf[msg_cnt++]); 

      #ifdef SERIAL_CONSOLE
        Serial.print("Got: ");
        Serial.write((char*)radioBuf, gps_len);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);

      
        Serial.println();
      #endif

      // Test for valid FIX
      if(gps.location.isUpdated()){
        #ifdef SD_CARD
          //Now write data to SD card
          // open the file.
          myFile = SD.open("LOG.TXT", FILE_WRITE);

          // if the file opened okay, write to it:
          if (myFile) {
            myFile.write(radioBuf, gps_len);
            myFile.close();
          }
        #endif
        
        #ifdef OneWireEE
          //Write data to EEPROM
          saveLastGPStoEE();
        #endif
          
        updatedGPSFix = true;   // update loop indicator that we've had a new GPS fix
        LastPackettime = 0;
        
        // update LKG position
        LKG_Position.Lat = gps.location.lat();
        LKG_Position.Long = gps.location.lng();
        
        //Save new MAX altitude
        if(gps.altitude.feet()-gpsBS.altitude.feet() > max_altitude_feet) max_altitude_feet = gps.altitude.feet()-gpsBS.altitude.feet();
        //Save new MAX speed
        if(gps.speed.mph() > max_speed_mph) max_speed_mph = gps.speed.mph();

        digitalWrite(LED, LOW);
      }
    }
    digitalWrite(RFM95_CS,HIGH);//Disable the radio
  } 

  #ifdef COMPASS
    //////////////////////////////////////////////////////////////////////////
    // Read compass values
    compass.read();
    heading = compass.getAzimuth();
    // subtract mounting error of 90 degs but add the 11 deg inclination angle = -79 degs
    heading -= 79;
    if(heading < 0)
      heading = 360 + heading;
  #endif

  #ifdef LOCAL_GPS
    //////////////////////////////////////////////////////////////////////////
    // Then get fix from basestation GPS

    unsigned long timeout = millis() + 250;
    uint8_t inIndex = 0;

    while((Serial1.available() > 0) && (timeout > millis()) ) {
      // read the incoming byte:
      radioBuf[inIndex] = Serial1.read();
      gpsBS.encode(radioBuf[inIndex++]);
    }
    #ifdef SERIAL_CONSOLE
      Serial.write(radioBuf,inIndex);
      Serial.print(" ");
      Serial.println(display_mode);
    #endif
    // Now update display
    if(gpsBS.location.isUpdated())
      updatedGPSFix = true;   // update loop indicator that we've had a new GPS fix
  #endif

  ////////////////////////////
  //  Lastly, update the display if its time

  if(((++updateDisplayCnt % 2) == 0) && updatedGPSFix){// only update display on every other instance of update GPS fix
    // Check if we need to clear the screen
    if(first_fix){    
      #ifdef LCD_DISPLAY
        tft.fillScreen(ST77XX_BLACK);   // clear display
      #endif
      first_fix = false;
    }
    #ifdef LCD_DISPLAY
      updateDisplay(display_mode);  // new GPS update, update display
    #endif
    updatedGPSFix = false;
  }
 
}


/* 
 *  
 *  Display the appropriate menu or display
 *  
 */
static void updateDisplay(uint8_t display)
{
  TinyGPSPlus *pGPS; 
  int distanceToTarget, courseToTarget;
  float measuredvbat;
  char numberArray[20];
  size_t len;
  
  radio_channel = FLASHchannel.read();

  if(display != TRACKER_MENU) {
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setTextSize(1);
    
    #ifdef SERIAL_CONSOLE
      Serial.print("CH");
      Serial.println(radio_channel);
    #endif
    // Draw header
    if(display == RADIO_GPS_MENU) {
      tft.setCursor(0, 0);
      tft.print("CH");
      tft.print(radio_channel);
      tft.setCursor(40, 0);
      tft.print("Remote GPS");
      tft.setCursor(120, 0);
      tft.print(LastPackettime/10);
      ultoa((LastPackettime/10),numberArray,10);
      len = strlen(numberArray)*6;
      Serial.print("numberArray-");Serial.println(numberArray);
      Serial.print("len-");Serial.println(len);
      tft.fillRect(120+len,0,160-120+len,8,ST77XX_BLACK); //Clear extra digits
      pGPS = &gps;
    }
    else {
      tft.setCursor(30, 0);  
      tft.print("Basestation Data");
      pGPS = &gpsBS;
    }
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0,10);
    if(display == RADIO_GPS_MENU) {
      tft.print(LKG_Position.Lat,6);
      tft.setCursor(0,30);
      tft.print(LKG_Position.Long,6);
      tft.setCursor(0,50);
      tft.print("ALT: ");
      tft.print(pGPS->altitude.feet(),0);
    }
    else {  
      tft.print(pGPS->location.lat(),6); 
      tft.setCursor(0,30);   
      tft.print(pGPS->location.lng(),6);
      tft.setCursor(0,50);   
      tft.print("Compass: ");
      tft.print(heading);
      tft.print(" ");
      if (heading<10) tft.print(" "); //Only single digit so add one more space to clear out the 3 digits
    }
    tft.setCursor(0,70);   
    tft.print("#Sats: ");
    tft.print(pGPS->satellites.value());
    tft.print(" ");
    tft.setCursor(0,90);   
    tft.print("Time: ");
    tft.print(pGPS->time.second()); 
    tft.print("  ");
    if(display == RADIO_GPS_MENU) {
      int lastRssi = rf95.lastRssi();
      digitalWrite(RFM95_CS,HIGH);//Disable the radio
      tft.setCursor(0,110);   
      tft.print("RSSI: ");
      tft.print(lastRssi, DEC);
      tft.print("    ");
    }
    else {	// compute and display battery voltage
      tft.setCursor(0,110);   
      tft.print("VBat: ");
      measuredvbat = analogRead(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      tft.print(measuredvbat, 2);
      tft.print(" ");
      //Draw battery symbol
      tft.fillRect(148,90,4,4,ST77XX_WHITE);
      tft.drawRect(145,94,10,26,ST77XX_WHITE);
      //Fill up battery
      float Prange=26,Psz;
      Psz=((measuredvbat-Vmn)/(Vmx-Vmn))*Prange;
      if(Psz>Prange) Psz=Prange; if(Psz<0) Psz=0;
      tft.fillRect(146,95,8,Prange-Psz,ST77XX_BLACK);
      tft.fillRect(145,120-Psz,10,Psz,ST77XX_WHITE);
    }
  } 
  else{    // If we got here, display must be tracker.  Process the tracker mode
    char cTemp[10];
    distanceToTarget = METERS_2_FEET*(int) gps.distanceBetween(gpsBS.location.lat(), gpsBS.location.lng(), LKG_Position.Lat, LKG_Position.Long);
    courseToTarget = (int) gps.courseTo(gpsBS.location.lat(), gpsBS.location.lng(),LKG_Position.Lat, LKG_Position.Long);

    // check distance units
    if(distanceToTarget > 5280)
      sprintf(cTemp,"%.2fMi    ",(float)distanceToTarget/5280);
    else
      sprintf(cTemp,"%dFt    ",distanceToTarget);
    cTemp[6] = 0; //Trim the string to 6 chars long
    // adjust heading based on compass
    courseToTarget -= heading;
    if(courseToTarget < 0)
      courseToTarget = 360 + courseToTarget;
    
  //Draw arrow to target    
    #ifdef directionalArrow
      displayIndex = courseToTarget / INCREMENT_DEGS_AR;
      if((courseToTarget % INCREMENT_DEGS_AR) > (INCREMENT_DEGS_AR/2))
        displayIndex++;
      if(displayIndex >= NUM_POINTS_AR){
        displayIndex = 0;
      }
      char buf[6]; 
      int16_t x1, y1,iTemp;
      uint16_t w, h;
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.print("D:"); tft.print(cTemp); // print distance
      sprintf(buf,"%d",max_altitude_feet);
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(160-w, 0);
      tft.print(buf);
      tft.setCursor(136, 17);
      tft.print("Ft");
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.setCursor(0, 114);
      if(gps.altitude.isValid() && gpsBS.altitude.isValid()) iTemp =gps.altitude.feet()-gpsBS.altitude.feet(); //Test if the GPS data is good
      else iTemp = 0;
      sprintf(cTemp,"AGL:%d     ",iTemp);
      if(maxAGLlen < strlen(cTemp)) maxAGLlen = strlen(cTemp);
      cTemp[maxAGLlen]; //Trim string to maximum lenth to clear screen
      tft.print(cTemp);
      tft.setCursor(136, 97);
      tft.print("MH");
      sprintf(buf,"%d",max_speed_mph);
      tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(160-w, 114);
      tft.print(buf);
      if(olddisplayIndex!=displayIndex){
        olddisplayIndex=displayIndex;
        tft.fillRect(30, 14, 100, 100, ST77XX_BLACK);
        tft.drawBitmap(30, 14, Arrow_bitmap_allArray[displayIndex],100,100,ST77XX_WHITE);
      }
      tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK); //BLUE is RED, go figure
      tft.setCursor(148, 40);
      tft.print("M");
      tft.setCursor(148, 57);
      tft.print("A");
      tft.setCursor(148, 74);
      tft.print("X");
    #else
      // Put update of arrows here. first clear the last pointer icon
      tft.fillTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_BLACK);
      displayIndex = courseToTarget / INCREMENT_DEGS;
      if((courseToTarget % INCREMENT_DEGS) > (INCREMENT_DEGS/2))
        displayIndex++;
      if(displayIndex >= NUM_POINTS){
        displayIndex = 0;
      }
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(50, 40);
      tft.print("D:");
      tft.print(dist);  // print distance
      tft.print("    ");
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.setCursor(30, 60);
      tft.print("AGL:");
      tft.print(gps.altitude.feet()-gpsBS.altitude.feet(),0);
      tft.print("    ");
        tft.drawCircle(80, 64, 63, ST77XX_WHITE);
        tft.fillTriangle(Tracker_icon[displayIndex].x0, Tracker_icon[displayIndex].y0, Tracker_icon[displayIndex].x1, Tracker_icon[displayIndex].y1, Tracker_icon[displayIndex].x2, Tracker_icon[displayIndex].y2, ST77XX_GREEN);
    #endif
  }
      
}

//Store latest GPS to EEPROM
static void saveLastGPStoEE(void){
  uint8_t len, A1, A2, A3;
  int addy=0, pt; 
  /// Write 128 bytes
  while(addy < 128)
  {
    ds.reset();
    ds.skip();
    ds.write(0x0F,1); // Write Scratchpad command
    ds.write(addy,1);// Address
    ds.write(0,1);// Address + 1
    // Now send data to write to EEPROM
    //ds.write_bytes((uint8_t *)buf[addy], 32);
    pt=0;
    while(pt < 8 ){
      ds.write(radioBuf[addy + pt],1);
      pt++;
      }
    // Read Scratchpad to get 3-byte authorization pattern
    ds.reset();
    ds.skip();
    ds.write(0xAA,1); // Read Scratchpad command
    A1 = ds.read(); //Get authorization byte 1
    A2 = ds.read(); //Get authorization byte 2
    A3 = ds.read(); //Get authorization byte 3
    while(!ds.reset());  //Serial.print("RS W3\n");
    ds.skip();
    ds.write(0x55,1); // Copy Scratchpad command
    ds.write(A1,1); // authorization byte 1
    ds.write(A2,1); // authorization byte 2
    ds.write(A3,1); // authorization byte 3
    delay(12);
    addy += 8;
  }
  ds.reset();
}

// Get the last Remote GPS data stored in SD or EE
static void getLastGPS(void) {
  #ifdef OneWireEE
    uint8_t msg_len = 0; 
    char c;
    ds.reset();
    ds.skip();
    ds.write(0xF0,1); // Read command
    ds.write(0x00,1);// Address
    ds.write(0x00,1);// Address + 1
    //Now just read data
    ds.read_bytes((uint8_t *)radioBuf, 128);
    ds.reset();
    do{
      c = radioBuf[msg_len++];
      gps.encode(c);
    } while(c);
  #else
    // Must extract data from last line in SD card
    // open the file.
    myFile = SD.open("LOG.TXT",FILE_WRITE);//Opens file for reading/writing pointing to the end of the file
    // if the file opened okay, read from it:
    if (myFile) {
      for(unsigned long pos = myFile.size()-2; pos != 0 && myFile.seek(pos); --pos) {
        int rd = myFile.peek();
        if (rd < 0)
            break;
        // found! 
        if (rd == '\n')
            break;
      }
      if(myFile.position() > 1) myFile.read(); //Moves pointer up one
      
      //Now the file pointer is on the last sentence so extract
      while (myFile.peek() != '\n' && myFile.peek()!=-1){
        gps.encode(myFile.read());   // radio GPS object        
      }  
      // close the file:
      myFile.close();
    }
  #endif
  if(gps.location.isUpdated()){
    //Parse the data
    LKG_Position.Lat = gps.location.lat();
    LKG_Position.Long = gps.location.lng();
  }
}

// Setup Menu
static void SetupMenu(void) { 
  int old_radio_channel;
  int new_radio_channel;
  int selection = 0;
  //Setup display
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Highlight selection\nWait for timeout");
  tft.setTextSize(2);
  unsigned long currentMillis = millis();
  unsigned long previousMillis = currentMillis;
  while(currentMillis - previousMillis <= 5000){ //delay in mS
    tft.setCursor(0, 20);
    switch(selection){
      case 1:
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SETUP\n");
        tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.println("SET CHANNEL");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SET TX CH");
        tft.println("SCAN CHANNELS");
        tft.println("CAL COMPASS");
        break;
      case 2:
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SETUP\n");
        tft.println("SET CHANNEL");
        tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.println("SET TX CH");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SCAN CHANNELS");
        tft.println("CAL COMPASS");
      break;
      case 3:
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SETUP\n");
        tft.println("SET CHANNEL");
        tft.println("SET TX CH");
        tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.println("SCAN CHANNELS");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("CAL COMPASS");
      break;
      case 4:
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SETUP\n");
        tft.println("SET CHANNEL");
        tft.println("SET TX CH");
        tft.println("SCAN CHANNELS");
        tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.println("CAL COMPASS");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      break;
      default:
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.println("SETUP\n");
        tft.println("SET CHANNEL");
        tft.println("SET TX CH");
        tft.println("SCAN CHANNELS");
        tft.println("CAL COMPASS");
      break;
    }
    if(button.pressed()) {
      previousMillis = currentMillis;
      selection++;
      if(selection>4) selection = 0;
    }
    currentMillis = millis();
  }
  //Now act on the selection
  switch(selection){
    case 1:
      // First, get radio_channel from flash
      old_radio_channel = radio_channel = FLASHchannel.read();
      new_radio_channel = getRadioChannel();
      if(new_radio_channel != old_radio_channel) {
        // Write the new value into flash
        FLASHchannel.write(new_radio_channel); 
      }
      tft.fillScreen(ST77XX_BLACK);   // clear display
      tft.setTextSize(2);
      tft.setCursor(0, 30);
      tft.println("POWER CYCLE \nUNIT");
      while(1);
      break;
    case 2:
      radio_channel = setTxChannel(); //Does not return
      break;
    case 3:
      scanChannels(); //Does not return
      break;
    case 4:
      CalibrateCompass();
      tft.fillScreen(ST77XX_BLACK);   // clear display
      tft.setTextSize(2);
      tft.setCursor(0, 30);
      tft.println("POWER CYCLE \nUNIT");
      while(1);
      break;
    default:
      break;
  }
}

// Allow selection of receiver channel
static int getRadioChannel(void)
{
  bool changed = false;
  int TimerCnt = 20; // 20 times with 200 msec delay = 5 secs
  
  if(radio_channel > MAX_OP_CHAN)
    radio_channel = 0;  // only MAX_OP_CHAN channels, if we read higher, set to zero

  // Loop for 5 secs and wait for button press
  
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setRotation(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Button to select\nWait for timeout");
  tft.setTextSize(2);
  tft.setCursor(0, 25);
  tft.println("Current");
  tft.print("Channel = ");
  tft.println(radio_channel);
  tft.print("\nSelected\nChannel = ");
  tft.println(radio_channel);
  while(TimerCnt--) {
    if(button.pressed()){
      tft.setCursor(0, 89);
      if(++radio_channel > MAX_OP_CHAN)
        radio_channel = 0;
      tft.print("Channel = ");
      tft.print(radio_channel);
      tft.println("  ");
      TimerCnt = 20;
      changed = true;
    }
    delay(200);
  }
  return radio_channel;
}

// Calibrate the compass
static void CalibrateCompass(void) { 
  bool changed = false;
  bool done = false;
  int t = 0;
  int c = 0;
  int x, y, z;
  int calibrationData[3][2];
  //Clear data
  calibrationData[0][0]=0;
  calibrationData[0][1]=0;
  calibrationData[1][0]=0;
  calibrationData[1][1]=0;
  calibrationData[2][0]=0;
  calibrationData[2][1]=0;
  //Setup display
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 30);
  tft.print("This will calibrate your\ncompass.\nMove the compass in all \ndirections until \ncalibration is complete.");
  tft.println("\n\nPress button to continue");
  while(!button.pressed());
  tft.fillScreen(ST77XX_BLACK);   // clear display
  while(!done){
    int x, y, z;
    // Read compass values
    compass.read();
    // Return XYZ readings
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();
    changed = false;
    if(x < calibrationData[0][0]) {
      calibrationData[0][0] = x;
      changed = true;
    }
    if(x > calibrationData[0][1]) {
      calibrationData[0][1] = x;
      changed = true;
    }
    if(y < calibrationData[1][0]) {
      calibrationData[1][0] = y;
      changed = true;
    }
    if(y > calibrationData[1][1]) {
      calibrationData[1][1] = y;
      changed = true;
    }
    if(z < calibrationData[2][0]) {
      calibrationData[2][0] = z;
      changed = true;
    }
    if(z > calibrationData[2][1]) {
      calibrationData[2][1] = z;
      changed = true;
    }
    
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.print("CALIBRATING:\n");
    
    tft.print("X:");
    tft.print(calibrationData[0][0]);
    tft.print(", ");
    tft.println(calibrationData[0][1]);
    tft.print("Y:");
    tft.print(calibrationData[1][0]);
    tft.print(", ");
    tft.println(calibrationData[1][1]);
    tft.print("Z:");
    tft.print(calibrationData[2][0]);
    tft.print(", ");
    tft.println(calibrationData[2][1]);
    tft.println("\nPress button to continue");
      
    if ( button.pressed()) {
      done = true;
      Compass_cal.x0 = calibrationData[0][0];
      Compass_cal.x1 = calibrationData[0][1];
      Compass_cal.y0 = calibrationData[1][0];
      Compass_cal.y1 = calibrationData[1][1];
      Compass_cal.z0 = calibrationData[2][0];
      Compass_cal.z1 = calibrationData[2][1];
      //Set value
      compass.setCalibration(Compass_cal.x0,Compass_cal.x1,Compass_cal.y0,Compass_cal.y1,Compass_cal.z0,Compass_cal.z1);
      //Store value
      FLASHCompass_Calibration.write(Compass_cal ); 
          
      #ifdef SERIAL_CONSOLE
        Serial.print("compass.setCalibration(");
        Serial.print(calibrationData[0][0]);
        Serial.print(", ");
        Serial.print(calibrationData[0][1]);
        Serial.print(", ");
        Serial.print(calibrationData[1][0]);
        Serial.print(", ");
        Serial.print(calibrationData[1][1]);
        Serial.print(", ");
        Serial.print(calibrationData[2][0]);
        Serial.print(", ");
        Serial.print(calibrationData[2][1]);
        Serial.println(");");
      #endif
    }
  }
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setTextSize(2);
  tft.setCursor(0, 30);
  tft.println("POWER CYCLE \nUNIT");
  while(1);
}

// Allow selection of transmitter channel
static int setTxChannel(void)
{
  uint8_t len; 
  bool done = false;
  bool changed = false;
  bool avail = false;
  unsigned long start = millis() + 1000;// 1sec delay
  //Setup display
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setRotation(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("PINGing TX");
  tft.println("Power ON TX");
  tft.println("\nPower off to exit");
  
  // Transmit PING looking for ACK once a second
  while(!done){
    #ifdef SERIAL_CONSOLE
      Serial.write("PING", 4);
    #endif
    // Now write out through the radio
    delay(10);
    rf95.send((uint8_t *)"PING", 5);
    #ifdef SERIAL_CONSOLE
        Serial.println(" Waiting for packet to complete...");
    #endif
    delay(10);
    rf95.waitPacketSent();
    tft.print(".");
     //Wait for ACK
     avail = false;
     start = millis() + 1000;// 1sec delay
     do {
        if (rf95.available()){
          avail = true;
          start = 0;
        }
      } while (millis() < start);

    if (avail){
      len = sizeof(radioBuf);
      if (rf95.recv(radioBuf, &len)){
        digitalWrite(LED, HIGH);
        #ifdef SERIAL_CONSOLE
          Serial.print("\nGot: ");
          Serial.write((char*)radioBuf, len);
        #endif
        radioBuf[3] = 0;
        if(!strcmp((char *)radioBuf, "ACK")) done = true;//Return is zero on a match
      }
    }
  }
  //Print success message
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("Connected to\nRadio Tracker");
  delay(1000);
  tft.println("\nSet new\nchannel");
  delay(1000);
  while(1){
    // Get a new channel
    radio_channel = getRadioChannel();
    sprintf((char *)radioBuf,"Channel: %d",radio_channel);
    #ifdef SERIAL_CONSOLE
      Serial.print("\nNew ");
      Serial.println((char *)radioBuf);
    #endif
    tft.fillScreen(ST77XX_BLACK);   // clear display
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("New Channel ");
    tft.println(radio_channel);
    //Send new channel to transmitter
    #ifdef SERIAL_CONSOLE
        Serial.println("Channel sent to Tx.");
    #endif
    delay(10);
    len = strlen((char *)radioBuf);
    rf95.send((uint8_t *)radioBuf, len);
    delay(10);
    rf95.waitPacketSent();
    //Wait for ACK
    avail = false;
    start = millis() + 10000;// 10sec delay
    do {
      if (rf95.available()){
        avail = true;
        start = 0;
      }
    } while (millis() < start);

    if (avail){
      #ifdef SERIAL_CONSOLE
        Serial.println("Last ACK");
      #endif
      len = sizeof(radioBuf);
      if (rf95.recv(radioBuf, &len)){
        digitalWrite(LED, HIGH);
        #ifdef SERIAL_CONSOLE
          Serial.print("\nGot: ");
          Serial.write((char*)radioBuf, len);
        #endif
        // Write the new value into flash
        FLASHchannel.write(radio_channel); 
        tft.println("\nChannel \nUpdated");
        tft.setTextSize(2);
        tft.setCursor(0, 96);
        tft.println("POWER CYCLE \nUNIT");
        while(1);
      }
    }
    else{
      tft.fillScreen(ST77XX_BLACK);   // clear display
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setTextSize(2);
      tft.setCursor(0, 30);
      tft.println("Verify \nchannel");
      tft.println("\nPOWER CYCLE \nUNIT");
      while(1);
    }
  }
}

//Scan channels to find Tx's
void scanChannels(){
  bool found = false;
  unsigned long start;
  int ch = 0;
  int16_t Xpos = 0, Ypos = 0;
  uint8_t templen;
  uint8_t tempbuf[RH_RF95_MAX_MESSAGE_LEN];
  //Setup a screen to show channels
  tft.fillScreen(ST77XX_BLACK);   // clear display
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setRotation(1);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Channel Scanning");
  while(ch<15){
    rf95.init();
    delay(10);
    while(!rf95.setFrequency(FreqLookup(ch)));//Set next frequency
    digitalWrite(RFM95_RST, LOW); delay(10);
    digitalWrite(RFM95_RST, HIGH); delay(10);
    while(rf95.recv(tempbuf, &templen));// Clean out buffer
    found = false;// Reset this after freq change to use it as a flag
    start = millis() + 2000; // 3 second scan
    do {
      delay(500);
      if(rf95.available()){
        found = true;
        start = 0;
      }  
    } while (millis() < start);
    if(ch == 8) { Xpos = 80; Ypos = 0;} //Second row
    tft.setCursor(Xpos, (Ypos+= 14));
    tft.print("Ch"); tft.print(ch); tft.print(" ");
    if(found)  tft.print("Found");
    else tft.print("None");
    ch++;
  }
  tft.setCursor(80, 115);
  tft.print("FINISHED");
  while(1);
}

//Get frequency assignment
float FreqLookup(int FQ){
  switch(FQ){
    case 0:
      return RF95_FREQ_0;
      break;
    case 1:
      return RF95_FREQ_1;
      break;
    case 2:
      return RF95_FREQ_2;
      break;
    case 3:
      return RF95_FREQ_3;
      break;
    case 4:
      return RF95_FREQ_4;
      break;
    case 5:
      return RF95_FREQ_5;
      break;
    case 6:
      return RF95_FREQ_6;
      break;
    case 7:
      return RF95_FREQ_7;
      break;
    case 8:
      return RF95_FREQ_8;
      break;
    case 9:
      return RF95_FREQ_9;
      break;
    case 10:
      return RF95_FREQ_10;
      break;
    case 11:
      return RF95_FREQ_11;
      break;
    case 12:
      return RF95_FREQ_12;
      break;
    case 13:
      return RF95_FREQ_13;
      break;
    case 14:
      return RF95_FREQ_14;
      break;
    default:
      return 0;
      break;
  }
  return 0;
}
