//This includes some string data so as not to clutter the sketch
const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t GPGLLOff[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
const PROGMEM  uint8_t GPGSVOff[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
const PROGMEM  uint8_t GPVTGOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
//const PROGMEM  uint8_t GPRMCOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
const PROGMEM  uint8_t GPRMCOff[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17}; 
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
const PROGMEM  uint8_t SettingChangeBaud[] = {0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
const PROGMEM  uint8_t SaveSettings[] = {0xB5, 0x62, 0x06, 0x13, 0x00, 0x00, 0x19, 0x51, 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF}; //Save permanently
//const PROGMEM  uint8_t Reset[] = {0xB5, 0x62, 0x06, 0x13, 0x00, 0x00, 0x19, 0x51, 0xB5, 0x62, 0x06, 0x09, 0x0d, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE};
const PROGMEM  uint8_t Reset[] = {0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
//const PROGMEM  uint8_t SetBaud9600[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,0x00,0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x95};         
//const PROGMEM  uint8_t SetBaud9600[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
 //0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00,
 //0x00, 0x00, 0x00, 0x9E, 0x95};
 const PROGMEM  uint8_t SetBaud9600[] = {0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22, 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x9E, 0x95};
const PROGMEM  uint8_t SetAirbornMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F};
 
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

#define MAX_OP_CHAN 14

#define NUM_POINTS 18
#define INCREMENT_DEGS	20
#define NUM_POINTS_AR 8
#define INCREMENT_DEGS_AR	45

typedef struct {
  uint8_t x0;
  uint8_t y0;
  uint8_t x1;
  uint8_t y1;
  uint8_t x2;
  uint8_t y2;
} Tracker_struct;

// an arrow for each 20 degs
static const Tracker_struct Tracker_icon[NUM_POINTS] = {
{80,1,76,13,85,13},  //0
{102,5,92,15,102,18}, // 20
{120,16,109,22,117,29},
{135,33,121,34,127,43},
{142,53,129,50,131,60},
{142,75,131,68,129,78},
{135,96,127,85,121,94},
{120,112,117,99,109,106},
{102,123,102,110,92,113},
{80,127,85,115,75,115},
{58,123,68,113,58,110},
{40,112,51,106,43,99},
{25,96,39,94,33,85},
{18,75,31,78,29,68},
{18,53,29,60,31,50},
{25,33,33,43,39,34},
{40,16,43,29,51,22},
{58,5,58,18,68,15}};  // 340

// Support for arrow display
 #define DEG2RAD 0.0174532925
