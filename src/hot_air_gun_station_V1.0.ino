/*
 * Hot air gun controller based on atmega328 IC
 * Released November 5, 2018
 */
#include <Wire.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <CommonControls.h>
#include <EEPROM.h>
#include <SPI.h>
#include <max6675.h>

//Correcciones definidas para el perfil de simulador simulador de wokwi, sugiero retirarlas antes de probar en hardware físico
// #define ST7735_RED     0x07E0
// #define ST7735_GREEN   0x001F
// #define ST7735_BLUE    0xF800
// #define ST7735_YELLOW  0x07FF
// #define ST7735_CYAN   0xF81F

class configSCREEN;

const uint16_t temp_minC 	= 150;
const uint16_t temp_maxC	= 500;
const uint16_t temp_ambC    = 25;
const uint16_t temp_tip[3] = {200, 300, 400};                               // Temperature reference points for calibration

const uint8_t AC_SYNC_PIN   = 2;                                            // Outlet 220 v synchronization pin. Do not change!

// ---------------- HOT AIR ----------------
const uint8_t HOT_GUN_PIN   = 16;   // Reservado para el SSR/MOC3041
const uint8_t FAN_GUN_PIN   = 17;   // Reservado para PWM ventilador

// ---------------- ENCODER ----------------
const uint8_t R_MAIN_PIN    = 4;    // CLK
const uint8_t R_SECD_PIN    = 5;    // DT
const uint8_t R_BUTN_PIN    = 6;    // SW

// ---------------- AUX ----------------
const uint8_t BUZZER_PIN    = 7;
const uint8_t REED_SW_PIN   = 3;    // Temporal, revisar cuando migremos el reed

constexpr uint8_t COL_LEFT  = 20;
constexpr uint8_t COL_RIGHT = 104;

// ---------------- MAX6675 ----------------
const int PIN_MAX6675_DO  = 13;
const int PIN_MAX6675_CS  = 14;
const int PIN_MAX6675_CLK = 15;

MAX6675 sensorTermocupla(PIN_MAX6675_CLK, PIN_MAX6675_CS, PIN_MAX6675_DO);

static const char TXT_READY[]      PROGMEM = "READY";
static const char TXT_ADJUST[]     PROGMEM = "ADJUSTING...";
static const char TXT_SET[]        PROGMEM = "SET";
static const char TXT_STATE[]      PROGMEM = "STATE";
static const char TXT_CUR[]        PROGMEM = "CUR";
static const char TXT_MEAS[]       PROGMEM = "MEAS";
static const char TXT_POWER[]      PROGMEM = "POWER";
static const char TXT_FAN[]        PROGMEM = "FAN";
static const char TXT_SPEED[]      PROGMEM = "SPEED";
static const char TXT_HTR[]        PROGMEM = "HTR";
static const char TXT_OFF[]        PROGMEM = "OFF";
static const char TXT_ON[]         PROGMEM = "ON";
static const char TXT_HOLD[]       PROGMEM = "HOLD";
static const char TXT_COLD[]       PROGMEM = "COLD";
static const char TXT_CALIBRATION[] PROGMEM = "CALIBRATION";

#define FS(x) ((__FlashStringHelper*)(x))

// ---------------- TFT ----------------
#define TFT_CS_PIN   8
#define TFT_DC_PIN   9
#define TFT_RST_PIN 10

#define TFT_LABEL_SIZE (uint8_t)2
#define TFT_VALUE_SIZE (uint8_t)3
#define TFT_SMALL_SIZE (uint8_t)1

#define TFT_CHAR_W(s) (6 * (s))
#define TFT_CHAR_H(s) (8 * (s))

//#define SIMULATION_MODE //Se agrega para fines de poder corregir cualquier posible problema con la interfaz de la calibración.

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
 * uint32_t ID                           each time increment by 1
 * struct cfg                            config data, 8 bytes
 * byte CRC                              the checksum
*/
struct cfg {
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout
};

class CONFIG {
    public:
        CONFIG() {
            can_write     = false;
            buffRecords   = 0;
            rAddr = wAddr = 0;
            eLength       = 0;
            nextRecID     = 0;
            uint8_t rs = sizeof(struct cfg) + 5;                             // The total config record size
            // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
            for (record_size = 8; record_size < rs; record_size <<= 1);
        }
        void init();
        bool load(void);
        void getConfig(struct cfg &Cfg);                                    // Copy config structure from this class
        void updateConfig(struct cfg &Cfg);                                 // Copy updated config into this class
        bool save(void);                                                    // Save current config copy to the EEPROM
        bool saveConfig(struct cfg &Cfg);                                   // write updated config into the EEPROM

    protected:
        struct   cfg Config;

    private:
        bool     readRecord(uint16_t addr, uint32_t &recID);
        bool     can_write;                                                 // The flag indicates that data can be saved
        uint8_t  buffRecords;                                               // Number of the records in the outpt buffer
        uint16_t rAddr;                                                     // Address of thecorrect record in EEPROM to be read
        uint16_t wAddr;                                                     // Address in the EEPROM to start write new record
        uint16_t eLength;                                                   // Length of the EEPROM, depends on arduino model
        uint32_t nextRecID;                                                 // next record ID
        uint8_t  record_size;                                               // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
    eLength = EEPROM.length();
    uint32_t recID;
    uint32_t minRecID = 0xffffffff;
    uint16_t minRecAddr = 0;
    uint32_t maxRecID = 0;
    uint16_t maxRecAddr = 0;
    uint8_t  records = 0;

    nextRecID = 0;

    // read all the records in the EEPROM find min and max record ID
    for (uint16_t addr = 0; addr < eLength; addr += record_size) {
        if (readRecord(addr, recID)) {
            ++records;
            if (minRecID > recID) {
                minRecID = recID;
                minRecAddr = addr;
            }
            if (maxRecID < recID) {
                maxRecID = recID;
                maxRecAddr = addr;
            }
        } else {
            break;
        }
    }

    if (records == 0) {
        wAddr = rAddr = 0;
        can_write = true;
        return;
    }

    rAddr = maxRecAddr;
    if (records < (eLength / record_size)) {                                // The EEPROM is not full
        wAddr = rAddr + record_size;
        if (wAddr > eLength) wAddr = 0;
    } else {
        wAddr = minRecAddr;
    }
    can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
    memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
    memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
    updateConfig(Cfg);
    return save();                                                          // Save new data into the EEPROM
}

bool CONFIG::save(void) {
    if (!can_write) return can_write;
    if (nextRecID == 0) nextRecID = 1;

    uint16_t startWrite = wAddr;
    uint32_t nxt = nextRecID;
    uint8_t summ = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(startWrite++, nxt & 0xff);
        summ <<=2; summ += nxt;
        nxt >>= 8;
    }
    uint8_t* p = (byte *)&Config;
    for (uint8_t i = 0; i < sizeof(struct cfg); ++i) {
        summ <<= 2; summ += p[i];
        EEPROM.write(startWrite++, p[i]);
    }
    summ ++;                                                                // To avoid empty records
    EEPROM.write(wAddr+record_size-1, summ);

    rAddr = wAddr;
    wAddr += record_size;
    if (wAddr > EEPROM.length()) wAddr = 0;
    nextRecID ++;                                                           // Get ready to write next record
    return true;
}

bool CONFIG::load(void) {
    bool is_valid = readRecord(rAddr, nextRecID);
    nextRecID ++;
    return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
    uint8_t Buff[record_size];

    for (uint8_t i = 0; i < record_size; ++i) 
        Buff[i] = EEPROM.read(addr+i);
  
    uint8_t summ = 0;
    for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {
        summ <<= 2; summ += Buff[i];
    }
    summ ++;                                                                // To avoid empty fields
    if (summ == Buff[record_size-1]) {                                      // Checksumm is correct
        uint32_t ts = 0;
        for (char i = 3; i >= 0; --i) {
            ts <<= 8;
            ts |= Buff[byte(i)];
        }
        recID = ts;
        memcpy(&Config, &Buff[4], sizeof(struct cfg));
        return true;
    }
    return false;
}

//------------------------------------------ class HOT GUN CONFIG ----------------------------------------------
class HOTGUN_CFG : public CONFIG {
    public:
        HOTGUN_CFG()                                                        { }
        void     init(void);
        bool     isCold(uint16_t temp);                                     // Whether the HOT GUN temperature is low
        uint16_t tempPreset(void);                                          // The preset temperature in internal units
		uint8_t	 fanPreset(void);                                           // The preset fan speed 0 - 255 
        uint16_t tempInternal(uint16_t temp);                               // Translate the human readable temperature into internal value
        uint16_t tempHuman(uint16_t temp);                                  // Translate temperature from internal units to the Celsius
        void     save(uint16_t temp, uint8_t fan_speed);                     // Save preset temperature in the internal units and fan speed
        void     applyCalibrationData(uint16_t tip[3]);
        void     getCalibrationData(uint16_t tip[3]);
        void     saveCalibrationData(uint16_t tip[3]);
        void     setDefaults(bool Write);                                   // Set default parameter values if failed to load data from EEPROM
    private:
        uint16_t t_tip[3];
        const   uint16_t def_tip[3] = {587, 751, 850};                      // Default values of internal sensor readings at reference temperatures
        const   uint16_t min_temp  = 50;
        const   uint16_t max_temp  = 900;
        const   uint16_t def_temp  = 600;                                   // Default preset temperature
        const   uint8_t  def_fan   = 64;                                  	// Default preset fan speed 0 - 255
        const   uint16_t ambient_temp = 0;
        const   uint16_t ambient_tempC= 25;
};

void HOTGUN_CFG::init(void) {
    CONFIG::init();
    if (!CONFIG::load()) setDefaults(false);                                // If failed to load the data from EEPROM, initialize the config data with the default values
    uint32_t   cd = Config.calibration;
    t_tip[0] = cd & 0x3FF; cd >>= 10;                                       // 10 bits per calibration parameter, because the ADC readings are 10 bits
    t_tip[1] = cd & 0x3FF; cd >>= 10;
    t_tip[2] = cd & 0x3FF;
    // Check the tip calibration is correct
    if ((t_tip[0] >= t_tip[1]) || (t_tip[1] >= t_tip[2])) {
        setDefaults(false);
        for (uint8_t i = 0; i < 3; ++i)
            t_tip[i] = def_tip[i];
    }
    return;
}
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout

bool HOTGUN_CFG::isCold(uint16_t temp) {
    return (temp < t_tip[0]) && (map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]) < 50);
}

uint16_t HOTGUN_CFG::tempPreset(void) {
    return Config.temp;
}

uint8_t HOTGUN_CFG::fanPreset(void) {
    return Config.fan;
}

uint16_t HOTGUN_CFG::tempInternal(uint16_t t) {                             // Translate the human readable temperature into internal value
    uint16_t temp = t;
    t = constrain(t, temp_minC, temp_maxC);
    if (t >= temp_tip[1])
        temp = map(t+1, temp_tip[1], temp_tip[2], t_tip[1], t_tip[2]);
    else
        temp = map(t+1, temp_tip[0], temp_tip[1], t_tip[0], t_tip[1]);
 
    for (uint8_t i = 0; i < 10; ++i) {
        uint16_t tH = tempHuman(temp);
        if (tH <= t) break;
        --temp;
    }
    return temp;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Fahrenheit)
uint16_t HOTGUN_CFG::tempHuman(uint16_t temp) {
    uint16_t tempH = 0;
    if (temp <= ambient_temp) {
        tempH = ambient_tempC;
    } else if (temp < t_tip[0]) {
        tempH = map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]);
    } else if (temp >= t_tip[1]) {
        tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1], temp_tip[2]);
    } else {
        tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0], temp_tip[1]);
    }
    return tempH;
}

void HOTGUN_CFG::save(uint16_t temp, uint8_t fan_speed) {
    Config.temp        = constrain(temp, min_temp, max_temp);
    Config.fan         = fan_speed;
    CONFIG::save();                                                         // Save new data into the EEPROM
}

void HOTGUN_CFG::applyCalibrationData(uint16_t tip[3]) {
    if (tip[0] < ambient_temp) {
        uint16_t t = ambient_temp + tip[1];
        tip[0] = t >> 1;
    }
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    if (tip[2] > max_temp) tip[2] = max_temp; 
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::getCalibrationData(uint16_t tip[3]) {
    tip[0] = t_tip[0];
    tip[1] = t_tip[1];
    tip[2] = t_tip[2];
}

void HOTGUN_CFG::saveCalibrationData(uint16_t tip[3]) {
    if (tip[2] > max_temp) tip[2] = max_temp;
    uint32_t cd = tip[2] & 0x3FF; cd <<= 10;                                // Pack tip calibration data in one 32-bit word: 10-bits per value
    cd |= tip[1] & 0x3FF; cd <<= 10;
    cd |= tip[0];
    Config.calibration = cd;
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::setDefaults(bool Write) {
    uint32_t c = def_tip[2] & 0x3FF; c <<= 10;
    c |= def_tip[1] & 0x3FF;         c <<= 10;
    c |= def_tip[0] & 0x3FF;
    Config.calibration = c;
    Config.temp        = def_temp;
    Config.fan         = def_fan;
    if (Write) {
        CONFIG::save();
    }
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
    BUZZER(byte buzzerP)  { buzzer_pin = buzzerP; }
    void init(void);
    void shortBeep(void)  { digitalWrite(buzzer_pin, HIGH); delay(80);  digitalWrite(buzzer_pin, LOW); }
    void lowBeep(void)    { digitalWrite(buzzer_pin, HIGH); delay(160); digitalWrite(buzzer_pin, LOW); }
    void doubleBeep(void) { digitalWrite(buzzer_pin, HIGH); delay(160); digitalWrite(buzzer_pin, LOW); delay(150);
                            digitalWrite(buzzer_pin, HIGH); delay(160); digitalWrite(buzzer_pin, LOW);
                          }
    void failedBeep(void) { digitalWrite(buzzer_pin, HIGH); delay(170); digitalWrite(buzzer_pin, LOW); delay(10);
                            digitalWrite(buzzer_pin, HIGH); delay(80);  digitalWrite(buzzer_pin, LOW); delay(100);
                            digitalWrite(buzzer_pin, HIGH); delay(80);  digitalWrite(buzzer_pin, LOW);
                          }
  private:
    byte buzzer_pin;
};

void BUZZER::init(void) {
    pinMode(buzzer_pin, OUTPUT);
    noTone(buzzer_pin);
}

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
class DSPL {
    public:
        DSPL(void) : tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN) { }
        void    init(void);
        void    invalidateCache(void);
        void    clear(void)                                                 { tft.fillScreen(ST7735_BLACK); }
        void    drawStatic(void);
        void    drawCalibrationIntro(void);
        void    drawCalibrationLayout(bool ready);
        void    drawCalibrationLayoutBase();
        void    drawCalibrationSaved(uint8_t currentPoint);
        void    drawCalibrationModeLabel(bool ready);
        void    drawCalibrationComplete();
        void    drawHeader(const __FlashStringHelper* title, uint16_t titleBgColor);
        void    drawTemp(uint16_t t, uint8_t x, uint8_t y, uint8_t textSize);     //Show tempEratures
        void    drawPercent(uint16_t percentage, uint8_t x, uint8_t y, uint8_t textSize);  //show percentages
        void    drawState(const __FlashStringHelper* txtState, uint16_t color);   //Show the heater state
        void    printAt(int16_t x, int16_t y, const __FlashStringHelper* txt);
        void    printWColorAt(int16_t x, int16_t y, const __FlashStringHelper*txt, uint16_t color);
        void    printWSizeAt(int16_t x, int16_t y, const __FlashStringHelper* txt, uint8_t size);
        void    printWColorSizeAt(int16_t x, int16_t y, const __FlashStringHelper* txt, uint16_t color, uint8_t size);
        void    printAt(int16_t x, int16_t y, const char* txt); 
        void    printWColorAt(int16_t x, int16_t y, const char* txt, uint16_t color);
        void    printWSizeAt(int16_t x, int16_t y, const char*  txt, uint8_t size);
        void    printWColorSizeAt(int16_t x, int16_t y, const char*  txt, uint16_t color, uint8_t size);
        void    printValAt(int16_t x, int16_t y, const __FlashStringHelper* label, int16_t value);
        void    tSet(uint16_t t, uint8_t x, uint8_t y, uint8_t textSize);     // Show the preset temperature
        void    tCurr(uint16_t t, uint8_t x, uint8_t y, uint8_t textSize);    // Show the current temperature
        void    tInternal(uint16_t t, uint8_t x, uint8_t y,uint8_t textSize);                                      // Show the current temperature in internal units
        void    tReal(uint16_t t, uint8_t x, uint8_t y,uint8_t textSize);                                          // Show the real temperature in Celsius in calibrate mode
        void    fan_speed(uint8_t s, uint8_t x, uint8_t y, uint8_t textSize);                                        // Show the fan speed
		void	appliedPower(uint8_t p, uint8_t x, uint8_t y, uint8_t textSize, bool show_zero = true);			    // Show applied power (%)
        void    setupMode(uint8_t mode, bool forceRefresh = false);
        void    msgFail(void);                                              // Show 'Fail' message
        void    msgTune(void);
        void    showError(uint8_t errCode, uint16_t lastTemp, uint8_t fan, uint8_t pwr);                                              // Show 'Tune' message
    private:
		char 	temp_units;
        uint16_t last_set;
        uint16_t last_curr;
        uint8_t  last_fan;
        uint8_t  last_power;
        char     last_temp_units;
        bool     last_on_state;
        Adafruit_ST7735 tft;
};

void DSPL::init(void) {
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  temp_units = 'C';
  last_temp_units = 0;
  last_set = 0xffff;
  last_curr = 0xffff;
  last_fan = 0xff;
  last_power = 0xff;
  last_on_state = false;
}

void DSPL::invalidateCache(void) {

    last_set = 65535;
    last_curr = 65535;

    last_power = 255;
    last_fan = 255;
}

void DSPL::drawCalibrationIntro() {

    tft.fillScreen(ST7735_BLACK);
    drawHeader(FS(TXT_CALIBRATION), ST7735_BLUE);

    printWSizeAt(26, 42, F("This process will"), TFT_SMALL_SIZE);
    printAt(47, 54, F("calibrate 3"));
    printAt(53, 66, F("setpoints"));

    printWColorAt(22, 88, F("200C"), ST7735_YELLOW);
    printAt(68, 88, F("300C"));
    printAt(114, 88, F("400C"));

    tft.drawFastHLine(10, 104, 140, ST7735_BLUE);
    printWColorAt(14, 114, F("Press encoder to start"), ST7735_GREEN);
}

void DSPL::drawCalibrationLayoutBase() {

    tft.fillScreen(ST7735_BLACK);

    // ===== STATIC LABELS =====
    printWColorSizeAt(COL_LEFT, 24, TXT_SET, ST7735_YELLOW, TFT_LABEL_SIZE);
    printAt(COL_LEFT, 105, TXT_STATE);

    // ===== RIGHT PANEL =====
    printWSizeAt(COL_RIGHT, 24, TXT_FAN, TFT_SMALL_SIZE);
    printAt(COL_RIGHT, 34, TXT_SPEED);
    printAt(COL_RIGHT, 66, TXT_HTR);
    printAt(COL_RIGHT, 76, TXT_POWER);
}

void DSPL::drawCalibrationSaved(uint8_t currentPoint) {

    tft.fillScreen(ST7735_BLACK);

    drawHeader(F("POINT SAVED"), ST7735_GREEN);

    tft.setTextSize(TFT_LABEL_SIZE);
    tft.setTextColor(ST7735_WHITE);

    for (uint8_t i = 0; i < 3; i++) {
        tft.setCursor(32, 37 + (i * 28));
        if (i <= currentPoint)
            tft.print(F("[X] "));
        else
            tft.print(F("[ ] "));

        tft.print(temp_tip[i]);

        tft.print(temp_units);
    }
}

void DSPL::drawCalibrationComplete() {

    tft.fillScreen(ST7735_BLACK);

    drawHeader(F("DONE"), ST7735_GREEN);

    printWColorSizeAt(14, 54, TXT_CALIBRATION, ST7735_WHITE, TFT_LABEL_SIZE);

    printAt(50, 76, F("SAVED"));
}

void DSPL::drawCalibrationLayout(bool ready) {
    tft.fillScreen(ST7735_BLACK);
    drawHeader(FS(TXT_CALIBRATION), ST7735_BLUE);

    // Labels
    printWColorSizeAt(COL_LEFT, 30, TXT_SET, ST7735_YELLOW, TFT_LABEL_SIZE);
    
    printAt(COL_LEFT, 72, ready ? (PGM_P)TXT_MEAS : (PGM_P)TXT_CUR);

    printAt(COL_LEFT, 105, TXT_STATE);

    printWSizeAt(COL_RIGHT, 24, TXT_FAN, TFT_SMALL_SIZE);
    printAt(COL_RIGHT, 34, TXT_SPEED);
    printAt(COL_RIGHT, 66, TXT_HTR);
    printAt(COL_RIGHT, 76, TXT_POWER);
}

void DSPL::drawHeader(const __FlashStringHelper* title, uint16_t titleBgColor) {
    tft.fillRect(0, 0, 160, 18, titleBgColor);
    uint16_t w = strlen_P((PGM_P)title) * 6 * TFT_LABEL_SIZE;
    int16_t x = (160 - w) / 2;
    printWColorSizeAt(x, 2, title, ST7735_WHITE, TFT_LABEL_SIZE);
}

void DSPL::drawStatic(void) {
    // Draw all static elements (labels) once
    printWColorSizeAt(COL_LEFT, 5, TXT_SET, ST7735_YELLOW, TFT_LABEL_SIZE);
    printAt(COL_LEFT, 56, TXT_CUR);
    printAt(COL_LEFT, 105, TXT_STATE);
    printAt(COL_RIGHT, 5, TXT_FAN);
    printAt(COL_RIGHT, 56, TXT_HTR);
    printWSizeAt(COL_RIGHT, 23, TXT_SPEED, TFT_SMALL_SIZE);
    printAt(COL_RIGHT, 74, TXT_POWER);
}

void DSPL::printAt(int16_t x, int16_t y, const char* txt) { 
    tft.setCursor(x, y); 
    tft.print(FS(txt)); 
}

void DSPL::printWColorAt(int16_t x, int16_t y, const char* txt, uint16_t color){
    tft.setTextColor(color);
    printAt(x, y, txt);
}

void DSPL::printWSizeAt(int16_t x, int16_t y, const char* txt, uint8_t size){
    tft.setTextSize(size);
    printAt(x, y, txt);
}

void DSPL::printWColorSizeAt(int16_t x, int16_t y, const char* txt, uint16_t color, uint8_t size){
    tft.setTextColor(color);
    tft.setTextSize(size);
    printAt(x, y, txt);
}

void DSPL::printAt(int16_t x, int16_t y, const __FlashStringHelper* txt) { 
    tft.setCursor(x, y); 
    tft.print(txt);
}

void DSPL::printWColorAt(int16_t x, int16_t y, const __FlashStringHelper* txt, uint16_t color){
    tft.setTextColor(color);
    printAt(x, y, txt);
}

void DSPL::printWSizeAt(int16_t x, int16_t y, const __FlashStringHelper* txt, uint8_t size){
    tft.setTextSize(size);
    printAt(x, y, txt);
}

void DSPL::printWColorSizeAt(int16_t x, int16_t y, const __FlashStringHelper* txt, uint16_t color, uint8_t size){
    tft.setTextColor(color);
    tft.setTextSize(size);
    printAt(x, y, txt);
}

void DSPL::printValAt(int16_t x, int16_t y, const __FlashStringHelper* label, int16_t value) {
    tft.setCursor(x, y);
    tft.print(label); // Imprime "Temp: " y el cursor se mueve solo a la derecha
    tft.print(value); // Imprime el número (ej: 200) inmediatamente después
}

static void print3d_on_tft(Adafruit_ST7735 &tft, uint16_t value, uint8_t x, uint8_t y, uint16_t textSize)
{
    tft.setTextSize(textSize);
    if (value > 999)
        value = 999;

    uint16_t offset = 0;

    if (value < 10)
        offset = textSize * 12;

    else if (value < 100)
        offset = textSize * 6;

    tft.setCursor(x + offset, y);

    tft.print(value);
}

void DSPL::tSet(uint16_t t, uint8_t x, uint8_t y, uint8_t textSize) {
    if (t == last_set) return;
    last_set = t;
    drawTemp(t, x, y, textSize);
}

void DSPL::tCurr(uint16_t t, uint8_t x, uint8_t y, uint8_t textSize) {
    if (t == last_curr) return;
    last_curr = t; 
    drawTemp(t, x, y, textSize);
}

void DSPL::drawTemp(uint16_t t, uint8_t x, uint8_t y, uint8_t textSize){
    // Redraw only the current temperature area
    tft.fillRect(x, y, textSize*24, textSize*8, ST7735_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    print3d_on_tft(tft, t, x, y, textSize);
    tft.print(temp_units);
}

void DSPL::tInternal(uint16_t t, uint8_t x, uint8_t y,uint8_t textSize) {  
    tft.fillRect(x, y, textSize*24, textSize * 8, ST7735_BLACK);

    tft.setTextColor(ST7735_WHITE);

    if (t < 1023) {

        print3d_on_tft(tft, t, x, y, textSize);
        tft.print(F("C"));

    } else {
        printWSizeAt(x, y, F("xxxx"), textSize);
    }
}


void DSPL::tReal(uint16_t t, uint8_t x, uint8_t y,uint8_t textSize) {

    tft.fillRect(x,y,textSize * 24,textSize * 8,ST7735_BLACK);
    printWColorSizeAt(20, 83, F(">"), ST77XX_WHITE, textSize);

    print3d_on_tft(tft, t, x, y, textSize);

    tft.print(F("C"));
}

void DSPL::fan_speed(uint8_t s, uint8_t x, uint8_t y, uint8_t textSize) {
    uint8_t fanValue = map(s, 0, 255, 0, 99);
    if (fanValue == last_fan) return;
    last_fan = fanValue;
    
    // Redraw only the fan speed area
    drawPercent(fanValue, x, y, textSize);
}

void DSPL::appliedPower(uint8_t p, uint8_t x, uint8_t y, uint8_t textSize, bool show_zero) {

    if (p > 99) p = 99;

    if (p == 0 && !show_zero) {

        if (last_power != 0xff) {

            last_power = 0xff;

            tft.fillRect(x, y, textSize * 24, textSize * 8, ST7735_BLACK);
        }

        return;
    }

    if (p == last_power) return;

    last_power = p;

    // Redraw only the power area
    drawPercent(p, x, y, textSize);
}

void DSPL::drawCalibrationModeLabel(bool ready) {
    tft.fillRect(18, 66, 44, 14, ST7735_BLACK);
    printWColorSizeAt(COL_LEFT, 66, ready ? (PGM_P)TXT_MEAS : (PGM_P)TXT_CUR, ST7735_YELLOW, TFT_LABEL_SIZE);
}

void DSPL::drawPercent(uint16_t percentage, uint8_t x, uint8_t y, uint8_t textSize){
    // Redraw only the percentage temperature area
    tft.fillRect(x, y, textSize*24, textSize*8, ST7735_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    print3d_on_tft(tft, percentage, x, y, textSize);
    tft.print(F("%"));
}

void DSPL::drawState(const __FlashStringHelper* txtState, uint16_t color) {
    tft.fillRect(90, 105, TFT_LABEL_SIZE*30, TFT_LABEL_SIZE*8, ST7735_BLACK);
    printWColorSizeAt(90, 105, txtState, color, TFT_LABEL_SIZE);
}

void DSPL::setupMode(byte mode, bool forceRefresh) {
    // Almacena la posición del selector del menú actual en el que estamos trabajando
    static byte currentMenuSelector = 255; 
    const uint8_t yPos[] = {25, 45, 65, 85, 105};

    // Si la pantalla nos pide un rediseño total (porque entramos de cero)
    if (forceRefresh) {
        tft.fillScreen(ST7735_BLACK);
        printWColorSizeAt(50, 5, F("SETUP"), ST7735_GREEN, TFT_LABEL_SIZE);
        printWColorAt(15, yPos[0], F("Calibrate"), ST7735_WHITE);
        printAt(15, yPos[1], F("Heater Test"));
        printAt(15, yPos[2], F("Save config"));
        printAt(15, yPos[3], F("Exit"));
        printAt(15, yPos[4], F("Reset config"));
        
        // Dibujamos el selector inicial
        tft.setTextColor(ST7735_YELLOW);
        tft.setCursor(2, yPos[mode]);
        tft.write(16);
        
        currentMenuSelector = mode;
        return;
    }

    // Si no se fuerza el refresco y el modo es el mismo, no hacemos nada
    if (mode == currentMenuSelector) return;

    // Borrar selector anterior SOLO si era una posición válida
    if (currentMenuSelector != 255) {
        tft.fillRect(2, yPos[currentMenuSelector], 12, 16, ST7735_BLACK);
    }

    // Dibujar nuevo selector
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(2, yPos[mode]);
    tft.write(16);

    currentMenuSelector = mode;
}

void DSPL::msgFail(void) {
    tft.fillScreen(ST7735_BLACK);
    printWSizeAt(COL_LEFT, 8, F("-== Failed ==-"), TFT_SMALL_SIZE);
}

void DSPL::msgTune(void) {

    tft.fillScreen(ST7735_BLACK);
    drawHeader(F("Heater Test"), ST7735_BLUE);

    // Labels

    printWColorSizeAt(COL_LEFT, 25, TXT_CUR, ST7735_YELLOW, TFT_LABEL_SIZE);
    
    printAt(COL_LEFT, 105, TXT_STATE);

    printWSizeAt(COL_RIGHT, 23, TXT_FAN, TFT_SMALL_SIZE);
    printAt(COL_RIGHT, 33, TXT_SPEED);
    printAt(COL_RIGHT, 64, TXT_HTR);
    printAt(COL_RIGHT, 74, TXT_POWER);
}

void DSPL::showError(uint8_t errCode, uint16_t lastTemp, uint8_t fan, uint8_t pwr) {
    tft.fillScreen(ST7735_BLACK);
    drawHeader(F("ALERT"), ST7735_RED);
  
  // Icono central
  tft.setTextColor(ST7735_RED, ST7735_BLACK);
  printWSizeAt(100, 45, F("!"), 5);
  
  // Descripción del error
  tft.setTextColor(ST7735_RED, ST7735_BLACK);
  tft.setTextSize(TFT_SMALL_SIZE);
  tft.setCursor(5, 25);
  
  switch(errCode) {
    case 1: tft.print(F("AC synchronization failure")); break;
    case 2: tft.print(F("Temperature reading failure")); break;
    case 3: tft.print(F("Overheating detected")); break;
    default: tft.print(F("Uncatalogued error")); break;
  }
  
  // Datos de contexto para diagnóstico
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
  printValAt(5, 45, F("Temp: "), lastTemp);
  printValAt(5, 65, F("Fan: "), fan);
  printValAt(5, 85, F("Htr power: "), pwr);

    // Línea separadora inferior
  tft.drawFastHLine(10, 104, 140, ST7735_GREEN);
  
  // Instrucción de acción
  printWColorSizeAt(11, 114, F("Press encoder to return"), ST7735_YELLOW, TFT_SMALL_SIZE);
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
	public:
		HISTORY(void)                               						{ len = 0; }
		void     init(void)                         						{ len = 0; }
		uint16_t last(void);
		uint16_t top(void)                          						{ return queue[0]; }
		void     put(uint16_t item);                						// Put new entry to the history
		uint16_t average(void);                     						// calculate the average value
        float    dispersion(void);                                          // calculate the math dispersion
	private:
		volatile uint16_t queue[H_LENGTH];
		volatile byte len;                          						// The number of elements in the queue
		volatile byte index;                        						// The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
	if (len < H_LENGTH) {
		queue[len++] = item;
	} else {
		queue[index ] = item;
		if (++index >= H_LENGTH) index = 0;         						// Use ring buffer
	}
}

uint16_t HISTORY::last(void) {
    if (len == 0) return 0;
	uint8_t i = len - 1;
	if (index)
		i = index - 1;
	return queue[i];
}

uint16_t HISTORY::average(void) {
	uint32_t sum = 0;
    if (len == 0) return 0;
	if (len == 1) return queue[0];
	for (uint8_t i = 0; i < len; ++i) sum += queue[i];
	sum += len >> 1;                              							// round the average
	sum /= len;
	return uint16_t(sum);
}

float HISTORY::dispersion(void) {
    if (len < 3) return 1000;
    uint32_t sum = 0;
    uint32_t avg = average();
    for (uint8_t i = 0; i < len; ++i) {
        long q = queue[i];
        q -= avg;
        q *= q;
        sum += q;
    }
    sum += len << 1;
    float d = (float)sum / (float)len;
    return d;
}

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algorithm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formula is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 *  
 *  PID coefficients history:
 *  10/14/2017  [768, 32, 328]
 */
class PID {
    public:
        PID(void) {
            Kp = 638;
            Ki = 196;
            Kd =   1;
        }
        void resetPID(int temp = -1);                                       // reset PID algorithm history parameters
        // Calculate the power to be applied
        long reqPower(int temp_set, int temp_curr);
        int  changePID(uint8_t p, int k);                                   // set or get (if parameter < 0) PID parameter
    private:
        void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
        int   temp_h0, temp_h1;                                             // previously measured temperature
        bool  pid_iterate;                                                  // Whether the iterative process is used
        long  i_summ;                                                       // Ki summary multiplied by denominator
        long  power;                                                        // The power iterative multiplied by denominator
        long  Kp, Ki, Kd;                                                   // The PID algorithm coefficients multiplied by denominator
        const byte denominator_p = 11;                                      // The common coefficient denominator power of 2 (11 means divide by 2048)
};

void PID::resetPID(int temp) {
    temp_h0 = 0;
    power  = 0;
    i_summ = 0;
    pid_iterate = false;
    if ((temp > 0) && (temp < 1000))
        temp_h1 = temp;
    else
        temp_h1 = 0;
}

int PID::changePID(uint8_t p, int k) {
    switch(p) {
        case 1:
            if (k >= 0) Kp = k;
            return Kp;
        case 2:
            if (k >= 0) Ki = k;
            return Ki;
        case 3:
            if (k >= 0) Kd = k;
            return Kd;
        default:
        break;
    }
    return 0;
}

long PID::reqPower(int temp_set, int temp_curr) {
    if (temp_h0 == 0) {
        // When the temperature is near the preset one, reset the PID and prepare iterative formula                        
        if ((temp_set - temp_curr) < 30) {
            if (!pid_iterate) {
                pid_iterate = true;
                power = 0;
                i_summ = 0;
            }
        }
        i_summ += temp_set - temp_curr;                                     // first, use the direct formula, not the iterate process
        power = Kp*(temp_set - temp_curr) + Ki*i_summ;
    // If the temperature is near, prepare the PID iteration process
    } else {
        long kp = Kp * (temp_h1 - temp_curr);
        long ki = Ki * (temp_set - temp_curr);
        long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
        long delta_p = kp + ki + kd;
        power += delta_p;                                                   // power kept multiplied by denominator!
    }
    if (pid_iterate) temp_h0 = temp_h1;
    temp_h1 = temp_curr;
    long pwr = power + (1 << (denominator_p-1));                            // prepare the power to delete by denominator, round the result
    pwr >>= denominator_p;                                                  // delete by the denominator
    return pwr;
}

//--------------------- High frequency PWM signal calss on D9 pin ------------------------- ---------------
class FastPWM_D9 {
    public:
        FastPWM_D9()                                { }
        void init(void);
        void duty(uint8_t d)                        { OCR1A = d; }
};

void FastPWM_D9::init(void) {
    pinMode(FAN_GUN_PIN, OUTPUT);
    digitalWrite(FAN_GUN_PIN, LOW);
    noInterrupts();
    TCNT1   = 0;
    TCCR1B  = _BV(WGM13);                           // set mode as phase and frequency correct pwm, stop the timer
    TCCR1A  = 0;
    ICR1    = 256;
    TCCR1B  = _BV(WGM13) | _BV(CS10);               // Top value = ICR1, prescale = 1
    TCCR1A |= _BV(COM1A1);                          // XOR D9 on OCR1A, detached from D10
    OCR1A   = 0;                                    // Switch-off the signal on pin 9;
    interrupts();
}

//--------------------- Hot air gun manager using total sine shape to power on the hardware ---------------
class HOTGUN : public PID {
    public:
        HOTGUN(uint8_t HG_pwr_pin) {
            gun_pin = HG_pwr_pin;
        }

        void        init(void);
		bool		isOn(void)												{ return on; }
		void		setTemp(uint16_t t)										{ temp_set = t; }
		uint16_t	getTemp(void)											{ return temp_set; }
        float       readTemp(void);
		float       getCurrTemp(void)										{ return h_temp.last(); }
		uint16_t 	tempAverage(void)                  						{ return h_temp.average(); }
        uint8_t     powerAverage(void)                                      { return h_power.average(); }
		uint8_t     appliedPower(void)                						{ return actual_power; }
		void		setfan_speed(uint8_t f)									{ fan_speed = f; if (on) hg_fan.duty(f); }
		uint8_t	    getfan_speed(void)   									{ return fan_speed; }
        uint16_t    tempDispersion(void)                                    { return h_temp.dispersion(); }
        void        switchPower(bool On);
        void        fixPower(uint8_t Power);                                // Set the specified power to the the hot gun
		void     	keepTemp(void);
        bool        areExternalInterrupts(void)                             { return millis() - last_period < period * 10; }
        uint8_t     getMaxFixedPower(void)                                  { return period; }
        bool        syncCB(void);											// Return true at the end of the power period
    private:
        FastPWM_D9  hg_fan;
		long     	power;                             						// The hot air gun power, calculated by the PID algorithm
		uint16_t	temp_set;												// The preset temperature of the hot air gun (internal units)
		float       temp_curr;												// The current temperature of the hot air gun
		uint8_t		fan_speed;
		uint8_t		gun_pin;
		HISTORY  	h_power;                           						// The history queue of power applied values
		HISTORY  	h_temp;                            						// The history queue of the temperature
        uint32_t cronometroMax;
		volatile    uint8_t     cnt;
        volatile    uint8_t     actual_power;
        volatile    bool        active;
        bool        on, fan, fix_power;
        bool        chill;                                                  // To chill the hot gun
        uint32_t    last_period;                                            // The time in ms when the counter reset
        const       uint8_t     period 			= 100;
		const		uint8_t		min_fan_speed	= 30;
        const       uint16_t    temp_gun_cold   = 80;                       // The temperature of the cold iron 
        uint32_t heatStartTime = 0;
        float heatStartTemp = 0;
};

void HOTGUN::init(void) {
    cnt         = 0;
    power       = 0;
    active      = false;
    on          = false;
    fan         = false;
    fix_power   = false;
    chill       = false;
    last_period = 0;
    cronometroMax=0;
    pinMode(gun_pin, OUTPUT);
	digitalWrite(gun_pin, LOW);
    hg_fan.init();
	h_temp.init();
    resetPID();
}

bool HOTGUN::syncCB(void) {
    if (++cnt >= period) {
        cnt = 0;
        last_period = millis();                                             // Save the current time to check the external interrupts
        if (!active && (actual_power > 0)) {
            digitalWrite(gun_pin, HIGH);
            active = true;
        }
    } else if (cnt >= actual_power) {
        if (active) {
            digitalWrite(gun_pin, LOW);
            active = false;
        }
    }
	return (cnt == 0);														// End of the Power period (period AC voltage shapes)
}

void HOTGUN::switchPower(bool On) {
	on = On;
	if (!on) {
		digitalWrite(gun_pin, LOW);
        actual_power = 0;
	} else {
	    if (fan_speed < min_fan_speed)
		    fan_speed = min_fan_speed;
		hg_fan.duty(fan_speed);
        fan = true;
	}
}

// This routine is used to keep the hot air gun temperature near required value
void HOTGUN::keepTemp(void) {

    float temp = readTemp();             						// Check the hot air gun temperature

    if (actual_power > 20 && heatStartTime == 0) {
        heatStartTime = millis();
        heatStartTemp = temp;
    }

    //Thermal Runaway Protection for MAX6675
    if (temp > temp_set + 80) {
        actual_power = 0;
        digitalWrite(gun_pin, LOW);
        on = false;
        return;
    }

    if (heatStartTime != 0) {

        if (millis() - heatStartTime > 8000) {

            if ((temp - heatStartTemp) < 10) {

                actual_power = 0;

                digitalWrite(gun_pin, LOW);

                on = false;

                return;
            }

            heatStartTime = 0;
        }
    }

    static uint32_t lastPID = 0;

    if (millis() - lastPID < 300)
        return;

    lastPID = millis();

    h_temp.put((uint16_t)temp);
    if (!chill && on && temp > temp_set + 20) {
        digitalWrite(gun_pin, LOW);
        actual_power = 0;
        chill = true;
    }

	if (on) {
		if (chill) {
			if (temp < (temp_set - 8)) {
				chill = false;
				resetPID();
			} else {
				power = 0;
				actual_power = 0;
                return;
			}
		}
		power = reqPower(temp_set, temp);           						// Use PID algorithm to calculate power to be applied
		actual_power = constrain(power, 0, period);
		h_power.put(actual_power);
	} else {
        if (!fix_power) {
		    actual_power = 0;
		    digitalWrite(gun_pin, LOW);
        }
	}

    // Keep fan running till the hot gun become cold
    if (fan) {
        if ((actual_power == 0) && temp <= temp_gun_cold) {                // Switch off the fan when the gun become cold
            hg_fan.duty(0);
            fan = false;
        }
    } else {
        if (temp > temp_gun_cold) {
            hg_fan.duty(fan_speed);
            fan = true;
        }
    }
}

void HOTGUN::fixPower(uint8_t Power) {
    if (Power == 0) {                                                       // To switch off the hot gun, set the Power to 0
        fix_power = false;
        actual_power = 0;
        return;
    }

    if (Power > period) Power = period;
    power = Power;
    actual_power = power;
    fix_power = true;
}

float HOTGUN::readTemp(void) {

    static uint16_t ultimaTemperaturaValida = 25;
    static uint16_t tempFiltrada = 25;

    if (millis() - cronometroMax >= 300) {

        cronometroMax = millis();

        float lectura = sensorTermocupla.readCelsius();

        // Validación de seguridad
        if (!isnan(lectura) && lectura > 0 && lectura < 600) {

            // Filtro exponencial
            tempFiltrada = (tempFiltrada * 85 + (uint16_t)lectura * 15) / 100;

            ultimaTemperaturaValida = tempFiltrada;
        }
        else {

            // ERROR SENSOR
            actual_power = 0;
            digitalWrite(gun_pin, LOW);
            on=false;
        }
    }

    return ultimaTemperaturaValida;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
	public:
		SCREEN* next;                               						// Pointer to the next screen
		SCREEN() {
			next			= 0;
			update_screen  	= 0;
			scr_timeout    	= 0;
			time_to_return 	= 0;
		}
		virtual void    init(void)                     						{ }
		virtual SCREEN* show(void)                  						{ return this; }
		virtual SCREEN* menu(void)                  						{ return this; }
		virtual SCREEN* menu_long(void)             						{ if (this->next != 0)  return this->next;  else return this; }
        virtual SCREEN* reedSwitch(bool on)                                 { return this; }    
		virtual void    rotaryValue(int16_t value)     						{ }
		void            forceRedraw(void)                   				{ update_screen = 0; }
	protected:
		uint32_t update_screen;                     						// Time in ms when the screen should be updated
		uint32_t scr_timeout;                       						// Timeout is sec. to return to the main screen, canceling all changes
		uint32_t time_to_return;                    						// Time in ms to return to main screen
};

//---------------------------------------- class mainSCREEN [the hot air gun is OFF] ---------------------------
class mainSCREEN : public SCREEN {
	public:
		mainSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
			pHG 	= HG;
			pD      = DSP;
			pEnc    = ENC;
			pBz     = Buzz;
			pCfg    = Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* reedSwitch(bool on);
		virtual void	rotaryValue(int16_t value); 						// Setup the preset temperature
        SCREEN*     on;                                                     // Screen mode when the power is
	private:
		HOTGUN*		pHG;                            						// Pointer to the hot air gun instance
		DSPL*     	pD;                               						// Pointer to the DSPLay instance
		ENCODER*	pEnc;                             						// Pointer to the rotary encoder instance
		BUZZER*   	pBz;                              						// Pointer to the simple buzzer instance
		HOTGUN_CFG* pCfg;                             						// Pointer to the configuration instance
		uint32_t  	clear_used_ms;                    						// Time in ms when used flag should be cleared (if > 0)
		bool		mode_temp;												// Preset mode: change temperature or change fan speed
		bool      	used;                             						// Whether the IRON was used (was hot)
		bool      	cool_notified;                    						// Whether there was cold notification played
		const uint16_t period 				= 1000;               			// The period to update the screen
		const uint32_t cool_notify_period 	= 120000; 						// The period to display 'cool' message (ms)
		const uint16_t show_temp 			= 20000;           				// The period to show the preset temperature (ms)
};

void mainSCREEN::init(void) {
    pD->invalidateCache();
    pD->clear();
    pD->drawStatic();
	pHG->switchPower(false);
	uint16_t temp_set 	= pCfg->tempPreset(); 
	uint16_t tempH 	    = pCfg->tempHuman(temp_set);         				// The preset temperature in the human readable units
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
    pEnc->write(tempH);
	used = !pCfg->isCold(pHG->tempAverage());
	cool_notified = !used;
	if (used) {                                   							// the hot gun was used, we should save new data in EEPROM
		pCfg->save(temp_set, pHG->getfan_speed());
	}
	mode_temp = true;
	clear_used_ms = 0;
	forceRedraw();
}

void mainSCREEN::rotaryValue(int16_t value) {
	if (mode_temp) {														// set hot gun temperature
		uint16_t temp = pCfg->tempInternal(value);
		pHG->setTemp(temp);
		pD->tSet(value, 20, 25, TFT_VALUE_SIZE);
	} else {																// set fan speed
		pHG->setfan_speed(value);
		pD->fan_speed(value, 92, 35, TFT_LABEL_SIZE);
	}
	update_screen  = millis() + period;
}

SCREEN* mainSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;

	if (clear_used_ms && (millis() > clear_used_ms)) {
		clear_used_ms = 0;
		used = false;
	}

    // --- CORRECCIÓN: Tomamos el valor actual del encoder para pintar el "SET" ---
    uint16_t tempH_set = pEnc->read();
    pD->tSet(tempH_set, 20, 25, TFT_VALUE_SIZE);
	uint16_t temp  = pHG->tempAverage();
	uint16_t tempH = pCfg->tempHuman(temp);
	if (pCfg->isCold(temp)) {
		if (used) {
			pD->drawState(FS(TXT_COLD), ST7735_BLUE);
		} else {
			pD->drawState(FS(TXT_OFF), ST7735_RED);
		}
		if (used && !cool_notified) {
		    pBz->lowBeep();
		    cool_notified = true;
		    clear_used_ms = millis() + cool_notify_period;
		}
	} else {
        pD->drawState(FS(TXT_OFF), ST7735_RED);
	}
	pD->tCurr(tempH, 20, 76, TFT_VALUE_SIZE);
    pD->appliedPower(0, 92, 86, TFT_LABEL_SIZE, false);
    pD->fan_speed(pHG->getfan_speed(), 92, 35, TFT_LABEL_SIZE);
	return this;
}

SCREEN* mainSCREEN::menu(void) {
	if (mode_temp) {                                                        // Prepare to adjust the fan speed
		uint8_t	fan_speed = pHG->getfan_speed();
		pEnc->reset(fan_speed, 0, 255, 5, 20);
		mode_temp = false;
	} else {                                                                // Prepare to adjust the preset temperature
		uint16_t temp_set   = pHG->getTemp();
		uint16_t tempH 	    = pCfg->tempHuman(temp_set);
		pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
		mode_temp = true;
	}
    return this;
}

SCREEN* mainSCREEN::menu_long(void) {
    // Corrección: Devuelve directamente la pantalla de configuración
    extern configSCREEN cfgScr; 
    return (SCREEN*)&cfgScr; 
}

SCREEN* mainSCREEN::reedSwitch(bool on) {
    if (on && this->on)
        return this->on;
    return this; 
}

//---------------------------------------- class workSCREEN [the hot air gun is ON] ----------------------------
class workSCREEN : public SCREEN {
	public:
		workSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
			update_screen = 0;
			pHG 	= HG;
			pD    	= DSP;
			pBz   	= Buzz;
			pEnc  	= Enc;
			pCfg  	= Cfg;
		}
		virtual void    init(void);
		virtual SCREEN* show(void);
		virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* reedSwitch(bool on);
		virtual void    rotaryValue(int16_t value); 						// Change the preset temperature
	private:
		HOTGUN*     pHG;                            						// Pointer to the IRON instance
		DSPL*     	pD;                               						// Pointer to the DSPLay instance
		BUZZER*   	pBz;                              						// Pointer to the simple Buzzer instance
		ENCODER*  	pEnc;                             						// Pointer to the rotary encoder instance
		HOTGUN_CFG* pCfg;                             						// Pointer to the configuration instance
		bool      	ready;                            						// Whether the IRON have reached the preset temperature
		bool		mode_temp;												// Preset mode: temperature or fan speed
		const uint16_t period = 1000;               						// The period to update the screen (ms) 
};

void workSCREEN::init(void) {
    pD->invalidateCache();
    pD->clear();
    pD->drawStatic();
	uint8_t fan_speed = pHG->getfan_speed();
    pEnc->reset(fan_speed, 0, 255, 5, 20);
    mode_temp   = false;                                                    // By default adjust the fan speed
	pHG->switchPower(true);
	ready = false;
	forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {   							// Setup new preset temperature by rotating the encoder
	if (mode_temp) {
        ready = false;
		uint16_t temp = pCfg->tempInternal(value);      				    // Translate human readable temperature into internal value
		pHG->setTemp(temp);
		pD->tSet(value, 20, 25, TFT_VALUE_SIZE);
	} else {
		pHG->setfan_speed(value);
		pD->fan_speed(value, 92, 35, TFT_LABEL_SIZE);
	}
	update_screen = millis() + period;
}

SCREEN* workSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;

    int temp_set  = pHG->getTemp();
    int tempH_set = pCfg->tempHuman(temp_set);
    pD->tSet(tempH_set, 20, 25, TFT_VALUE_SIZE);
    int temp      = pHG->tempAverage();
    int tempH     = pCfg->tempHuman(temp);
    pD->tCurr(tempH, 20, 76, TFT_VALUE_SIZE);
    pD->drawState(FS(TXT_ON), ST7735_GREEN);
	uint8_t p 	= pHG->appliedPower();
	pD->appliedPower(p, 92, 86, TFT_LABEL_SIZE);
    pD->fan_speed(pHG->getfan_speed(), 92, 35, TFT_LABEL_SIZE);

    
Serial.print(F("Diff = ")); Serial.print(temp_set - temp);
uint32_t disp = pHG->tempDispersion();
Serial.print(F(", Dispersion = ")); Serial.println(disp);

    if ((abs(temp_set - temp) < 5) && (pHG->tempDispersion() <= 20))  {
        if (!ready) {
            pBz->shortBeep();
            ready = true;
            pD->drawState(FS(TXT_HOLD), ST7735_CYAN);
            Serial.println(FS(TXT_READY));
            update_screen = millis() + (period << 2);
            return this;
        }
    }
	return this;
}

SCREEN* workSCREEN::menu(void) {
	if (mode_temp) {
		uint8_t	fan_speed = pHG->getfan_speed();
		pEnc->reset(fan_speed, 0, 255, 5, 20);
		mode_temp = false;
	} else {
		uint16_t temp_set   = pHG->getTemp();
		uint16_t tempH 	    = pCfg->tempHuman(temp_set);
		pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
		mode_temp = true;
	}
    return this;
}

SCREEN* workSCREEN::menu_long(void) {
    // Corrección: Devuelve directamente la pantalla de configuración
    extern configSCREEN cfgScr; 
    return (SCREEN*)&cfgScr; 
}

SCREEN* workSCREEN::reedSwitch(bool on) {
    if (!on && next)
        return next;
    return this; 
}

//---------------------------------------- class errorSCREEN [the error detected] ------------------------------

class errorSCREEN : public SCREEN {
public:
  errorSCREEN(HOTGUN* HG, DSPL* DSP, BUZZER* Buzz) {
    pHG = HG; pD = DSP; pBz = Buzz;
  }
  virtual void    init(void);
  virtual SCREEN* show(void)  { return this; } // No necesita redibujado
  virtual SCREEN* menu(void)  { if (this->next != 0) return this->next; else return this; }
private:
  HOTGUN*     pHG;
  DSPL*       pD;
  BUZZER*     pBz;
};

void errorSCREEN::init(void) {
  // Captura el estado EXACTO en el instante del fallo
  uint16_t temp = pHG-> tempAverage();
  uint8_t  fan_speed = pHG->getfan_speed();
  uint8_t  power = pHG->appliedPower();

  pHG->switchPower(false); // Apaga hardware inmediatamente
 
  pD->showError(1, temp, fan_speed, power); // 1; // Pinta el panel diagnóstico
  pBz->failedBeep();      // Alerta sonora
}

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
    public:
        configSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual SCREEN* menu(void);
        virtual void    rotaryValue(int16_t value);
        SCREEN*         calib;                                              // Pointer to the calibration SCREEN
        SCREEN*         tune;                                               // Pointer to the tune SCREEN
    private:
        HOTGUN*     pHG;                                                    // Pointer to the HOTGUN instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the config instance
        uint8_t     mode;                                                   // 0 - hotgun calibrate, 1 - tune, 2 - save, 3 - cancel, 4 - defaults
        uint8_t last_mode;
        const uint16_t period = 10000;                                      // The period in ms to update the screen
};

void configSCREEN::init(void) {
    last_mode = 255;
    pHG->switchPower(false);
    mode = 0;
    pEnc->reset(mode, 0, 4, 1, 0, true);
    this->scr_timeout = 30;
    update_screen = 0; // Forzar dibujo inmediato
}

SCREEN* configSCREEN::show(void) {

    if (millis() < update_screen) return this;

    update_screen = millis() + 500;

    // Dibujar pantalla COMPLETA solo la primera vez

    if (last_mode == 255) {

        pD->clear();

        pD->setupMode(mode, true);

        last_mode = mode;

        return this;
    }

    // Luego SOLO mover selector

    if (last_mode != mode) {

        pD->setupMode(mode, false);

        last_mode = mode;
    }

    return this;
}

SCREEN* configSCREEN::menu(void) {

    switch (mode) {

        case 0:                                                             // calibrate hotgun
            last_mode = 255;
            if (calib) return calib;
            break;

        case 1:                                                             // Tune potentiometer
            last_mode = 255;
            if (tune) return tune;
            break;

        case 2:                                                             // Save configuration data
            pCfg->save(pCfg->tempPreset(), pHG->getfan_speed());
            last_mode = 255;
            if (next) return next;
            break;

        case 3:                                                             // Cancel, Return to the main menu
            last_mode = 255;
            if (next) return next;
            break;

        case 4:                                                             // Save defaults
            pCfg->setDefaults(true);
            last_mode = 255;
            if (next) return next;
            break;
    }

    forceRedraw();

    return this;
}

void configSCREEN::rotaryValue(int16_t value) {
    mode = value;
}

//---------------------------------------- class calibSCREEN [ tip calibration ] -------------------------------
class calibSCREEN : public SCREEN {
    public:
        calibSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
            pBz     = Buzz;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
    private:
        uint16_t        selectTemp(byte index);                             // Calculate the value of the temperature limit depending on mode
        void            buildCalibration(uint16_t tip[3]);
        void invalidateStateCache(void);
        HOTGUN*         pHG;                                                // Pointer to the HOTGUN instance
        DSPL*           pD;                                                 // Pointer to the DSPLay instance
        ENCODER*        pEnc;                                               // Pointer to the rotary encoder instance
        HOTGUN_CFG*     pCfg;                                               // Pointer to the config instance
        BUZZER*         pBz;                                                // Pointer to the buzzer instance
        uint8_t         mode;                                               // Which parameter to change: t_min, t_mid, t_max
        uint8_t last_mode;
        uint16_t        calib_temp[2][3];                                   // Calibration temperature data measured at each of calibration points (Celsius, internal temp)
        uint16_t        preset_temp;                                        // The preset temp in human readable units
        uint16_t temp_backup_calib;
        bool            ready;                                              // Whether the temperature has been established
        bool            tune;                                               // Whether the parameter is modifiying
        bool last_tune;
        bool last_ready;
        bool started;
        const uint32_t  period   = 1000;                                    // Update screen period
        const uint16_t  temp_max = 900;
};

void calibSCREEN::init(void) {
    temp_backup_calib = pCfg->tempPreset();

    started=false;

    mode = 0;

    last_mode = 255;

    pHG->switchPower(false);

    tune  = false;
    ready = false;

    last_tune  = false;
    last_ready = false;

    for (uint8_t i = 0; i < 3; ++i)
        calib_temp[0][i] = temp_tip[i];

    pCfg->getCalibrationData(&calib_temp[1][0]);

    pD->clear();

    // Draw initial layout
    pD->drawCalibrationIntro();

    pD->invalidateCache();

    uint16_t temp = temp_tip[mode];

    preset_temp = pHG->getTemp();
    preset_temp = pCfg->tempHuman(preset_temp);

    uint16_t temp_set = pCfg->tempInternal(temp);

    pHG->setTemp(temp_set);

    forceRedraw();
}

SCREEN* calibSCREEN::show(void) {
    if (!started) {
        return this;
    }

    if ((tune != last_tune) || (ready != last_ready) || (mode != last_mode)) {

        last_tune  = tune;
        last_ready = ready;
        last_mode  = mode;

        pD->drawCalibrationLayoutBase();

        if (ready)
            pD->drawHeader(FS(TXT_READY), ST7735_GREEN);
        else
            pD->drawHeader(FS(TXT_ADJUST), ST7735_BLUE);

        pD->drawCalibrationModeLabel(ready);

        pD->invalidateCache();
    }

    // Refresh timer
    if (millis() < update_screen)
        return this;

    update_screen = millis() + period;

    // Temperature readings
    int temp       = pHG->tempAverage();
    int temp_set   = pHG->getTemp();

    uint16_t tempH     = pCfg->tempHuman(temp);
    uint16_t temp_setH = pCfg->tempHuman(temp_set);

    // Always show SET temperature
    pD->tSet(temp_setH, 20, 45, TFT_LABEL_SIZE);

    // =========================
    // Selection state
    // =========================
    if (!tune) {

        pD->drawState(FS(TXT_OFF), ST7735_RED);
    }

    // =========================
    // Heating state
    // =========================
    if (tune && !ready) {

        pD->drawState(FS(TXT_ON), ST7735_GREEN);

        pD->tCurr(tempH, 20, 86, TFT_LABEL_SIZE);

        uint8_t p = pHG->appliedPower();

        if (!pHG->isOn())
            p = 0;
        pD->appliedPower(p, 92, 86, TFT_LABEL_SIZE);
        pD->fan_speed(pHG->getfan_speed(), 92, 45, TFT_LABEL_SIZE);

        #ifdef SIMULATION_MODE

        if(p>=0){
            pBz->shortBeep();
            ready=true;
        }

        #else
        // Detect stabilized temperature
        if ((abs(temp_set - temp) < 5) &&
            (pHG->tempDispersion() <= 20) &&
            (p > 1)) {

            pBz->shortBeep();

            ready = true;
        }
        #endif
    }

    // =========================
    // Ready state
    // =========================
    if (ready) {

        pD->drawState(FS(TXT_HOLD), ST7735_CYAN);

        pD->tReal(
            pEnc->read(), 20, 83, TFT_LABEL_SIZE
        );

        uint8_t p = pHG->appliedPower();

        pD->appliedPower(
            p,
            92,
            86,
            TFT_LABEL_SIZE
        );

        pD->fan_speed(
            pHG->getfan_speed(),
            92,
            45,
            TFT_LABEL_SIZE
        );
    }

    // Hotgun switched OFF unexpectedly
    if (tune && !pHG->isOn()) {

        tune  = false;
        ready = false;

        pD->drawState(FS(TXT_OFF), ST7735_RED);

        pD->drawCalibrationLayout(ready);

        pD->invalidateCache();
    }

    return this;
}

void calibSCREEN::rotaryValue(int16_t value) {
    update_screen = millis() + period;
    if (!tune) {                                                            // select the temperature to be calibrated, t_min, t_mid or t_max
        mode = value;
        if (mode > 2) mode = 2;
        uint16_t temp = temp_tip[mode];
        temp = pCfg->tempInternal(temp);
        pHG->setTemp(temp);
    }
    forceRedraw();
}

SCREEN* calibSCREEN::menu(void) { 

    // =========================
    // Start wizard
    // =========================
    if (!started) {

        started = true;

        tune  = true;
        ready = false;

        uint16_t temp = temp_tip[mode];

        pEnc->reset(temp, 40, 600, 1, 5);

        temp = pCfg->tempInternal(temp);

        pHG->setTemp(temp);

        pHG->switchPower(true);

        // FORCE FULL REDRAW
        pD->drawCalibrationLayout(false);
        pD->invalidateCache();

        update_screen = 0;

        forceRedraw();

        return this;
    }

    // =========================
    // SAVE CURRENT POINT
    // =========================
    if (ready) {

        tune = false;

        calib_temp[0][mode] = pEnc->read();

        calib_temp[1][mode] = pHG->tempAverage();

        pHG->switchPower(false);

        // ===== SAVED SCREEN =====
        pD->drawCalibrationSaved(mode);

        delay(1200);

        uint16_t tip[3];

        buildCalibration(tip);

        pCfg->applyCalibrationData(tip);

        // ===== NEXT POINT =====
        mode++;

        // Finished all points
        if (mode > 2) {

            pD->drawCalibrationComplete();

            delay(1500);

            return menu_long();
        }

        // =========================
        // Start next heating cycle
        // =========================
        ready = false;

        tune  = true;

        uint16_t temp = temp_tip[mode];

        pEnc->reset(temp, 40, 600, 1, 5);

        temp = pCfg->tempInternal(temp);

        pHG->setTemp(temp);

        pHG->switchPower(true);

        // FORCE FULL REDRAW
        pD->drawCalibrationLayout(false);
        pD->invalidateCache();

        invalidateStateCache();

        update_screen = millis();

        forceRedraw();

        return this;
    }

    return this;
}

SCREEN* calibSCREEN::menu_long(void) {
    pHG->switchPower(false);
    // temp_tip - array of calibration temperatures in Celsius
    uint16_t tip[3];
    buildCalibration(tip);
    pCfg->saveCalibrationData(tip);
    pCfg->applyCalibrationData(tip);
    pHG->init(); 
    
    // Guardamos usando la instrucción nativa idéntica a la del menú EXIT
    pCfg->save(pCfg->tempPreset(), pHG->getfan_speed());
    
    // Sincronizamos de forma forzada la perilla física del encoder con tus grados reales (208)
    pEnc->write(preset_temp);

    if (next) return next;
    return this;
}

void calibSCREEN::invalidateStateCache(void) {

    last_tune  = !tune;

    last_ready = !ready;

    last_mode  = 255;
}

/*
 * Calculate hot gun calibration parameters using linear approximation by Ordinary Least Squares method
 * Y = a * X + b, where
 * Y - internal temperature, X - real temperature. a and b are double coefficients
 * a = (N * sum(Xi*Yi) - sum(Xi) * sum(Yi)) / ( N * sum(Xi^2) - (sum(Xi))^2)
 * b = 1/N * (sum(Yi) - a * sum(Xi))
 * N = 3 (3 reference temperature points are used)
 */
void calibSCREEN::buildCalibration(uint16_t tip[3]) {
    long sum_XY = 0;                                                        // sum(Xi * Yi)
    long sum_X  = 0;                                                        // sum(Xi)
    long sum_Y  = 0;                                                        // sum(Yi)
    long sum_X2 = 0;                                                        // sum(Xi^2)

    for (uint8_t i = 0; i < 3; ++i) {
        uint16_t X  = calib_temp[0][i];
        uint16_t Y  = calib_temp[1][i];
        sum_XY  += X * Y;
        sum_X   += X;
        sum_Y   += Y;
        sum_X2  += X * X;
    }

    double a = (double)(3 * sum_XY - sum_X * sum_Y) / (double)(3 * sum_X2 - sum_X * sum_X);
    double b = ((double)sum_Y - a * (double)sum_X) / 3.0;

    for (uint8_t i = 0; i < 3; ++i) {
        double temp = a * (double)temp_tip[i] + b;
        tip[i] = round(temp);
    }
    if (tip[2] > temp_max) tip[2] = temp_max;                               // Maximal possible temperature
}

//---------------------------------------- class tuneSCREEN [tune the potentiometer ] --------------------------
class tuneSCREEN : public SCREEN {
    public:
        tuneSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pBz     = Buzz;
        }
        virtual void    init(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
    private:
        HOTGUN*     pHG;                                                    // Pointer to the IRON instance
        DSPL*       pD;                                                     // Pointer to the display instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        BUZZER*     pBz;                                                    // Pointer to the simple Buzzer instance
        bool        on;                                                     // Wether the power is on
        uint32_t    heat_ms;                                                // Time in ms when power was on
        uint8_t     max_power;                                              // Maximum possible power to be applied
        const uint16_t period = 500;                                        // The period in ms to update the screen
};

void tuneSCREEN::init(void) {
    pD->invalidateCache();
    pHG->switchPower(false);
    max_power = pHG->getMaxFixedPower();
    pEnc->reset(max_power >> 2, 0, max_power, 1, 2);                        // Rotate the encoder to change the power supplied
    on = false;
    heat_ms = 0;
    pD->clear();
    pD->msgTune();
    pD->drawState(FS(TXT_OFF), ST7735_RED);
    pD->fan_speed(255, 92, 45, TFT_LABEL_SIZE);
    forceRedraw();
}

void tuneSCREEN::rotaryValue(int16_t value) {
    if (on) {
        heat_ms = millis();
        pHG->fixPower(value);
    }
    forceRedraw();
}

SCREEN* tuneSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    uint16_t temp   = pHG->getCurrTemp();
    uint8_t  htr_power  = pHG->appliedPower();
    uint8_t fan_speed = pHG->getfan_speed();
    if (!on) {
        htr_power = pEnc->read();
    }
    pD->tInternal(temp, 20, 46, TFT_LABEL_SIZE);
    pD->appliedPower(htr_power, 92, 86, TFT_LABEL_SIZE);
    pD->fan_speed(fan_speed, 92, 45, TFT_LABEL_SIZE);
    if (heat_ms && ((millis() - heat_ms) > 3000) && (pHG->tempDispersion() < 10) && (htr_power > 1)) {
        pBz->shortBeep();
        heat_ms = 0;
    }
    return this;
}
  
SCREEN* tuneSCREEN::menu(void) {                                            // The rotary button pressed
    if (on) {
        pHG->fixPower(0);
        on = false;
        pD->drawState(FS(TXT_OFF), ST7735_RED);
    } else {
        on = true;
        heat_ms = millis();
        uint8_t power = pEnc->read();                                       // applied power
        pHG->fixPower(power);
        pD->drawState(FS(TXT_ON), ST7735_GREEN);
    }
    return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
    pHG->fixPower(0);                                                       // switch off the power
    pHG->switchPower(false);
    if (next) return next;
    return this;
}

//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
	public:
		pidSCREEN(HOTGUN* HG, ENCODER* ENC) {
			pHG 	= HG;
			pEnc  	= ENC;
		}
		virtual void    init(void);
		virtual SCREEN* menu(void);
		virtual SCREEN* menu_long(void);
		virtual SCREEN* show(void);
		virtual void    rotaryValue(int16_t value);
	private:
		void     	showCfgInfo(void);                 						// show the main config information: Temp set, fan speed and PID coefficients
		HOTGUN*		pHG;                             						// Pointer to the IRON instance
		ENCODER* 	pEnc;                              						// Pointer to the rotary encoder instance
		uint8_t     mode;                              						// Which parameter to tune [0-5]: select element, Kp, Ki, Kd, temp, speed
		uint32_t 	update_screen;                     						// Time in ms when to print thee info
		int      	temp_set;
		const uint16_t period = 1100;
};

void pidSCREEN::init(void) {
	temp_set = pHG->getTemp();
	mode = 0;                                     							// select the element from the list
	pEnc->reset(1, 1, 5, 1, 1, true);             							// 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp, 5 - fan
	showCfgInfo();
	Serial.println("");
}

void pidSCREEN::rotaryValue(int16_t value) {
	if (mode == 0) {                              							// select element from the menu
		showCfgInfo();
		switch (value) {
			case 1:
				Serial.println("Kp");
				break;
			case 2:
				Serial.println("Ki");
				break;
			case 4:
				Serial.println(F("Temp"));
				break;
			case 5:
				Serial.println(FS(TXT_FAN));
                break;
			case 3:
			default:
				Serial.println("Kd");
			break;
		}
	} else {
		switch (mode) {
			case 1:
				Serial.print(F("Kp = "));
				pHG->changePID(mode, value);
				break;
			case 2:
				Serial.print(F("Ki = "));
				pHG->changePID(mode, value);
				break;
			case 4:
				Serial.print(F("Temp = "));
				temp_set = value;
				pHG->setTemp(value);
				break;
			case 5:
				Serial.print(F("Fan Speed = "));
				pHG->setfan_speed(value);
				break;
			case 3:
			default:
				Serial.print(F("Kd = "));
				pHG->changePID(mode, value);
				break;
		}
		Serial.println(value);
	}
}

SCREEN* pidSCREEN::show(void) {
	if (millis() < update_screen) return this;
	update_screen = millis() + period;
	if (pHG->isOn()) {
		int		 temp   = pHG->getCurrTemp();
		uint8_t	 htr_power 	= pHG->powerAverage();
		uint8_t  fan_speed		= pHG->getfan_speed();
		fan_speed = map(fan_speed, 0, 255, 0, 100);
        Serial.print(temp_set - temp);

        Serial.print(F(": power = "));
        Serial.print(htr_power);
        Serial.print('%');
        Serial.print(F(", fan = "));
        Serial.print(fan_speed);
        Serial.println(';');
	}
	return this;
}
SCREEN* pidSCREEN::menu(void) {                 							// The encoder button pressed
	if (mode == 0) {                              							// select upper or lower temperature limit
		mode = pEnc->read();
		if (mode > 0 && mode < 4) {
			int k = pHG->changePID(mode, -1);
			pEnc->reset(k, 0, 10000, 1, 10);
		} else if (mode == 4) {
			pEnc->reset(temp_set, 0, 970, 1, 5);
		} else {
			pEnc->reset(pHG->getfan_speed(), 0, 250, 5, 20);
		}
	} else {    
		mode = 0;
		pEnc->reset(1, 1, 5, 1, 1, true);           						// 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp, 5 - fan speed
	}
	return this;
}

SCREEN* pidSCREEN::menu_long(void) {
	bool on = pHG->isOn();
	pHG->switchPower(!on);
	if (on)
		Serial.println(F("The air gun is OFF"));
	else
		Serial.println(F("The air gun is ON"));
  return this;
}

void pidSCREEN::showCfgInfo(void) {
	Serial.print(F("Temp set: "));
	Serial.print(temp_set, DEC);
	Serial.print(F(", fan speed = "));
	Serial.print(pHG->getfan_speed());
	Serial.print(F(", PID: ["));
	for (byte i = 1; i < 4; ++i) {
		int k = pHG->changePID(i, -1);
		Serial.print(k, DEC);
		if (i < 3) Serial.print(F(", "));
	}
	Serial.print(F("]; "));
}

//=========================================================================================================
HOTGUN 		hg(HOT_GUN_PIN);
DSPL       	disp;
ENCODER    	rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON     	rotButton(R_BUTN_PIN);
SWITCH      reedSwitch(REED_SW_PIN);
HOTGUN_CFG 	hgCfg;
BUZZER     	simpleBuzzer(BUZZER_PIN);

mainSCREEN   offScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
workSCREEN   wrkScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
configSCREEN cfgScr(&hg,  &disp, &rotEncoder, &hgCfg);
calibSCREEN  clbScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
tuneSCREEN   tuneScr(&hg, &disp, &rotEncoder, &simpleBuzzer);
errorSCREEN  errScr(&hg,  &disp, &simpleBuzzer);
pidSCREEN    pidScr(&hg,  &rotEncoder);

SCREEN 	*pCurrentScreen = &offScr;

volatile bool	end_of_power_period = false;

void syncAC(void) {
    end_of_power_period = hg.syncCB();
}

void rotEncChange(void) {
	rotEncoder.changeINTR();
}

void setup() {
	Serial.begin(115200);
	disp.init();

	// Load configuration parameters
	hgCfg.init();
	hg.init();
	uint16_t temp 	= hgCfg.tempPreset();
	uint8_t  fan	= hgCfg.fanPreset();
	hg.setTemp(temp);
	hg.setfan_speed(fan);

    reedSwitch.init(500, 3000);

	// Initialize rotary encoder
	rotEncoder.init();
	rotButton.init();
	delay(500);
	attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(AC_SYNC_PIN), syncAC, RISING);

	// Initialize SCREEN hierarchy
	offScr.next     = &cfgScr;
    offScr.on       = &wrkScr;
    wrkScr.next     = &offScr;
    cfgScr.next     = &offScr;
    cfgScr.calib    = &clbScr;
    cfgScr.tune     = &tuneScr;
    clbScr.next     = &offScr;
    tuneScr.next    = &offScr;
	errScr.next     = &offScr;

    pCurrentScreen->init();
  
    // Pequeña pausa para que el primer ciclo del loop no colisione con la inicialización
    delay(50); 
    //ledcSetup(0, 25000, 8);
}

void loop() {
  static bool     reset_encoder       = true;
  static int16_t  old_pos             = 0;
  static uint32_t ac_check            = 5000;
  static uint32_t last_update         = 0;
  static uint32_t last_screen_change  = 0;
  static bool     boot_guard          = true;
  static bool     processing_transition = false;

  if (processing_transition) return;

  int16_t pos = rotEncoder.read();
  if (reset_encoder) {
    old_pos = pos;
    reset_encoder = false;
  } else if (old_pos != pos) {
    pCurrentScreen->rotaryValue(pos);
    old_pos = pos;
  }

  SCREEN* nxt = nullptr;
  // 1. Reed Switch
  SCREEN* rNxt = pCurrentScreen->reedSwitch(reedSwitch.status());
  if (rNxt != pCurrentScreen) nxt = rNxt;

  // 2. Botones
  if (!nxt) {
    uint8_t bStatus = rotButton.buttonCheck();
    if (bStatus == 2) {
      processing_transition = true;
      SCREEN* bNxt = pCurrentScreen->menu_long();
      if (bNxt != pCurrentScreen) {
        nxt = bNxt;
        // Espera a que se suelte el botón para evitar rebotes
        while(rotButton.buttonCheck() != 0);
        delay(100);
      }
      processing_transition = false;
    } else if (bStatus == 1) {
      SCREEN* bNxt = pCurrentScreen->menu();
      if (bNxt != pCurrentScreen) nxt = bNxt;
    }
  }

  // 3. Show & AC Check
   if (!nxt) {
    SCREEN* sNxt = pCurrentScreen->show();
    if (sNxt != pCurrentScreen) nxt = sNxt;
    uint32_t current_time = millis();
    if (current_time - last_update >= 1000) last_update = current_time;
    
    /*Se comenta este bloque para pruebas de los menús en el simulador*/
    if (current_time > ac_check) {
      ac_check = current_time + 1000;
      if (!hg.areExternalInterrupts()) nxt = &errScr;
      
    }
    /**/
  }

  // === TRANSICIÓN CORREGIDA ===
  if (nxt && nxt != pCurrentScreen) {
    if (!boot_guard || (millis() > 1000 && millis() - last_screen_change > 500)) {
      processing_transition = true;
      
        // Asegurarse de que no haya conflictos con pantallas anteriores
        if (nxt != &errScr && nxt != &tuneScr && nxt != &clbScr) {
        
        // --- CAMBIO AQUÍ: Forzar limpieza total al regresar a las pantallas principales ---
        if (nxt == &offScr || nxt == &wrkScr) {
          disp.clear();
        } 
        // Si no va a las principales, mantiene tu filtro de protección original para submenús
        else if (pCurrentScreen != &cfgScr) {
          disp.clear();
        }
        // ---------------------------------------------------------------------------------
        
      }
      
      nxt->init();
      pCurrentScreen = nxt;
      last_screen_change = millis();
      boot_guard = false;
      reset_encoder = true;
      
      delay(50);
      processing_transition = false;
    }
  }

  if (end_of_power_period) {
    hg.keepTemp();
    end_of_power_period = false;
  }
}

float obtenerTemperaturaSegura() {
  static unsigned long cronometroMax = 0;
  static float ultimaTemperaturaValida = 25.0; // Valor inicial por defecto

  // El chip solo actualiza cada 220ms. Leerlo más rápido es perder el tiempo.
  if (millis() - cronometroMax >= 300) { 
    cronometroMax = millis();
    float lecturaActual = sensorTermocupla.readCelsius();
    
    // El MAX6675 a veces devuelve "NAN" (Not a Number) si hay ruido. 
    // Esto evita que el PID se vuelva loco si falla una lectura.
    if (!isnan(lecturaActual) && lecturaActual > 0.0) {
      ultimaTemperaturaValida = lecturaActual;
    }
  }
  return ultimaTemperaturaValida;
}