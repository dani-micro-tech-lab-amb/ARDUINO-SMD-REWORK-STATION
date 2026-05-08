/*
 * Hot air gun controller based on atmega328 IC
 * Released November 5, 2018
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <CommonControls.h>
#include <EEPROM.h>
#include <SPI.h>

const uint16_t temp_minC 	= 150;
const uint16_t temp_maxC	= 500;
const uint16_t temp_ambC    = 25;
const uint16_t temp_tip[3] = {200, 300, 400};                               // Temperature reference points for calibration

const uint8_t AC_SYNC_PIN   = 2;                                            // Outlet 220 v synchronization pin. Do not change!
const uint8_t HOT_GUN_PIN   = 7;                                            // Hot gun heater management pin
const uint8_t FAN_GUN_PIN   = 9;                                            // Hot gun fan management pin. Do not change! 
const uint8_t TEMP_GUN_PIN	= A0;                                           // Hot gun temperature checking pin

const uint8_t TEMP_POT_PIN  = A4;                                           // Potentiometer for temperature setpoint
const uint8_t FAN_POT_PIN   = A5;                                           // Potentiometer for fan/air speed

const uint8_t REED_SW_PIN   = 11;                                            // Reed switch pin
const uint8_t BUZZER_PIN	= 8;                                            // Buzzer pin

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
        void     save(uint16_t temp, uint8_t fanSpeed);                     // Save preset temperature in the internal units and fan speed
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

void HOTGUN_CFG::save(uint16_t temp, uint8_t fanSpeed) {
    Config.temp        = constrain(temp, min_temp, max_temp);
    Config.fan         = fanSpeed;
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
    t_tip[2] = tip[1];
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
// Heredamos de LiquidCrystal (estándar) en lugar de la versión I2C
class DSPL : protected LiquidCrystal { 
  public:
    // El constructor ahora recibe los pines: RS, E, D4, D5, D6, D7
    // Usando los pines que calculamos antes: 12, 10, A2, A3, 13, 6
    DSPL(void) : LiquidCrystal(12, 10, A2, A3, 13, 6) { } 

    void init(void);
    
    // Cambiamos las llamadas internas a LiquidCrystal
    void clear(void) { LiquidCrystal::clear(); }
    
    void tSet(uint16_t t, bool Celsius = true); 
    void tCurr(uint16_t t);
    void tInternal(uint16_t t);
    void tReal(uint16_t t);
    void fanSpeed(uint8_t s);
    void appliedPower(uint8_t p, bool show_zero = true);
    void setupMode(uint8_t mode);
    void msgON(void);
    void msgOFF(void);
    void msgReady(void);
    void msgCold(void);
    void msgFail(void);
    void msgTune(void);

  private:
    bool full_second_line;
    char temp_units;
    const uint8_t custom_symbols[3][8] = {
        { 0b00110, 0b01001, 0b01001, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000 }, // Grado
        { 0b00100, 0b01100, 0b01100, 0b00110, 0b01011, 0b11001, 0b10000, 0b00000 }, // Fan
        { 0b00011, 0b00110, 0b01100, 0b11111, 0b00110, 0b01000, 0b10000, 0b00000 }  // Power
    };
};

void DSPL::init(void) {
    // 1. En la versión paralela se usa .begin(columnas, filas)
    // Nota: 'lcd' debe ser el objeto que definiste globalmente
    LiquidCrystal::begin(16, 2); 
    
    // 2. Limpiar pantalla
    LiquidCrystal::clear(); 
    
    // 3. ¡IMPORTANTE! LiquidCrystal estándar NO tiene función .backlight()
    // Si quieres controlarla por código, debes usar un pin digital (ej. D10)
    // Si el pin 15 está directo a 5V, simplemente borra esta línea:
    // digitalWrite(PIN_BACKLIGHT, HIGH); 

    // 4. Crear caracteres personalizados
    // Se usa el objeto 'lcd', no el nombre de la clase
    for (uint8_t i = 0; i < 3; ++i) {
        LiquidCrystal::createChar(i + 1, (uint8_t *)custom_symbols[i]);
    }

    full_second_line = false;
    temp_units = 'C';
}

void DSPL::tSet(uint16_t t, bool Celsius) {
    char buff[10];
	if (Celsius) {
		temp_units = 'C';
	} else {
		temp_units = 'F';
	}
    LiquidCrystal::setCursor(0, 0);
    sprintf(buff, "Set:%3d%c%c", t, (char)1, temp_units);
    LiquidCrystal::print(buff);
}

void DSPL::tCurr(uint16_t t) {
    char buff[6];
    LiquidCrystal::setCursor(0, 1);
    if (t < 1000) {
        sprintf(buff, "%3d%c ", t, (char)1);
    } else {
        LiquidCrystal::print(F("xxx"));
        return;
    }
    LiquidCrystal::print(buff);
    if (full_second_line) {
        LiquidCrystal::print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tInternal(uint16_t t) {
    char buff[6];
    LiquidCrystal::setCursor(0, 1);
    if (t < 1023) {
        sprintf(buff, "%4d ", t);
    } else {
        LiquidCrystal::print(F("xxxx"));
        return;
    }
    LiquidCrystal::print(buff);
    if (full_second_line) {
        LiquidCrystal::print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tReal(uint16_t t) {
    char buff[6];
    LiquidCrystal::setCursor(11, 1);
    if (t < 1000) {
        sprintf(buff, ">%3d%c", t, (char)1);
    } else {
        LiquidCrystal::print(F("xxx"));
        return;
    }
    LiquidCrystal::print(buff);
}

void DSPL::fanSpeed(uint8_t s) {
    char buff[6];
    s = map(s, 0, 255, 0, 99);
    sprintf(buff, " %c%2d%c", (char)2, s, '%');
    LiquidCrystal::setCursor(11, 1);
    LiquidCrystal::print(buff);
}

void DSPL::appliedPower(uint8_t p, bool show_zero) {
	char buff[6];
	if (p > 99) p = 99;
    LiquidCrystal::setCursor(5, 1);
    if (p == 0 && !show_zero) {
        LiquidCrystal::print(F("     "));
    } else {
	    sprintf(buff, " %c%2d%c", (char)3, p, '%');
        LiquidCrystal::print(buff);
    }
}

void DSPL::setupMode(byte mode) {
    LiquidCrystal::clear();
    LiquidCrystal::print(F("setup"));
    LiquidCrystal::setCursor(1,1);
    switch (mode) {
        case 0:                                                             // tip calibrate
            LiquidCrystal::print(F("calibrate"));
            break;
        case 1:                                                             // tune
            LiquidCrystal::print(F("tune"));
            break;
        case 2:                                                             // save
            LiquidCrystal::print(F("save"));
            break;
        case 3:                                                             // cancel
            LiquidCrystal::print(F("cancel"));
            break;
        case 4:                                                             // set defaults
            LiquidCrystal::print(F("reset config"));
            break;
        default:
            break;
    }
}

void DSPL::msgON(void) {
    LiquidCrystal::setCursor(10, 0);
    LiquidCrystal::print(F("    ON"));
}

void DSPL::msgOFF(void) {
    LiquidCrystal::setCursor(10, 0);
    LiquidCrystal::print(F("   OFF"));
}

void DSPL::msgReady(void) {
    LiquidCrystal::setCursor(10, 0);
    LiquidCrystal::print(F(" Ready"));
}

void DSPL::msgCold(void) {
    LiquidCrystal::setCursor(10, 0);
    LiquidCrystal::print(F("  Cold"));
}

void DSPL::msgFail(void) {
    LiquidCrystal::setCursor(0, 1);
    LiquidCrystal::print(F(" -== Failed ==- "));
}

void DSPL::msgTune(void) {
    LiquidCrystal::setCursor(0, 0);
    LiquidCrystal::print(F("Tune"));
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
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
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
        HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin);
        void        init(void);
		bool		isOn(void)												{ return on; }
		void		setTemp(uint16_t t)										{ temp_set = t; }
		uint16_t	getTemp(void)											{ return temp_set; }
		uint16_t	getCurrTemp(void)										{ return h_temp.last(); }
		uint16_t 	tempAverage(void)                  						{ return h_temp.average(); }
        uint8_t     powerAverage(void)                                      { return h_power.average(); }
		uint8_t     appliedPower(void)                						{ return actual_power; }
		void		setFanSpeed(uint8_t f)									{ fan_speed = f; if (on) hg_fan.duty(f); }
		uint8_t	    getFanSpeed(void)   									{ return fan_speed; }
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
		uint16_t	temp_curr;												// The current temperature of the hot air gun
		uint8_t		fan_speed;
        uint8_t     sen_pin;
		uint8_t		gun_pin;
		HISTORY  	h_power;                           						// The history queue of power applied values
		HISTORY  	h_temp;                            						// The history queue of the temperature
		volatile    uint8_t     cnt;
        volatile    uint8_t     actual_power;
        volatile    bool        active;
        bool        on, fan, fix_power;
        bool        chill;                                                  // To chill the hot gun
        uint32_t    last_period;                                            // The time in ms when the counter reset
        const       uint8_t     period 			= 100;
		const		uint8_t		min_fan_speed	= 30;
        const       uint16_t    temp_gun_cold   = 80;                       // The temperature of the cold iron 
};

HOTGUN::HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin) {
    sen_pin = HG_sen_pin;
	gun_pin	= HG_pwr_pin;
}

void HOTGUN::init(void) {
    cnt         = 0;
    power       = 0;
    active      = false;
    on          = false;
    fan         = false;
    fix_power   = false;
    chill       = false;
    last_period = 0;
    pinMode(sen_pin, INPUT);
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

	uint16_t temp = analogRead(sen_pin);             						// Check the hot air gun temperature

    h_temp.put(temp);
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

HOTGUN 		hg(TEMP_GUN_PIN, HOT_GUN_PIN);
DSPL       	disp;
HOTGUN_CFG 	hgCfg;
BUZZER     	simpleBuzzer(BUZZER_PIN);

volatile bool	end_of_power_period = false;

void syncAC(void) {
    end_of_power_period = hg.syncCB();
}

uint16_t readTempPot(void) {
    return map(analogRead(TEMP_POT_PIN), 0, 1023, temp_minC, temp_maxC);
}

uint8_t readFanPot(void) {
    return map(analogRead(FAN_POT_PIN), 0, 1023, 0, 255);
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
	hg.setFanSpeed(fan);

	pinMode(TEMP_POT_PIN, INPUT);
	pinMode(FAN_POT_PIN, INPUT);
	delay(500);
	attachInterrupt(digitalPinToInterrupt(AC_SYNC_PIN), syncAC, RISING);

}

void loop() {
    uint16_t tempHuman = readTempPot();
    uint8_t  fanSpeed  = readFanPot();
    uint16_t currentTemp = hgCfg.tempHuman(hg.tempAverage());

    hg.setTemp(hgCfg.tempInternal(tempHuman));
    hg.setFanSpeed(fanSpeed);

    disp.tSet(tempHuman);
    disp.tCurr(currentTemp);
    disp.fanSpeed(fanSpeed);
    disp.appliedPower(hg.appliedPower());

    if (hg.isOn()) {
        disp.msgON();
    } else {
        disp.msgOFF();
    }

    if (end_of_power_period) {
        hg.keepTemp();
        end_of_power_period = false;
    }
}

