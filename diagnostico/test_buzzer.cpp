/*
 * ==========================================================
 * HOT AIR GUN STATION
 * Hardware Diagnostic
 *
 * File      : test_buzzer.cpp
 * Version   : 1.0
 * Author    : Dani Micro Tech Lab
 *
 * Description
 * -----------
 * Standalone buzzer diagnostic.
 *
 * Encoder button toggles buzzer state.
 * Arduino onboard LED mirrors buzzer output.
 *
 * ==========================================================
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//
// Pin mapping
//

const uint8_t R_BUTN_PIN = 5;

const uint8_t BUZZER_PIN = 8;
const uint8_t LED_PIN    = LED_BUILTIN;

#define TFT_CS_PIN   6
#define TFT_DC_PIN   10
#define TFT_RST_PIN  A1

Adafruit_ST7735 tft(
    TFT_CS_PIN,
    TFT_DC_PIN,
    TFT_RST_PIN
);

//
// Configuration
//

const uint16_t DEBOUNCE_TIME = 40;

//
// Variables
//

bool buzzerState = false;

bool lastButton = HIGH;
bool stableButton = HIGH;

unsigned long debounceTimer = 0;

//
// Statistics
//

uint32_t toggleCounter = 0;

unsigned long lastTransition = 0;

unsigned long accumulatedON = 0;
unsigned long accumulatedOFF = 0;

//
// ----------------------------------------------------------
// Draw Header
// ----------------------------------------------------------
//

void drawHeader()
{
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);

    tft.setCursor(18,5);
    tft.print("BUZZER");

    tft.setCursor(34,24);
    tft.print("TEST");

    tft.drawFastHLine(
        0,
        48,
        160,
        ST77XX_WHITE
    );

    tft.setTextSize(1);

    tft.setCursor(4,56);
    tft.print("STATE");

    tft.setCursor(4,72);
    tft.print("COUNT");

    tft.setCursor(4,88);
    tft.print("ON");

    tft.setCursor(82,88);
    tft.print("OFF");

    tft.setCursor(4,104);
    tft.print("LIVE");

    tft.setCursor(4,120);
    tft.print("BUTTON");
}

//
// ----------------------------------------------------------
// Draw Buzzer State
// ----------------------------------------------------------
//

void drawStatus()
{
    tft.fillRect(
        55,
        54,
        100,
        14,
        ST77XX_BLACK
    );

    if(buzzerState)
    {
        tft.fillCircle(
            62,
            61,
            4,
            ST77XX_GREEN
        );

        tft.setTextColor(ST77XX_GREEN);

        tft.setCursor(72,56);
        tft.print("ON");
    }
    else
    {
        tft.fillCircle(
            62,
            61,
            4,
            ST77XX_RED
        );

        tft.setTextColor(ST77XX_RED);

        tft.setCursor(72,56);
        tft.print("OFF");
    }

    tft.setTextColor(ST77XX_WHITE);
}

//
// ----------------------------------------------------------
// Draw Statistics
// ----------------------------------------------------------
//

void drawCounters()
{
    tft.fillRect(
        55,
        70,
        45,
        10,
        ST77XX_BLACK
    );

    tft.setCursor(55,72);

    tft.setTextColor(ST77XX_CYAN);

    tft.print(toggleCounter);

    tft.setTextColor(ST77XX_WHITE);
}

//
// ----------------------------------------------------------
// Draw Button State
// ----------------------------------------------------------
//

void drawButtonState()
{
    tft.fillRect(
        55,
        118,
        60,
        10,
        ST77XX_BLACK
    );

    if(stableButton == LOW)
    {
        tft.setTextColor(ST77XX_GREEN);
        tft.setCursor(55,120);
        tft.print("PRESSED");
    }
    else
    {
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(55,120);
        tft.print("READY");
    }

    tft.setTextColor(ST77XX_WHITE);
}

//
// ----------------------------------------------------------
// Refresh all dynamic information
// ----------------------------------------------------------
//

void refreshScreen()
{
    drawStatus();
    drawCounters();
    drawTimers();
    drawButtonState();
}

//
// ----------------------------------------------------------
// Time formatter
// Format:
// MM:SS.mmm
// ----------------------------------------------------------
//

void printTime(
    uint8_t x,
    uint8_t y,
    unsigned long milliseconds)
{
    unsigned long totalSeconds = milliseconds / 1000;

    uint16_t minutes = totalSeconds / 60;
    uint8_t seconds  = totalSeconds % 60;

    uint16_t ms = milliseconds % 1000;

    tft.setCursor(x, y);

    if(minutes < 10) tft.print('0');
    tft.print(minutes);
    tft.print(':');

    if(seconds < 10) tft.print('0');
    tft.print(seconds);
    tft.print('.');

    if(ms < 100) tft.print('0');
    if(ms < 10)  tft.print('0');

    tft.print(ms);
}

//
// ----------------------------------------------------------
// Draw accumulated timers
// ----------------------------------------------------------
//

void drawTimers()
{
    //
    // ON accumulated
    //

    tft.fillRect(
        18,
        96,
        58,
        10,
        ST77XX_BLACK
    );

    tft.setTextColor(ST77XX_GREEN);

    printTime(
        18,
        96,
        accumulatedON
    );

    //
    // OFF accumulated
    //

    tft.fillRect(
        96,
        96,
        58,
        10,
        ST77XX_BLACK
    );

    tft.setTextColor(ST77XX_RED);

    printTime(
        96,
        96,
        accumulatedOFF
    );

    tft.setTextColor(ST77XX_WHITE);
}

//
// ----------------------------------------------------------
// Draw live timer
// ----------------------------------------------------------
//

void drawLiveTimer()
{
    unsigned long elapsed =
        millis() - lastTransition;

    tft.fillRect(
        36,
        104,
        118,
        10,
        ST77XX_BLACK
    );

    if(buzzerState)
        tft.setTextColor(ST77XX_GREEN);
    else
        tft.setTextColor(ST77XX_RED);

    printTime(
        36,
        104,
        elapsed
    );

    tft.setTextColor(ST77XX_WHITE);
}

//
// ----------------------------------------------------------
// Register state transition
// ----------------------------------------------------------
//

void registerTransition()
{
    unsigned long elapsed =
        millis() - lastTransition;

    if(buzzerState)
        accumulatedON += elapsed;
    else
        accumulatedOFF += elapsed;

    lastTransition = millis();
}

//
// ----------------------------------------------------------
// Toggle buzzer
// ----------------------------------------------------------
//

void toggleBuzzer()
{
    registerTransition();

    buzzerState = !buzzerState;

    toggleCounter++;

    setOutputs(buzzerState);

    refreshScreen();
}

//
// ----------------------------------------------------------
// Main Loop
// ----------------------------------------------------------
//

void loop()
{
    bool reading = digitalRead(R_BUTN_PIN);

    //
    // Detect button transition
    //

    if(reading != lastReading)
    {
        debounceTimer = millis();
        lastReading = reading;
    }

    //
    // Debounce
    //

    if((millis() - debounceTimer) > DEBOUNCE_TIME)
    {
        if(reading != stableButton)
        {
            stableButton = reading;

            //
            // Toggle only on button press
            //

            if(stableButton == LOW)
            {
                toggleBuzzer();
            }
        }
    }

    //
    // Refresh live timer every 100 ms
    //

    static unsigned long refreshTimer = 0;

    if((millis() - refreshTimer) >= 100)
    {
        refreshTimer = millis();

        drawLiveTimer();
    }
}