/*
 * ================================================================
 * HOT AIR GUN STATION
 * Hardware Unit Test
 * ================================================================
 *
 * File      : test_reed.cpp
 * Version   : 2.1 (Fixed)
 *
 * Component Under Test
 * --------------------
 * Magnetic Reed Switch
 * ================================================================
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//
// Pin Mapping
//
const uint8_t REED_PIN = 12;
const uint8_t LED_PIN = LED_BUILTIN;

#define TFT_CS_PIN   6
#define TFT_DC_PIN   10
#define TFT_RST_PIN  A1

//
// TFT Initialization
//
Adafruit_ST7735 display(
    TFT_CS_PIN,
    TFT_DC_PIN,
    TFT_RST_PIN
);

//
// Configuration
//
const uint16_t DEBOUNCE_TIME = 25;
const uint16_t BOUNCE_TIME = 10;
const uint16_t DISPLAY_REFRESH = 100;

//
// Reed State Enum
//
enum ReedState
{
    REED_OPEN,
    REED_DETECTED
};

//
// Runtime variables
//
ReedState currentState = REED_OPEN;
ReedState lastStableState = REED_OPEN;

bool lastReading = HIGH;
unsigned long debounceTimer = 0;
unsigned long refreshTimer = 0;
unsigned long lastTransition = 0;

//
// Statistics
//
uint32_t activationCounter = 0;
uint32_t bounceCounter = 0;
unsigned long totalOnTime = 0;
unsigned long totalOffTime = 0;

//
// Result Enum
//
enum TestResult
{
    TEST_WAITING,
    TEST_PASS,
    TEST_WARNING,
    TEST_FAIL
};

TestResult result = TEST_WAITING;

//
// Function Prototypes
//
void drawLayout();
void drawState();
void drawCounters();
void drawTimers();
void drawLiveTimer();
void drawResult();
void updateLED();
void processReed();
void registerTransition();
void printTime(uint8_t x, uint8_t y, unsigned long value);

//
// Draw Static Layout
//
void drawLayout()
{
    display.fillScreen(ST77XX_BLACK);
    display.setTextWrap(false);

    display.setTextColor(ST77XX_WHITE);
    display.setTextSize(2);
    display.setCursor(26,4);
    display.print("REED");
    display.setCursor(34,22);
    display.print("TEST");

    display.setTextSize(1);
    display.setCursor(125,6);
    display.print("v2.1");

    display.drawFastHLine(0, 40, 160, ST77XX_WHITE);

    display.setCursor(4,48);
    display.print("STATE");
    display.setCursor(4,64);
    display.print("COUNT");
    display.setCursor(4,80);
    display.print("BOUNCE");
    display.setCursor(4,96);
    display.print("LIVE");
    display.setCursor(4,112);
    display.print("RESULT");
}

//
// Draw Reed State
//
void drawState()
{
    display.fillRect(55, 46, 100, 12, ST77XX_BLACK);

    if(currentState == REED_DETECTED)
    {
        display.fillCircle(60, 52, 4, ST77XX_GREEN);
        display.setTextColor(ST77XX_GREEN);
        display.setCursor(70,48);
        display.print("DETECTED");
    }
    else
    {
        display.fillCircle(60, 52, 4, ST77XX_RED);
        display.setTextColor(ST77XX_RED);
        display.setCursor(70,48);
        display.print("OPEN");
    }

    display.setTextColor(ST77XX_WHITE);
}

//
// Draw Counters
//
void drawCounters()
{
    display.fillRect(60, 62, 40, 10, ST77XX_BLACK);
    display.setCursor(60,64);
    display.setTextColor(ST77XX_CYAN);
    display.print(activationCounter);

    display.fillRect(72, 78, 40, 10, ST77XX_BLACK);
    display.setCursor(72,80);

    if(bounceCounter == 0)
        display.setTextColor(ST77XX_GREEN);
    else
        display.setTextColor(ST77XX_YELLOW);

    display.print(bounceCounter);
    display.setTextColor(ST77XX_WHITE);
}

//
// Draw Test Result
//
void drawResult()
{
    display.fillRect(60, 110, 90, 12, ST77XX_BLACK);

    switch(result)
    {
        case TEST_WAITING:
            display.setTextColor(ST77XX_YELLOW);
            display.setCursor(60,112);
            display.print("WAITING");
            break;

        case TEST_PASS:
            display.setTextColor(ST77XX_GREEN);
            display.setCursor(60,112);
            display.print("PASS");
            break;

        case TEST_WARNING:
            display.setTextColor(ST77XX_YELLOW);
            display.setCursor(60,112);
            display.print("WARNING");
            break;

        case TEST_FAIL:
            display.setTextColor(ST77XX_RED);
            display.setCursor(60,112);
            display.print("FAIL");
            break;
    }

    display.setTextColor(ST77XX_WHITE);
}

//
// Print Time Format: MM:SS.mmm
//
void printTime(uint8_t x, uint8_t y, unsigned long value)
{
    unsigned long totalSeconds = value / 1000;
    uint16_t minutes = totalSeconds / 60;
    uint8_t seconds = totalSeconds % 60;
    uint16_t milliseconds = value % 1000;

    display.setCursor(x, y);

    if(minutes < 10) display.print('0');
    display.print(minutes);
    display.print(':');

    if(seconds < 10) display.print('0');
    display.print(seconds);
    display.print('.');

    if(milliseconds < 100) display.print('0');
    if(milliseconds < 10) display.print('0');
    display.print(milliseconds);
}

//
// Draw Live Timer
//
void drawLiveTimer()
{
    display.fillRect(44, 94, 80, 10, ST77XX_BLACK);

    unsigned long elapsed = millis() - lastTransition;

    if(currentState == REED_DETECTED)
        display.setTextColor(ST77XX_GREEN);
    else
        display.setTextColor(ST77XX_RED);

    printTime(44, 96, elapsed);
    display.setTextColor(ST77XX_WHITE);
}

//
// Register Transition Time
//
void registerTransition()
{
    unsigned long elapsed = millis() - lastTransition;

    if(currentState == REED_DETECTED)
        totalOnTime += elapsed;
    else
        totalOffTime += elapsed;

    lastTransition = millis();
}

//
// Update Status LED
//
void updateLED()
{
    digitalWrite(LED_PIN, (currentState == REED_DETECTED) ? HIGH : LOW);
}

//
// Process Reed Switch Input
//
void processReed()
{
    bool reading = digitalRead(REED_PIN);

    // Detect pin change for bounce calculation
    if(reading != lastReading)
    {
        if((millis() - debounceTimer) < BOUNCE_TIME)
        {
            bounceCounter++;
            if(bounceCounter > 3)
                result = TEST_WARNING;
        }

        debounceTimer = millis();
        lastReading = reading;
    }

    // Wait for stabilization
    if((millis() - debounceTimer) < DEBOUNCE_TIME)
        return;

    // Active LOW logic (INPUT_PULLUP)
    ReedState newState = (reading == LOW) ? REED_DETECTED : REED_OPEN;

    if(newState == currentState)
        return;

    // Register timing transition before changing state
    registerTransition();

    currentState = newState;

    updateLED();
    drawState();

    if(currentState == REED_DETECTED)
    {
        activationCounter++;
        if(result == TEST_WAITING)
            result = TEST_PASS;
    }

    drawCounters();
    drawResult();
}

//
// Setup
//
void setup()
{
    pinMode(REED_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    display.initR(INITR_BLACKTAB);
    display.setRotation(1);

    bool reading = digitalRead(REED_PIN);
    lastReading = reading;
    currentState = (reading == LOW) ? REED_DETECTED : REED_OPEN;
    lastStableState = currentState;
    lastTransition = millis();

    updateLED();

    drawLayout();
    drawState();
    drawCounters();
    drawLiveTimer();
    drawResult();
}

//
// Main Loop
//
void loop()
{
    processReed();

    if((millis() - refreshTimer) >= DISPLAY_REFRESH)
    {
        refreshTimer = millis();
        drawLiveTimer();
    }
}
