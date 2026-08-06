/*
  Prueba Unitaria Integrada - MAX6675 + Pantalla TFT 1.8" (ST7735)
  Pines mapeados al hardware real de la estación de calor.
*/

#include <Adafruit_GFX.h>    
#include <Adafruit_ST7735.h> 
#include <SPI.h>

// --- ASIGNACIÓN DE PINES (Mapeado a tu firmware principal) ---
const uint8_t HOT_GUN_PIN   = 7;   // Calefacción (Desactivada por seguridad)
const uint8_t FAN_GUN_PIN   = 9;   // Ventilador (Desactivado)
const uint8_t BUZZER_PIN    = 8;   // Buzzer

#define TFT_CS_PIN  6      // CS TFT
#define TFT_DC_PIN  10     // DC TFT
#define TFT_RST_PIN A1     // RST TFT
// Nota: SPI Hardware usa D11 (MOSI) y D13 (SCK)

#define ST7735_NAVY 0x000F

const int PIN_MAX6675_DO  = A2;    // SO / MISO
const int PIN_MAX6675_CS  = A3;    // CS
const int PIN_MAX6675_CLK = A4;    // SCK / CLK

// Inicialización de la pantalla TFT con tus pines reales
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

// Control de refresco
float tempPrevia = -9999.0;
unsigned long previoMillis = 0;
const long intervalo = 400; // Refresco cada 400ms

// Lectura por software del MAX6675 en pines A2, A3, A4
float leerMAX6675() {
  uint16_t rawData = 0;

  digitalWrite(PIN_MAX6675_CS, LOW);
  delayMicroseconds(1);

  for (int i = 15; i >= 0; i--) {
    digitalWrite(PIN_MAX6675_CLK, HIGH);
    delayMicroseconds(1);

    if (digitalRead(PIN_MAX6675_DO)) {
      rawData |= (1 << i);
    }

    digitalWrite(PIN_MAX6675_CLK, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(PIN_MAX6675_CS, HIGH);

  // D2 indica si el termopar está abierto/desconectado
  if (rawData & 0x04) {
    return -999.0;
  }

  rawData >>= 3;
  return rawData * 0.25;
}

void setup() {
  Serial.begin(115200);

  // --- SEGURIDAD: Apagar salidas activas/sensibles inmediatamente ---
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(HOT_GUN_PIN, OUTPUT);
  digitalWrite(HOT_GUN_PIN, LOW);

  pinMode(FAN_GUN_PIN, OUTPUT);
  digitalWrite(FAN_GUN_PIN, LOW);

  // Configuración de pines MAX6675
  pinMode(PIN_MAX6675_CS, OUTPUT);
  pinMode(PIN_MAX6675_CLK, OUTPUT);
  pinMode(PIN_MAX6675_DO, INPUT);
  digitalWrite(PIN_MAX6675_CS, HIGH);
  digitalWrite(PIN_MAX6675_CLK, LOW);

  // Inicializar Pantalla TFT
  tft.initR(INITR_BLACKTAB); 
  tft.setRotation(1); // Orientación horizontal
  tft.fillScreen(ST7735_BLACK);

  // Dibujar interfaz estática
  tft.drawRect(0, 0, 160, 128, ST7735_BLUE);
  tft.fillRect(2, 2, 156, 22, ST7735_NAVY);

  tft.setCursor(12, 8);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.print("DIAGNOSTICO MAX6675");

  tft.setCursor(15, 38);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.print("TEMP. PISTOLA:");

  Serial.println(F("--- TEST UNITARIO CORREGIDO: MAX6675 + TFT ---"));
}

void loop() {
  unsigned long actualMillis = millis();

  if (actualMillis - previoMillis >= intervalo) {
    previoMillis = actualMillis;

    float tempC = leerMAX6675();

    // 1. Salida a Monitor Serie
    Serial.print(F("[TEST] Temperatura: "));
    if (tempC == -999.0) {
      Serial.println(F("ERROR - Sonda abierta o no conectada"));
    } else {
      Serial.print(tempC, 2);
      Serial.println(F(" °C"));
    }

    // 2. Actualización de Pantalla TFT
    if (tempC != tempPrevia) {
      tempPrevia = tempC;

      if (tempC == -999.0) {
        tft.fillRect(10, 58, 140, 45, ST7735_BLACK);
        tft.setCursor(18, 70);
        tft.setTextColor(ST7735_RED, ST7735_BLACK);
        tft.setTextSize(2);
        tft.print("! ERROR !");
      } else {
        tft.fillRect(10, 58, 140, 45, ST7735_BLACK);

        tft.setCursor(20, 65);
        tft.setTextSize(3);
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);

        if (tempC < 100) tft.print(" ");
        if (tempC < 10)  tft.print(" ");

        tft.print((int)tempC);

        tft.setTextSize(2);
        tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
        tft.print(" C");
      }
    }
  }
}