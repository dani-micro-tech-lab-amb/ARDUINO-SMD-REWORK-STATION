#include <Arduino.h>

// Asignación de tus pines Arduino ordenados de Mayor a Menor Canal
const int CANAL_8_SCL = 13; // Cable Blanco-Azul  -> Pin D13 Arduino (B8 -> A8)
const int CANAL_7_SDA = 11; // Cable Azul         -> Pin D11 Arduino (B7 -> A7)
const int CANAL_6_RES = A1; // Cable Blanco-Naran -> Pin A1 Arduino (B6 -> A6)
const int CANAL_5_DC  = 10; // Cable Naranja      -> Pin D10 Arduino (B5 -> A5)
const int CANAL_4_CS  = 6;  // Cable Blanco-Verde -> Pin D6 Arduino (B4 -> A4) [BAJO SOSPECHA]

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("--- INICIANDO TEST DE 5 SEGUNDOS (CANAL 8 AL 4) ---");
  Serial.println("Tómate tu tiempo. Cada canal mantendrá 3.3V durante 5 segundos en la salida del logig shifter del ado de la pantalla.");

  // Configurar todos los pines como salidas digitales
  pinMode(CANAL_8_SCL, OUTPUT);
  pinMode(CANAL_7_SDA, OUTPUT);
  pinMode(CANAL_6_RES, OUTPUT);
  pinMode(CANAL_5_DC,  OUTPUT);
  pinMode(CANAL_4_CS,  OUTPUT);

  // Forzar estado inicial bajo en todas las líneas
  digitalWrite(CANAL_8_SCL, LOW);
  digitalWrite(CANAL_7_SDA, LOW);
  digitalWrite(CANAL_6_RES, LOW);
  digitalWrite(CANAL_5_DC,  LOW);
  digitalWrite(CANAL_4_CS,  LOW);
}

void probarCanal(const char* canalStr, int pinArduino, const char* funcion, const char* colorCable) {
  Serial.print("[PROBANDO] ");
  Serial.print(canalStr);
  Serial.print(" | Función: ");
  Serial.print(funcion);
  Serial.print(" | Cable: ");
  Serial.println(colorCable);
  Serial.println(">> Generando 3 pulsos lentos...");

  // Ciclo de parpadeo extendido para medición ultra cómoda
  for (int i = 0; i < 3; i++) {
    digitalWrite(pinArduino, HIGH);
    delay(5000); // 5 segundos encendido -> ¡Mide con total calma aquí!
    digitalWrite(pinArduino, LOW);
    delay(3000); // 1 segundo apagado -> Verificas que caiga a 0V
  }
  Serial.println(">> Fin de prueba de este canal.\n");
  delay(1500); // Pausa cómoda antes de saltar al siguiente pin físico
}

void loop() {
  // Secuencia inversa descendente (A8 -> A4) con tiempos extendidos
  probarCanal("CANAL 8 (B8 -> A8)", CANAL_8_SCL, "SCL (SCK Reloj)",   "Blanco-Azul");
  probarCanal("CANAL 7 (B7 -> A7)", CANAL_7_SDA, "SDA (MOSI Datos)",  "Azul");
  probarCanal("CANAL 6 (B6 -> A6)", CANAL_6_RES, "RES (Reset)",       "Blanco-Naranja");
  probarCanal("CANAL 5 (B5 -> A5)", CANAL_5_DC,  "DC (Data/Command)", "Naranja");
  probarCanal("CANAL 4 (B4 -> A4)", CANAL_4_CS,  "CS (Chip Select)", "Blanco-Verde");

  Serial.println("=================================================================");
  Serial.println("CICLO DE CHIP COMPLETADO. Reiniciando secuencia en 5 segundos...");
  Serial.println("=================================================================\n");
  delay(5000);
}