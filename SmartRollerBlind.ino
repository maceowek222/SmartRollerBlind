#include <Stepper.h>

#define motorPin1  8
#define motorPin2  9
#define motorPin3  10
#define motorPin4  11
Stepper stepper(200, motorPin1, motorPin3, motorPin2, motorPin4);

const int sw_out_pin = A0;  // Pin czujnika zew

float sw_out = 0;  // Definicja zmiennej czujnika zew
float outputDf = 0;  // Definicja wyjścia

// Zmienne czujnik zew
float jasno_out = 80, srednio_out = 400, ciemno_out = 650;

float swOut[3];  // Moisture membership function
// Rule Base
float rule[3];
float rule0, rule1, rule2;

String jasnoscOut = "";

// Ilość kroków aktualnej pozycji rolety
int currentPosition = 0;

unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 5000;  // Pomiar co 5 sekund
float Roletagora = 120, RoletaPol = 60, RoletaDol = 120;  // Polozenie rolety
float a, b;

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(100);
  lastMeasurementTime = millis();  // Inicjalizacja czasu ostatniego pomiaru
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = currentMillis;  // Zaktualizuj czas ostatniego pomiaru
    sw_out = readSwOut();
    deFuzzy();
  }

  if (outputDf > 0.0) {
    int targetPosition = map(outputDf, 0, 100, 0, 9000); // Map outputDf to stepper motor steps (0-200)
    int stepsToMove = targetPosition - currentPosition;
    currentPosition = targetPosition;
    if (stepsToMove != 0) {
      stepper.step(stepsToMove);
    }
  }

  Serial.print(sw_out);
  Serial.print("\t");
  Serial.print(outputDf);
  Serial.println("\t");
  delay(2000);
}

float readSwOut() {
  float WartoscSensor = analogRead(sw_out_pin);
  float WartoscOut = WartoscSensor;
  return WartoscOut;
}

void fuzzyOut() {
  // Wartość funkcji przynależności dla jasno
  if (sw_out <= jasno_out) {
    swOut[0] = 1; // Wartość funkcji przynależności
  } else if (sw_out >= jasno_out && sw_out <= srednio_out) {
    swOut[0] = (srednio_out - sw_out) / (srednio_out - jasno_out); // Wartość funkcji przynależności
  } else if (sw_out >= srednio_out) {
    swOut[0] = 0; // Wartość funkcji przynależności
  }

  // Wartość funkcji przynależności dla srednio
  if (sw_out <= jasno_out || sw_out >= ciemno_out) {
    swOut[1] = 0; // Wartość funkcji przynależności
  } else if (sw_out >= jasno_out && sw_out <= srednio_out) {
    swOut[1] = (sw_out - jasno_out) / (srednio_out - jasno_out); // Wartość funkcji przynależności
  } else if (sw_out >= srednio_out && sw_out <= ciemno_out) {
    swOut[1] = (ciemno_out - sw_out) / (ciemno_out - srednio_out); // Wartość funkcji przynależności
  }

  // Wartość funkcji przynależności dla ciemno
  if (sw_out <= srednio_out) {
    swOut[2] = 0; // Wartość funkcji przynależności
  } else if (sw_out >= srednio_out && sw_out <= ciemno_out) {
    swOut[2] = (sw_out - srednio_out) / (ciemno_out - srednio_out); // Wartość funkcji przynależności
  } else if (sw_out >= ciemno_out) {
    swOut[2] = 1; // Wartość funkcji przynależności
  }
}

void reguly() {
  fuzzyOut();
  rule0 = swOut[0]; // (jasno_out)
  rule1 = swOut[1]; // (srednio_out)
  rule2 = swOut[2]; // (ciemno_out)
}

void deFuzzy() {
  reguly();
  a = (rule0 * RoletaPol) + (rule1 * RoletaDol) + (rule2 * Roletagora);

  b = rule0 + rule1 + rule2;
  outputDf = a / b;
}
