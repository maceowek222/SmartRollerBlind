#include <Stepper.h> //Biblioteka do obslugi silnika krokowego
// Piny do obslugi silnika krokowego

#define motorPin1  8
#define motorPin2  9
#define motorPin3  10
#define motorPin4  11
Stepper stepper(200, motorPin1, motorPin3, motorPin2, motorPin4);

const int sw_out_pin = A0;  // Pin czujnika zew
const int buttonPin = 6;

float sw_out = 0;  // Definicja zmiennej czujnika zew
float outputDf = 0;  // Definicja wyjścia algorytmu logiki rozmytej 

// Zmienne czujnik zew
float jasno_out = 80, srednio_out = 400, ciemno_out = 650;

float swOut[3];  // deklaracja tablicy pod reguly logiki rozmytej

float rule[3];
float rule0, rule1, rule2;

// Ilość kroków aktualnej pozycji rolety
int currentPosition = 0;

unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 5000;  // Pomiar co 5 sekund
float Roletagora = 120, RoletaPol = 60, RoletaDol = 1;  // Polozenie rolety
float a, b;

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(100);
  lastMeasurementTime = millis();  // Inicjalizacja czasu ostatniego pomiaru

  // Opuszczanie rolety do momentu naciśnięcia krańcówki
  pinMode(buttonPin, INPUT);
  while (digitalRead(buttonPin) == LOW) {
    stepper.step(-10);
  }

}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = currentMillis;  // Zaktualizuj czas ostatniego pomiaru
   sw_out = readSwOut();// Wykonanie pomiaru
    deFuzzy();//Wykonanie algorytmu logiki rozmytej 
  }

  if (outputDf > 0.0) {
    int targetPosition = map(outputDf, 0, 100, 0, 5000); // Zmapowany silnik
    int stepsToMove = targetPosition - currentPosition;//Sprawdzanie polozenia rolety na podstawie krokow silnika
    currentPosition = targetPosition;
    if (stepsToMove != 0) {
      stepper.step(stepsToMove);
    }
  }
//Opisywanie wynikow na w konsoli serial
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
  a = (rule0 * Roletagora) + (rule1 * RoletaPol) + (rule2 * RoletaDol);

  b = rule0 + rule1 + rule2;
  outputDf = a / b;
}