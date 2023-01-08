#include <OneWire.h>
#include <DallasTemperature.h>
#include "constants.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
tempCArray temperatures;
bool isUnsafe = false;
float currentSetpointC = 0;
uint8_t loopsSinceStateChange = DEBOUNCE_LOOPS;
bool lastHeaterState = false;

void setup(void) {
  Serial.begin(SERIAL_BAUDRATE);
  sensors.begin();
  pinMode(OUTPUT_PIN, OUTPUT);
  // Write last heater state to the pin on startup so it is always correct
  digitalWrite(OUTPUT_PIN, lastHeaterState);
}

void loop(void) {
  if (isUnsafe) {
    digitalWrite(OUTPUT_PIN, LOW);
    return;
  }

  if (Serial.available() > 0) {
    currentSetpointC = Serial.parseFloat();
  }

  updateTemperatures(temperatures);
  serialPrintTemperatures(temperatures);
  if (!areAllTemperaturesValid(temperatures)) {
    Serial.println("Illegal temperature detected, entering safety mode");
    isUnsafe = true;
    return;
  }
  
  bool nextHeaterState = getHeaterControlState(temperatures, currentSetpointC);
  loopsSinceStateChange = min(loopsSinceStateChange + 1, DEBOUNCE_LOOPS);
  if (nextHeaterState == lastHeaterState || loopsSinceStateChange < DEBOUNCE_LOOPS) {
    return; // Either we don't want to change our control state, or aren't allowed to yet
  }

  digitalWrite(OUTPUT_PIN, nextHeaterState);
  loopsSinceStateChange = 0;
  lastHeaterState = nextHeaterState;
}

bool areAllTemperaturesValid(tempCArray temperatures) {
  for (int8_t i = 0; i < NUM_THERMOCOUPLES; i++) {
    float tempC = temperatures[i];

    if (tempC == DEVICE_DISCONNECTED_C ||
        tempC < MIN_LEGAL_TEMP_C ||
        tempC > MAX_LEGAL_TEMP_C) {
      return false;
    }
  }
  return true;
}

void updateTemperatures(tempCArray temperatures) {
  sensors.requestTemperatures();
  for (int8_t i = 0; i < NUM_THERMOCOUPLES; i++ ) {
    temperatures[i] = sensors.getTempCByIndex(i);
  }
}

void serialPrintTemperatures(tempCArray temperatures) {
  for (int i = 0; i < NUM_THERMOCOUPLES; i++ ) {
      Serial.print(temperatures[i]);
      Serial.print(" ");
  }
  Serial.println();
}

bool getHeaterControlState(tempCArray temperatures, float setpointC) {
  // Get the mean temperature
  float meanTemperatureC = 0;
  for (int i = 0; i < NUM_THERMOCOUPLES; i++ ) {
      meanTemperatureC += temperatures[i];
  }
  meanTemperatureC /= NUM_THERMOCOUPLES;

  // Simple bang-bang control
  return meanTemperatureC < setpointC;
}
