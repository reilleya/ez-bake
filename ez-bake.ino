#include <OneWire.h>
#include <DallasTemperature.h>
#include "constants.h"

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

void setup(void) {
  Serial.begin(SERIAL_BAUDRATE);
  sensors.begin();
}

void loop(void) {
  sensors.requestTemperatures();
  for (int i = 0; i < NUM_THERMOCOUPLES; i++ ) {
    float tempC = sensors.getTempCByIndex(i);

    if (tempC != DEVICE_DISCONNECTED_C) {
      Serial.print(tempC);
      Serial.print(" ");
    } else {
      Serial.print("X ");
    }
  }
  Serial.println();
}
