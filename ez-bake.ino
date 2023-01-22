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

// Fan variables
int pulse_duration1, last_pulse_duration1=10000; // Initialize to 10ms
int pulse_duration2, last_pulse_duration2=10000; // Initialize to 10ms
int rpm_fan_1;
int rpm_fan_2;
float hz_fan_1;
float hz_fan_2;
float fan_speed_cmd = 80;   // Fan speed initialized to 80%
bool measureFanBool1;
bool measureFanBool2;

void isr1() {
  if (measureFanBool1){
    pulse_duration1 = pulseIn(FAN_1_TACH_PIN, HIGH);
  }
  measureFanBool1 = 0;
}

void isr2() {
  if (measureFanBool2){
    pulse_duration2 = pulseIn(FAN_2_TACH_PIN, HIGH);
  }
  measureFanBool2 = 0;
}

void setup(void) {
  Serial.begin(SERIAL_BAUDRATE);
  sensors.begin();
  pinMode(OUTPUT_PIN, OUTPUT);
  // Write last heater state to the pin on startup so it is always correct
  digitalWrite(OUTPUT_PIN, lastHeaterState);

  // Set pinmodes and PWM frequencies for the fan control and tach pins
  analogWriteFrequency(FAN_1_CTRL_PIN, FAN_PWM_FREQ);
  analogWriteFrequency(FAN_2_CTRL_PIN, FAN_PWM_FREQ);
  pinMode(FAN_1_TACH_PIN, INPUT);
  pinMode(FAN_2_TACH_PIN, INPUT);
}

void loop(void) {
  if (isUnsafe) {
    digitalWrite(OUTPUT_PIN, LOW);
    return;
  }

  /*
  Example usage: send alphanumeric command over the serial port. MUST use "No line ending" in the serial monitor
  Sending 't133.5' changes the temperature setpoint to 133.5C
  Sending 'f89.4' changes the fan speed to 89.4% of full speed (3000 RPM)
  */

  if (Serial.available() >= 2)
  {
    char command = Serial.read();

    if (command=='t'){
      currentSetpointC = Serial.parseFloat();
    }
    else if (command == 'f') {
      fan_speed_cmd = Serial.parseFloat();
    }
    else {
      Serial.print(command);
      Serial.println(": Input not recognized, enter either a temperature ('t') or fan speed ('f') command");
    }

  // Set both fans to the desired speed
  analogWrite(FAN_1_CTRL_PIN, fan_speed_cmd/100*255);
  analogWrite(FAN_2_CTRL_PIN, fan_speed_cmd/100*255);

  // Measure Fan 1 RPM
  measureFanBool1 = 1;
  attachInterrupt(digitalPinToInterrupt(FAN_1_TACH_PIN), isr1, FALLING);
  delay(100); // 100 ms is long enough to cature one full cycle at the slowest speed
  detachInterrupt(digitalPinToInterrupt(FAN_1_TACH_PIN));
  
  if (pulse_duration1 < TACH_PULSE_MICROS_MIN){
    pulse_duration1 = last_pulse_duration1;
  }
  last_pulse_duration1 = pulse_duration1;
  
  // Divide by 2 because pulsedurationN measures the width of 1/2 the square wave
  hz_fan_1 = 1/(pulse_duration1/MICROS2SEC)/2;
  rpm_fan_1 = hz_fan_1/2*HZ2RPM;  // Divide by 2 because fan tach registers 2 hall effect pulses per rev
  rpm_fan_1 = rpm_fan_1 - (rpm_fan_1%10);  // Only care about 10s of revs

  // Measure Fan 2 RPM
  measureFanBool2 = 1;
  attachInterrupt(digitalPinToInterrupt(FAN_2_TACH_PIN), isr2, FALLING);
  delay(100);
  detachInterrupt(digitalPinToInterrupt(FAN_2_TACH_PIN));

  if (pulse_duration1 < TACH_PULSE_MICROS_MIN){
    pulse_duration1 = last_pulse_duration1;
  }
  last_pulse_duration1 = pulse_duration1;
  
  // Divide by 2 because pulsedurationN measures the width of 1/2 the square wave
  hz_fan_1 = 1/(pulse_duration1/MICROS2SEC)/2;
  rpm_fan_1 = hz_fan_1/2*HZ2RPM;  // Divide by 2 because fan tachregisters 2 hall effect pulses per rev
  rpm_fan_1 = rpm_fan_1 - (rpm_fan_1%10);

  Serial.print("Fan Speed Percent CMD = ");
  Serial.print(fan_speed_cmd);
  Serial.print("\t Fan 1 RPM = ");
  Serial.println(rpm_fan_1);
  Serial.print("\t Fan 2 RPM = ");
  Serial.println(rpm_fan_2);

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
