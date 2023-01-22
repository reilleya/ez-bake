const int analogOutPin = 9; // Analog output pin that the LED is attached to
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

int fanTachPin = 10;

int pulse_duration, last_pulse_duration=10000; // Initialize to 10ms
int rpm;
float hz;
bool measureFanBool;

float temp_setpointC = 0;
float fan_speed_cmd = 80;

void isr() {
  if (measureFanBool){
    pulse_duration = pulseIn(fanTachPin, HIGH);
  }
  measureFanBool = 0;
}

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  analogWriteFrequency(9, 25000);
  pinMode(fanTachPin, INPUT);
}

void loop() {

  if (Serial.available() >= 2)
  {
    char command = Serial.read();

    if (command=='t'){
      temp_setpointC = Serial.parseFloat();
    }
    else if (command == 'f') {
      fan_speed_cmd = Serial.parseFloat();
    }
    else {
      Serial.print(command);
      Serial.println(": Input not recognized, enter either a temperature ('t') or fan speed ('f') command");
    }
    
  }
  
  analogWrite(analogOutPin, fan_speed_cmd/100*255);

  measureFanBool = 1;
  attachInterrupt(digitalPinToInterrupt(fanTachPin), isr, FALLING);
  delay(100);
  detachInterrupt(digitalPinToInterrupt(fanTachPin));
  if (pulse_duration < 1000){
    pulse_duration = last_pulse_duration;
  }
  last_pulse_duration = pulse_duration;
  
  hz = 1/(pulse_duration/1000000.)/2;
  rpm = hz/2*60;
  rpm = rpm - (rpm%10);
  
  Serial.print("Temp s/etpoint C = ");
  Serial.print(temp_setpointC);
  Serial.print("Fan Speed Percent CMD = ");
  Serial.print(fan_speed_cmd);
  Serial.print("\t Fan RPM = ");
  Serial.println(rpm);
    
}
