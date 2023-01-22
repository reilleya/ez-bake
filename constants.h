#define ONE_WIRE_BUS 2
#define NUM_THERMOCOUPLES 1
#define OUTPUT_PIN 3
#define SERIAL_BAUDRATE 9600
#define DEBOUNCE_LOOPS 20

#define MIN_LEGAL_TEMP_C -20
#define MAX_LEGAL_TEMP_C 150

// Fan control and feedback
#define FAN_1_CTRL_PIN 4
#define FAN_2_CTRL_PIN 5
#define FAN_1_TACH_PIN 6
#define FAN_2_TACH_PIN 7
#define TACH_PULSE_MICROS_MIN 1000
#define HZ2RPM 60
#define MICROS2SEC 1000000.

#define FAN_PWM_FREQ 25000

typedef float tempCArray[NUM_THERMOCOUPLES];