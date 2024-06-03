#include <Servo.h>
#include "CytronMotorDriver.h"
#include "bixp.c"

#define SHAKER_MOTOR_PWM 1
#define SHAKER_MOTOR_DIR 2
#define CONVEYOR_MOTOR_PWM 3
#define CONVEYOR_MOTOR_DIR 4
#define GATE_SERVO_PIN 5
#define BOX_FAN_PWM 6

#define SERIAL_BUFFER_MAX_LENGTH 256

// Configure the motor drivers
CytronMD shaker_motor(PWM_DIR, SHAKER_MOTOR_PWM, SHAKER_MOTOR_DIR);
CytronMD conveyor_motor(PWM_DIR, CONVEYOR_MOTOR_PWM, CONVEYOR_MOTOR_DIR);

// Configure servo motor
Servo gate_servo;

// Error flag indicates unrecoverable error, which if set will have the arduino disable both motors
// and not turn on again until power is disconnected.
int err = 0;

char serial_buffer[SERIAL_BUFFER_MAX_LENGTH] = { 0 };
char serial_checksum[SERIAL_BUFFER_MAX_LENGTH] = { 0 };
unsigned int serial_buffer_len = 0;
unsigned int serial_checksum_len = 0;
char bixp_state = BIXP_STATE_DEFAULT;
unsigned long bananas_time = millis();
char beans_ready = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(9,OUTPUT); // Timer 1A
  setupTimer1();
  gate_servo.attach(GATE_SERVO_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:  
  if (err != 0) {
    shaker_motor.setSpeed(0);
    conveyor_motor.setSpeed(0);
    return;
  }

  char response;
  unsigned int bixp_rc;
  int rc;

  while (Serial.available()) {
    char next_byte = Serial.read();
    bixp_rc = bixp_handle_serial(serial_buffer, SERIAL_BUFFER_MAX_LENGTH, &serial_buffer_len, serial_checksum, SERIAL_BUFFER_MAX_LENGTH, &serial_checksum_len, &bixp_state, next_byte, &response);

    if (bixp_rc & BIXP_HANDLE_SERIAL_RESPONSE) Serial.write(response);
    else if (bixp_rc & BIXP_HANDLE_SERIAL_BANANAS) bananas_time = millis();
    else if (bixp_rc & (BIXP_HANDLE_SERIAL_UNHANDLED | BIXP_HANDLE_SERIAL_NO_BEANS | BIXP_HANDLE_SERIAL_CKSUM_FAILED)) {
      err = 1;
      return;
    } else if (bixp_rc & BIXP_HANDLE_SERIAL_BEANS_READY) {
      beans_ready = 1;
    }
  }

   if (beans_ready) {
    rc = handle_command(serial_buffer);
    if (rc != 0) {
      err = 1;
      return;
    }
   }
}

// first byte defines device ID (0: conveyor motor, 1: shaker motor, 2: servo motor, 3: box fan),
// second and third bytes, s and t, define value, v, where v = s << 8 + t
int handle_command(char cmd[]) {
  int value = cmd[1] << 8 + cmd[2];

  switch (cmd[1]) {
    case 0: conveyor_motor.setSpeed(value); break;
    case 1: shaker_motor.setSpeed(value); break;
    case 3: gate_servo.write(value); break;
    case 4: setPWM1A((float)value/100); break;
  }
}

//configure Timer 1 (pins 9,10) to output 25kHz PWM
void setupTimer1(){
    //Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM13);
    ICR1 = 320;
    OCR1A = 0;
    OCR1B = 0;
}

//equivalent of analogWrite on pin 9
void setPWM1A(float f){
    f=f<0?0:f>1?1:f;
    OCR1A = (uint16_t)(320*f);
}
