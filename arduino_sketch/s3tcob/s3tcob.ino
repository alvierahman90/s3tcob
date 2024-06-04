#include <Servo.h>
#include "CytronMotorDriver.h"

#define SHAKER_MOTOR_PWM 5
#define SHAKER_MOTOR_DIR 8
#define CONVEYOR_MOTOR_PWM 3
#define CONVEYOR_MOTOR_DIR 4
#define GATE_SERVO_PIN 6
#define BOX_FAN_PWM 9

#define SERIAL_BUFFER_MAX_LENGTH 256



// Configure the motor drivers
CytronMD shaker_motor(PWM_DIR, SHAKER_MOTOR_PWM, SHAKER_MOTOR_DIR);
CytronMD conveyor_motor(PWM_DIR, CONVEYOR_MOTOR_PWM, CONVEYOR_MOTOR_DIR);
// Configure servo motor
Servo gate_servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
      pinMode(9,OUTPUT); // Timer 1A
  setupTimer1();
    gate_servo.attach(GATE_SERVO_PIN);
  Serial.println("OK: Online!");
  gate_servo.write(20);
}

void loop() {
  if (!Serial.available()) return;
  
  String buf = Serial.readStringUntil('\n');
  if (buf.length() < 5) {
    Serial.println("ERR: Input too short!");
    return;
  }

  char object = buf[0];
  int value = ((buf[1] - '0') * 1000) + ((buf[2] - '0') * 100) + ((buf[3] - '0') * 10) + (buf[4] - '0');
  

  Serial.print("OK: object: ");
  Serial.print(object);
  Serial.print(" value: ");
  Serial.println(value);

  switch (object) {
    case 's': shaker_motor.setSpeed(value); break;
    case 'c': conveyor_motor.setSpeed(value); break;
    case 'f': Serial.println("OK: setting fan"); setPWM1A((float)value / 1024.0f); break;
    case 'g': gate_servo.write(value); break;
    default: Serial.print("ERR: Undefined object!: "); Serial.print(object);
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
