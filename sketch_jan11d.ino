#include <PID_v1.h>
#include <AFMotor.h>
int RPWM = 5;
int LPWM = 6;
int L_EN = 2;
int R_EN = 13;
AF_DCMotor motor(4);
boolean pin_state[10];
//byte input_pin[] = {1, 2, 3, 4, 9, 10, 11, 12, 13};
int dec_position = 0;
int dc = 0;
double kp = 50, ki = 45, kd = 2;
double input = 0, output = 0, setpoint = 0;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
 // pinMode(RPWM, OUTPUT);
 // pinMode(LPWM, OUTPUT);
  //for (byte i = 0; i < 9; i++) {
   // pinMode(input_pin[i], INPUT);
//  }
  TCCR1B = TCCR1B & 0b11111000 | 1; 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);          
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String baca = Serial.readString();
    setpoint = baca.toInt();
  }
  ReadEncoder();
  input = dc;
  myPID.Compute();
  pwmOut(output);
}

void pwmOut(int out) {
  if (out > 0) {
   // analogWrite(RPWM, out);//Sets speed variable via PWM
   motor.run(FORWARD);
   motor.setSpeed(out);
  }
  else {
    //analogWrite(LPWM, abs(out));//Sets speed variable via PWM
     motor.run(BACKWARD);
   motor.setSpeed(abs(out));
  }
}

void ReadEncoder() {
// FOR READING ENCODER POSITION, GIVING 0-359 OUTPUT CORRESPOND TO THE ENCODER POSITION
 // for (byte i = 0; i < 9; i++) {
//    pin_state[i] = !(digitalRead(input_pin[i]));
  }
  //dec_position = (pin_state[8] * 256) + (pin_state[7] * 128) + (pin_state[6] * 64) + (pin_state[5] * 32) + (pin_state[4] * 16) + (pin_state[3] * 8) + (pin_state[2] * 4) + (pin_state[1] * 2) + pin_state[0];
  dc = map(dec_position, 0, 500, 0, 360);
}
