#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define DC_MOTOR_CHAN_1 8
#define DC_MOTOR_CHAN_2 9
#define DC_MOTOR_2_CHAN_1 4
#define DC_MOTOR_2_CHAN_2 15

#define LIFT_MOTOR_CHAN_1 10
#define LIFT_MOTOR_CHAN_2 11

#define PUSH_MOTOR_CHAN_1 12
#define PUSH_MOTOR_CHAN_2 13

#define LEFT_SERVO_360 3
#define RIGHT_SERVO_360 4
#define FRONT_SERVO_180 5
#define BACK_SERVO_180 7

#define SERVO_180_MIN 80
#define SERVO_180_MAX 575
#define SERVO_360_MIN 105
#define SERVO_360_MAX 495

#define SEN_1_PIN 39
#define SEN_2_PIN 36
#define SEN_3_PIN 2
#define SEN_4_PIN 32

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define ENA
#define ENB

#define pressures false
#define rumble false

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;

uint8_t value = 0;
uint8_t valueTriangle = 0;
uint16_t buttonState_CROSS = 0;
uint16_t buttonState_TRIANGLE = 0;

void setup(){
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);

  pinMode(SEN_1_PIN, INPUT);
  pinMode(SEN_2_PIN, INPUT);
  pinMode(SEN_3_PIN, INPUT);
  pinMode(SEN_4_PIN, INPUT);

  pwm.setPWM(3, 0, 0);
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(5, 0, SERVO_180_MIN);
  pwm.setPWM(7, 0, SERVO_180_MIN);

//Wire.setClock(400000);
  Serial.begin(115200);
  Serial.print("Ket noi voi tay cam PS2:");

  int error = -1;
  for (int i = 0; i < 10; i++){
    delay(200);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }
  switch (error) {
  case 0:
    Serial.println(" Ket noi tay cam PS2 thanh cong");
    break;
  case 1:
    Serial.println(" LOI: Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
    break;
  case 2:
    Serial.println(" LOI: khong gui duoc lenh");
    break;
  case 3:
    Serial.println(" LOI: Khong vao duoc Pressures mode ");
    break;
  }
}

//Movement & Function 

void controlDC(uint16_t leftSpeed, uint16_t rightSpeed){
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, leftSpeed);
  pwm.setPWM(14, 0, 0);
  pwm.setPWM(15, 0, rightSpeed);
}
void controlBackward(){
  pwm.setPWM(8, 0, 2000);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(14, 0, 2000);
  pwm.setPWM(15, 0, 0);
}
void turnLeft(){
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 2000);
  pwm.setPWM(14, 0, 2000);
  pwm.setPWM(15, 0, 0);
}
void turnRight(){
  pwm.setPWM(8, 0, 2000);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(14, 0, 0);
  pwm.setPWM(15, 0, 2000);
}
void liftDC(uint16_t liftSpeed, uint16_t unliftSpeed){
  pwm.setPWM(10, 0, liftSpeed);
  pwm.setPWM(11, 0, unliftSpeed);
}
void pushDC(uint16_t downSpeed, uint16_t upSpeed){
  pwm.setPWM(12, 0, downSpeed);
  pwm.setPWM(13, 0, upSpeed);
}

//Line Following

char cRead_Sensor()
{
  short sSen_Pin[4] = {SEN_1_PIN, SEN_2_PIN, SEN_3_PIN, SEN_4_PIN};
  char cStatus = 0b00000000;

  for(short i=0; i<4; i++)
  {
    cStatus = cStatus | (digitalRead(sSen_Pin[i]) << i);
  }

  return cStatus;
}
void lineFollowing(){
  char cSen_Status = 0b00000000;
  cSen_Status = cRead_Sensor();
  if((cSen_Status == 0b00001001) || (cSen_Status == 0b00001011) || (cSen_Status == 0b00001101))
  {
    Serial.println("-----|-----");
  }
  else if((cSen_Status == 0b00001110) || (cSen_Status == 0b00001100) || (cSen_Status == 0b00001000))
  {
    Serial.println("----------|");
  }
  else if((cSen_Status == 0b00000111) || (cSen_Status == 0b00000011) || (cSen_Status == 0b00000001))
  {
    Serial.println("|----------");
  }
  else if(cSen_Status == 0b00001111)
  {
    Serial.println("-----------");
  }
}

// Control

void loop(){
  ps2Control();
}

void ps2Control(){
  ps2x.read_gamepad(false, false);

  // Movement

  // MOVE FORWARD
  if(ps2x.Button(PSB_PAD_UP)){
    controlDC(2000, 2000);
  };
  // TURN RIGHT
  if(ps2x.Button(PSB_PAD_RIGHT)){
    turnLeft();
  };
  // TURN LEFT
  if(ps2x.Button(PSB_PAD_LEFT)){
    turnRight();
  };
  // MOVE BACK
  if(ps2x.Button(PSB_PAD_DOWN)){
    controlBackward();       
  };
  if(ps2x.ButtonReleased(PSB_PAD_UP) || ps2x.ButtonReleased(PSB_PAD_DOWN) || ps2x.ButtonReleased(PSB_PAD_LEFT) || ps2x.ButtonReleased(PSB_PAD_RIGHT)){
    controlDC(0, 0);
  };

  // Function Motors
  
  if (ps2x.Analog(PSS_LY) == 255){
    liftDC(2300, 0);
  }
  else if (ps2x.Analog(PSS_LY) == 0){
    liftDC(0, 2300);
  }
  else{
    liftDC(0, 0);
  }
  if (ps2x.Button(PSB_L1)){
    pushDC(800, 0);
  }
  else if (ps2x.Button(PSB_R1)){
    pushDC(0, 800);
  }
  else if (ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_R1)){
    pushDC(0, 0);
  }

  //Button

  if (ps2x.NewButtonState()){
    if (ps2x.Button(PSB_CROSS)){
      buttonState_CROSS++;
      Serial.println(buttonState_CROSS);
    }
    if (ps2x.Button(PSB_TRIANGLE)){
      buttonState_TRIANGLE++;
      Serial.println(buttonState_TRIANGLE);
    }
  }

  //Automatic
  valueTriangle = buttonState_TRIANGLE % 2;
  switch (valueTriangle){
    case 0:
      break;
    case 2:
      controlDC(0, 0);
      break;
    case 1:
      lineFollowing();
      break;
  }

  //SERVO 180 (Spin Turbin)
  
  if (buttonState_CROSS % 2 == 0){
    pwm.setPWM(5, 0, SERVO_180_MIN);
    pwm.setPWM(7, 0, SERVO_180_MIN);
  }
  else if (buttonState_CROSS % 2 != 0){
    pwm.setPWM(5, 0, SERVO_180_MAX);
    pwm.setPWM(7, 0, SERVO_180_MAX);
  }

  // SERVO 360

  value = ps2x.Analog(PSS_RY);
  if(value < 50){
    pwm.setPWM(3, 0, SERVO_360_MAX);
    pwm.setPWM(4, 0, SERVO_360_MIN);
  }
  else if(value > 200){
    pwm.setPWM(3, 0, SERVO_360_MIN);
    pwm.setPWM(4, 0, SERVO_360_MAX);
  }
  else{
    pwm.setPWM(3, 0, 0);
    pwm.setPWM(4, 0, 0);
  }
  
  delay(50);
  
}