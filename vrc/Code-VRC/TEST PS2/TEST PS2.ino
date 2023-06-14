#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define PWMMOTOR_1_CHAN_1 8
#define PWMMOTOR_1_CHAN_2 9
#define PWMMOTOR_2_CHAN_1 14
#define PWMMOTOR_2_CHAN_2 15

#define PWMMOTOR_3_CHAN_1 10
#define PWMMOTOR_3_CHAN_2 11

#define PWMSERVO_1 2
#define PWMSERVO_2 3
#define PWMSERVO_3 4
#define PWMSERVO_4 5

#define SERVO_MIN 125
#define SERVO_MAX 600

#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK
#define ENA       //ENA
#define ENB      //ENB

#define pressures false
#define rumble false

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;


uint8_t buttonState = ps2x.Button(PSB_TRIANGLE);

void setup(){
  pwm.begin(); //khởi tạo PCA9685
  pwm.setOscillatorFrequency(27000000); // cài đặt tần số giao động
  pwm.setPWMFreq(50);// cài đặt tần số PWM servo

  Wire.setClock(400000);
  Serial.begin(9600);
  Serial.print("Ket noi voi tay cam PS2:");

  int error = -1;
  for (int i = 0; i < 10; i++){
    delay(1000); // đợi 1 giây
    // cài đặt chân và các chế độ: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
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
void loop(){
  Serial.println(buttonState);
  delay(1000);
}