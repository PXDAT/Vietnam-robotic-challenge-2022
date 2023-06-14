#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SEN_1_PIN 39
#define SEN_2_PIN 36
#define SEN_3_PIN 2
#define SEN_4_PIN 32

void goForward()
{
    pwm.setPWM(8, 0, 255);
    pwm.setPWM(9, 4096, 0);
    pwm.setPWM(10, 0, 255);
    pwm.setPWM(11, 4096, 0);
}

void goBackward()
{
    pwm.setPWM(8, 4096, 0);
    pwm.setPWM(9, 0, 255);
    pwm.setPWM(10, 4096, 0);
    pwm.setPWM(11, 0, 255);
}

void turnLeft()
{
    pwm.setPWM(8, 4096, 0);
    pwm.setPWM(9, 0, 255);
    pwm.setPWM(10, 0, 255);
    pwm.setPWM(11, 4096, 0);

}

void turnRight()
{
    pwm.setPWM(8, 0, 255);
    pwm.setPWM(9, 4096, 0);
    pwm.setPWM(10, 4096, 0);
    pwm.setPWM(11, 0, 255);
}


void lineFollowing(){
  uint8_t valueSen1 = digitalRead(SEN_1_PIN);
  uint8_t valueSen2 = digitalRead(SEN_2_PIN);
  uint8_t valueSen3 = digitalRead(SEN_3_PIN);
  uint8_t valueSen4 = digitalRead(SEN_4_PIN);
  Serial.print(valueSen1);
  Serial.print(" ");
  Serial.print(valueSen2);
  Serial.print(" ");
  Serial.print(valueSen3);
  Serial.print(" ");
  Serial.print(valueSen4);
  Serial.println(" ");
}
void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);

    pinMode(SEN_1_PIN,INPUT);
    pinMode(SEN_2_PIN,INPUT);
    pinMode(SEN_3_PIN,INPUT);
    pinMode(SEN_4_PIN,INPUT);
}

void loop() 
{
    lineFollowing();
}