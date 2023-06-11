#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

// void setup()
// {
//   Serial.begin(115200);
//   Wire.begin();
//   mpu6050.begin();
//   // mpu6050.calcGyroOffsets(true);
// }
// void loop()
// {
//   mpu6050.update();
//   Serial.print("angleZ : ");
//   Serial.println(mpu6050.getAngleZ()); // 偏航角
//   Serial.print("\t GyroZ : ");
//   Serial.println(mpu6050.getGyroZ()); // 偏航角速度
//   delay(50);
// }

// 电机死区测试
const int AIN1 = 26;
const int AIN2 = 26;
const int BIN1 = 26;
const int BIN2 = 26;
int pwm = 0;

void motor(int leftEn, int rightEn)
{
  if (leftEn == 0)
  {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
  }
  if (leftEn < 0)
  {
    if (leftEn < -255)
    {
      leftEn = -255;
    }
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0 - leftEn);
  }
  if (leftEn > 0)
  {
    if (leftEn > 255)
    {
      leftEn = 255;
    }
    analogWrite(AIN1, leftEn);
    analogWrite(AIN2, 0);
  }

  // 右电机
  if (rightEn == 0)
  {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
  if (rightEn < 0)
  {
    if (rightEn < -255)
    {
      rightEn = -255;
    }
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0 - rightEn);
  }
  if (rightEn > 0)
  {
    if (rightEn > 255)
    {
      rightEn = 255;
    }
    analogWrite(BIN1, rightEn);
    analogWrite(BIN2, 0);
  }
}

void serialDebug()
{
  if (Serial.available() > 0)
  {
    delay(5);
    char data = Serial.read();
    switch (data)
    {
    case '1':
      pwm++;
      Serial.print("pwm: ");
      Serial.println(pwm);
      break;
    case '0':
      pwm--;
      Serial.print("pwm: ");
      Serial.println(pwm);
      break;
    }
  }
}

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  serialDebug();
  motor(pwm, pwm);
}
