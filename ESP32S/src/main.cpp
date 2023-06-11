#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  // mpu6050.calcGyroOffsets(true);
}
void loop()
{
  mpu6050.update();

  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());   // 转向相关，偏航角随着时间的增加而增加。这个数据没法用
  delay(50);
}