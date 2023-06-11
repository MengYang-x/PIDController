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
  Serial.print("angleZ : ");
  Serial.println(mpu6050.getAngleZ()); // 偏航角
  Serial.print("\t GyroZ : ");
  Serial.println(mpu6050.getGyroZ()); // 偏航角速度
  delay(50);
}