#pragma once

#include <Wire.h>

//MPU6500のi2cのアドレス
#define MPU_ADDR 0x68


//MPU6500レジスタ直読み

//書き込み関数

void RegWrite(uint8_t addr, uint8_t data){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

void beginMPU(int SCL_PIN, int SDA_PIN){
  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.beginTransmission(MPU_ADDR);
  // WHO_AM_I レジスタ
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    byte c = Wire.read();
    Serial.print("WHO_AM_I = 0x");
    Serial.println(c, HEX);
    //MPU6500のwhoamiは0x70
  } else {
    Serial.println("Failed to read WHO_AM_I");
  }

  RegWrite(0x6B, 0x00);
}

void setRange(){
  RegWrite(0x1B, 0x08);
  RegWrite(0x1c, 0x00);

  RegWrite(0x1A, 0x05);
  RegWrite(0x1D, 0x05);
}

const float TEMP_SENS = 333.87f; // LSB/°C (データシートより)

float ax, ay, az, gx, gy, gz;

void readData(){

  //加速度と角速度は上位1バイトと下位1バイトを併せて16ビットの数値として扱う

  int16_t temp, rax, ray, raz, rgx, rgy, rgz;

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);  // デバイス内部の読み出し開始レジスタ番地を指定
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() != 14) return;

  rax = (Wire.read() << 8) | Wire.read();
  ray = (Wire.read() << 8) | Wire.read();
  raz = (Wire.read() << 8) | Wire.read();

  temp = (Wire.read() << 8) | Wire.read();
  temp = (temp / TEMP_SENS) + 21.0f; // データシートより

  rgx = (Wire.read() << 8) | Wire.read(); // GYRO_XOUT_H, GYRO_XOUT_L
  rgy = (Wire.read() << 8) | Wire.read(); // GYRO_YOUT_H, GYRO_YOUT_L
  rgz = (Wire.read() << 8) | Wire.read(); // GYRO_ZOUT_H, GYRO_ZOUT_L

  //センサーの生データを物理量に変換
  ax = rax / 16384.0f;
  ay = ray / 16384.0f;
  az = raz / 16384.0f;

  gx = rgx / 65.5f;
  gy = rgy / 65.5f;
  gz = rgz / 65.5f;

  /*
  Serial.print("  AX : "); Serial.print(ax);
  Serial.print("  AY : "); Serial.print(ay);
  Serial.print("  AZ : "); Serial.print(az);

  Serial.print(" TEMP : "); Serial.print(temp);

  Serial.print("  GX : "); Serial.print(gx);
  Serial.print("  GY : "); Serial.print(gy);
  Serial.print("  GZ : "); Serial.println(gz);
  */

}