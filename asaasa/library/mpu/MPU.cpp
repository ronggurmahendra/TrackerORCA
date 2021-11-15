// Versi 29 Agustus 2021

#include "MPU.h"


MPU_6050::MPU_6050(){
}

void MPU_6050::setTlast(uint64_t t)
{
    this->tlast = t;
}

uint64_t MPU_6050::getTlast(){
    return this->tlast;
}

void MPU_6050::setupMPU()
{

    //Komunikasi awal dengan MPU 6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    //Atur akselerometer (+/-8g)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    //Atur gyro (250dps)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Atur sensor temperatur
    Wire.beginTransmission(0x68);
    Wire.write(0x41);
    Wire.endTransmission();

}

void MPU_6050::readMPU()
{
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    // Baca 14 byte data MPU
    Wire.requestFrom(0x68, 14);
    if (!Wire.available()){
        Serial.println("MPU tidak terhubung. Mohon cek kabel yang terkoneksi ke MPU.");
    }

    if (Wire.available() == 14){
      Accel.X = (int16_t)(Wire.read() << 8 | Wire.read());
      Accel.Y = (int16_t)(Wire.read() << 8 | Wire.read());
      Accel.Z = (int16_t)(Wire.read() << 8 | Wire.read());
      temperature = (int16_t)(Wire.read() << 8 | Wire.read());
      Gyro.X = (int16_t)(Wire.read() << 8 | Wire.read());
      Gyro.Y = (int16_t)(Wire.read() << 8 | Wire.read());
      Gyro.Z = (int16_t)(Wire.read() << 8 | Wire.read());
    }
}

float MPU_6050::getPitch()
{
    return Attitude.pitch;
}

float MPU_6050::getRoll()
{
    return Attitude.roll;
}

float MPU_6050::getYaw(){
    return Attitude.yaw;
}

void MPU_6050::calibrate_gyro()
{
    // Kalibrasi gyro
    Serial.println("Kalibrasi gyro. Jaga sensor agar tetap diam.");
    for (int i=0; i<1000; i++){
        readMPU();
        Gyro_cal.X += Gyro.X;
        Gyro_cal.Y += Gyro.Y;
        Gyro_cal.Z += Gyro.Z;
        delay(1);
        if (i % 50 == 0) Serial.print(".");
    }
    Serial.println();

    Gyro_cal.X /= 1000;
    Gyro_cal.Y /= 1000;
    Gyro_cal.Z /= 1000;
}

void MPU_6050::update(){
    readMPU();

    Gyro.X -= Gyro_cal.X;
    Gyro.Y -= Gyro_cal.Y;
    Gyro.Z -= Gyro_cal.Z;

    // Pembagian angka yang terdapat di gyro dengan LSB (Least significant bit)
    // Format Gyro dalam derajat/s
    Gyro.X /= GYRO_LSB;
    Gyro.Y /= GYRO_LSB;
    Gyro.Z /= GYRO_LSB;

    // Format Accel dalam g
    Accel.X /= ACCEL_LSB;
    Accel.Y /= ACCEL_LSB;
    Accel.Z /= ACCEL_LSB;

    Attitude.pitch = GYRO_COEF * (Attitude.pitch + Gyro.X/(float)FREQ_MPU) + (1.0 - GYRO_COEF) * atan2(Accel.Y, Accel.Z) * 180/PI;
    Attitude.roll = GYRO_COEF * (Attitude.roll + Gyro.Y/(float)FREQ_MPU) + (1.0 - GYRO_COEF) * atan2(-1.0 * Accel.X, sqrt(Accel.Y*Accel.Y + Accel.Z*Accel.Z)) * 180/PI;
    Attitude.yaw = Attitude.yaw + Gyro.Z/(float)FREQ_MPU;
}

void MPU_6050::boot(){
    setupMPU();
    calibrate_gyro();
}

void MPU_6050::debug(){
    Serial.print("Gyro|X: ");
    Serial.print(Gyro.X);
    Serial.print("  Y: ");
    Serial.print(Gyro.Y);
    Serial.print("  Z: ");
    Serial.print(Gyro.Z);
    Serial.print("      Acc|X: ");
    Serial.print(Accel.X);
    Serial.print("  Y: ");
    Serial.print(Accel.Y);
    Serial.print("  Z: ");
    Serial.print(Accel.Z);
    Serial.print("      COMP|P: ");
    Serial.print(Attitude.pitch);
    Serial.print("  R: ");
    Serial.print(Attitude.roll);
    Serial.println();
}
