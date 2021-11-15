#ifndef MPU_h
#define MPU_h

#include "Arduino.h"
#include <Wire.h>

#define FREQ_MPU 50
#define CALIB_FREQ 50
#define GYRO_LSB 131.0
#define ACCEL_LSB 4096.0
#define GYRO_COEF 0.95

typedef struct 
{
    float X;
    float Y;
    float Z;
} XYZ;

typedef struct 
{
    float roll;
    float pitch;
    float yaw;
} RPY;

class MPU_6050
{
public:
    MPU_6050();
    void boot();
    void update();
    void setupMPU();
    void readMPU();
    void calibrate_gyro();
    uint64_t getTlast();
    void setTlast(uint64_t t);
    void debug();
    float getRoll();
    float getPitch();
    float getYaw();

private:
    uint64_t tlast;
    uint64_t tnow;
    float temperature;
    XYZ Accel;
    XYZ Gyro;
    XYZ Gyro_cal;
    RPY Attitude;
};

#endif