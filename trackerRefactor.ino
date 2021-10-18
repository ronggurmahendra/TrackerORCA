
//===== Include Libraries ========================
//#include "./library/math/math.h"
#include "param.h"

//#include "./library/i2cdevlib/Arduino/MPU6050/MPU6050.h"
#include "./library/mpu/MPU.cpp"

#include <Servo.h>

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <TaskScheduler.h>


//===== Define objects ========================

MPU_6050 mpu;
Servo servo_yaw;
Servo servo_pitch;

// ==== Scheduler ==============================
Scheduler ts;
void t_MavlinkHandler_callback();
void t_SensorHandler_callback();
void t_ServoHandler_callback();
Task t_MavlinkHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_MavlinkHandler_callback, &ts, true);
Task t_SensorHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_SensorHandler_callback, &ts, true);
Task t_ServoHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_ServoHandler_callback, &ts, true);

//===== Debugging macros ========================
#ifdef _DEBUG_
#define SerialD Serial
#define _PM(a) SerialD.print(millis()); SerialD.print(": "); SerialD.println(a)
#define _PP(a) SerialD.print(a)
#define _PL(a) SerialD.println(a)
#define _PX(a) SerialD.println(a, HEX)
#else
#define _PM(a)
#define _PP(a)
#define _PL(a)
#define _PX(a)
#endif



void setup(){
  Serial.begin(115200);
   //INISIALISASI SERVO
  servo_yaw.attach(PinServoYaw);
  servo_pitch.attach(PinServoPitch);

  // INISIALISASI SENSOR  
  Wire.begin();
  mpu.boot();
  
  //Calibration
  YawServoCalibration();
  PitchServoCalibration();
  TrackerPos();
  Serial.println("Setup DONE ...");
}

void loop(){
  ts.execute();
}

void YawServoCalibration(){ // disini cari value servo_yaw_max dan servo_yaw_min (di 0 derajat dan 360 derajat)
  _PM("SETUP YawServoCalibration");
  //Matek magnetometer


  
}
void PitchServoCalibration(){// disini cari value servo_pitch_max dan servo_pitch_min (di 0 derajat dan 360 derajat)
    _PM("SETUP PitchServoCalibration");
}

void TrackerPos(){ //disini define trackerlong ama trackerlat
    _PM("SETUP TrackerPos");
    //Matek GPS

  
}

void t_MavlinkHandler_callback(){ // harus define planelat planelong planeAlt disini
  _PM("TASK CALLING t_MavlinkHandler_callback");
}
  
void t_SensorHandler_callback(){ //harus define currPitch dan currYaw disini
  _PM("TASK CALLING t_SensorHandler_callback");
  // MPU6050
  mpu.update();

  currPitch = radians(mpu.getPitch()); //dari Accelerometer
  currYaw = radians(mpu.getYaw());//dari GPS(compass) + gyroscope -> idealnya pake compass tapi belum nemu library
  //Compass

  
}

void t_ServoHandler_callback(){ // dari global var define target pitch dan target yaw dan hitung error dan write ke servo
  _PM("TASK CALLING t_ServoHandler_callback");
  targetPitch = atan((planeAlt-trackerAlt)/sqrt((trackerlong-planelong)*(trackerlong-planelong) + (trackerlat-planelat)*(trackerlat-planelat)));  
  targetYaw = atan((trackerlong-planelong)/(trackerlat-planelat));

  

}
  
