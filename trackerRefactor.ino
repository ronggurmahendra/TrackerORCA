
//===== Include Libraries ========================
//#include "./library/math/math.h"
#include "param.h"

//#include "./library/i2cdevlib/Arduino/MPU6050/MPU6050.h"
#include "./library/mpu/MPU.cpp"
#include <QMC5883LCompass.h>

#include <Servo.h>

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <TaskScheduler.h>
#include "./library/mavlink/mavlink.h"
#include <WiFiNINA.h>
//#include <SPI.h>



//===== Define objects ========================

MPU_6050 mpu;
Servo servo_yaw;
Servo servo_pitch;
QMC5883LCompass compass;

// ==== Scheduler ==============================
Scheduler ts;
void t_MavlinkHandler_callback();
void t_SensorHandler_callback();
void t_ServoYawHandler_callback();
void t_ServoPitchHandler_callback();
Task t_MavlinkHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_MavlinkHandler_callback, &ts, true);
Task t_SensorHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_SensorHandler_callback, &ts, true);
Task t_ServoYawHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_ServoYawHandler_callback, &ts, true);
Task t_ServoPitchHandler(100 * TASK_MILLISECOND, TASK_FOREVER, & t_ServoPitchHandler_callback, &ts, true);

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
 #ifdef _DEBUG_
 Serial.begin(9600);
 #endif
  //INISIALISASI SERVO
 servo_yaw.attach(PinServoYaw);
 servo_pitch.attach(PinServoPitch);
 //INISIALISASI MAVLINK
 mavlinkSetup();
 // INISIALISASI SENSOR  
 Wire.begin();
 compass.init();
 //compass.setCalibration(0, 1926, 0, 1118, -261, 0);
 //compass.setCalibration(-975, 1113, -1220, 850, -455, 7);
 //compass.setCalibration(-1014, 1241, -2131, 961, -1633, 985);
 compass.setCalibration(-1205, 1356, -1393, 1128, -1311, 0);

 compass.setSmoothing(10,true);
 mpu.boot();
 
 
 //Calibration
 YawServoCalibration();
 PitchServoCalibration();
 TrackerPos();
 Serial.println("Setup DONE ...");
}

void loop(){
  //ts.execute();
  t_MavlinkHandler_callback();
  t_SensorHandler_callback();
  t_ServoYawHandler_callback();
  t_ServoPitchHandler_callback();



}

void YawServoCalibration(){ // disini cari value servo_yaw_max dan servo_yaw_min (di 0 derajat dan 360 derajat) //bisa diabaikan karena servo continous
 //_PM("SETUP YawServoCalibration");
 //Matek magnetometer 
 //setelah dapet initial value compass incremen servoyawwrite sampai hasil compass muter 360 deg
 //simpan write pertama dan write terakhir di servo_yaw_max ama servo_yaw_min 
 
}
void PitchServoCalibration(){// disini cari value servo_pitch_max dan servo_pitch_min (di 0 derajat dan 360 derajat) //bisa diabaikan karena servo continous
   //_PM("SETUP PitchServoCalibration");
   // servo_pitch_min = 0; //kalau IMU bagus bisa pake ini
   // servo_pitch_max = 90; //kalau IMU bagus bisa pake ini

}

void TrackerPos(){ //disini define trackerlong ama trackerlat
   //_PM("SETUP TrackerPos");
   //Matek GPS
   trackerlong = 1075701330; //shelter lanud
   trackerlat = -69798350; //shelter lanud
}
void mavlinkSetup(){
  //Serial.begin(9600);
  while (WiFi.status() != WL_CONNECTED) {
    _PM("Connecting ");
    WiFi.begin(ssid, pass);
    delay(2000);
  }
  IPAddress ip = WiFi.localIP();
  _PM("IP Address: ");
  _PM(ip);
  while (!client.connected()) {
    client.connect(serverAddress, port);
    delay(2000);
  }
  if (client.available()) {
    _PM("connected");
  }
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  client.write(buf, len);
  Mav_Request_Data();
}
void t_MavlinkHandler_callback(){ // harus define planelat planelong planeAlt disini
  _PM("TASK CALLING t_MavlinkHandler_callback");
  _PM("latitude: ");
  _PM(planelat);
  _PM("longitude: ");
  _PM(planelong);
  _PM("altitude: ");
  _PM(planeAlt);
   uint8_t c = client.read();  
  mavlink_status_t status;
  if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
    // Handle message
    switch(msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        {
          // Do nothing
        }
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        {
          mavlink_msg_attitude_decode(&msg, &attitude);
          //_PM("roll: ");
          //_PM(attitude.roll);
          
        }
        break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
          mavlink_msg_global_position_int_decode(&msg, &global);
          //if(global.lat != 0 and global.lon != 0){
            //_PM("latitude: ");
            //_PM(global.lat);
            planelat = global.lat;
            //_PM("longitude: ");
            //_PM(global.lon);
            planelong = global.lon;  
          //}
          //if(global.alt != 0){
            //_PM("altitude: ");
            //_PM(global.alt);
            planeAlt = global.relative_alt;  
          //}
        }
      default:
        break;
    }
  }
}


 

 
void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
  const uint16_t MAVRates[maxStreams] = {0x4};
    
  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    client.write(buf, len);
  }
}


void t_SensorHandler_callback(){ //harus define currPitch dan currYaw disini
 //_PM("TASK CALLING t_SensorHandler_callback");
 // MPU6050
  mpu.update();

  currPitch = mpu.getPitch()- 90; //dari Accelerometer
  
// if(heading > 180){
//  heading = heading - 360;
// }

  _PM("pitch from MPU: ");
  _PM(currPitch);

 
 //Compass

  compass.read();
  int heading = (360 - compass.getAzimuth());//dari GPS(compass) + gyroscope -> idealnya pake compass tapi belum nemu library
  currYaw = heading;
  _PM("Yaw from compass: ");
  _PM(currYaw);
}

void t_ServoYawHandler_callback(){ // dari global var define target pitch dan target yaw dan hitung error dan write ke servo
   //_PM("TASK CALLING t_ServoYawHandler_callback");

   targetYaw = atan2((planelat - trackerlat),(planelong - trackerlong)) * 180 / 3.14159265359;
   //targetYaw = 180;
   _PM("targetYaw : ");
   _PM(targetYaw);
   
  if(targetYaw - currYaw < -180){
    targetYaw += 360;
  }else if(targetYaw - currYaw > 180){
    targetYaw -= 360;
  }else{
    targetYaw = targetYaw;
  }
   double errYaw = targetYaw - currYaw;
   _PM("errYaw : ");
   _PM(errYaw);
   double servoOutYaw = map(errYaw*KpYaw, 180, -180, servo_yaw_min, servo_yaw_max); //idealnya pake PID sementara pake P dulu 
   _PM("servoOutYaw : ");
   _PM(servoOutYaw);
   servo_yaw.write(servoOutYaw);

}

void t_ServoPitchHandler_callback(){
   //_PM ("TASK CALLING t_ServoPITCHHandler_callback");
   //targetPitch = atan((planeAlt-trackerAlt)/sqrt((trackerlong-planelong)*(trackerlong-planelong) + (trackerlat-planelat)*(trackerlat-planelat))); //masi salah yang jarak trackder ama plane 
   targetPitch = atan2((planeAlt-trackerAlt),getDistanceFromLatLonInKm (trackerlat,trackerlong,planelat,planelong) * 100000000) * 180 / 3.14159265359;
   //targetPitch = 0;
   _PM("targetPitch : ");
   _PM(targetPitch);
   double errPitch = targetPitch - currPitch;
    //_PM("errPitch : ");
   //_PM(errPitch);
   double servoOutPitch = map(errPitch*KpPitch,180 ,-180,servo_pitch_min, servo_pitch_max ); //idealnya pake PID sementara pake P dulu 
   //_PM("servoOutPitch : ");
   //_PM(servoOutPitch);
   //servo_pitch.write(servoOutPitch);
   
}

double getDistanceFromLatLonInKm (double lat1,double lon1,double lat2,double lon2) {
  double R = 6371; // Radius of the earth in km
  double dLat = deg2rad(lat2-lat1);  // deg2rad below
  double dLon = deg2rad(lon2-lon1); 
  double a = 
    sin(dLat/2) * sin(dLat/2) +
    cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * 
    sin(dLon/2) * sin(dLon/2)
    ; 
  double c = 2 * atan2(sqrt(a),sqrt(1-a)); 
  double d = R * c; // Distance in km
  return d;
}

double deg2rad(double deg) {
  return deg * (3.14159265359/180);
}
