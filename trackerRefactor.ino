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
//   trackerlong = 1075699423; //shelter lanud
//   trackerlat = -69798433  ; //shelter lanud
  setupGPS();
  lat_gps_fix = 0;
  lon_gps_fix = 0;
  while(lat_gps_fix == 0 and lon_gps_fix == 0){
    read_gps();
    _PM("SEARCHING GPS");
    _PM(lat_gps_fix);
    _PM(lon_gps_fix);
  }
  _PM("GPS FOUND : ");
  trackerlong = lon_gps_fix;
  trackerlat = lat_gps_fix;
  _PM(trackerlong);
  _PM(trackerlat);
  
}


void setupGPS() {
  gpsSerial.begin(9600);

  // set baud rate to 57600
  uint8_t setBaudRate[28] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                             0x00, 0xE1, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xA9 };
  gpsSerial.write(setBaudRate, 28);
  delay(300);

  gpsSerial.begin(57600);
  // Set message rate to 5Hz
  uint8_t setMsgRate[14] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A };
  gpsSerial.write(setMsgRate, 14);
  delay(300);

  //  
}

void read_gps(void) {
  while (gpsSerial.available() && new_line_found == 0) {                                                   //Stay in this loop as long as there is serial information from the GPS available.
    char read_serial_byte = gpsSerial.read();                                                              //Load a new serial byte in the read_serial_byte variable.
    if (read_serial_byte == '$') {                                                                       //If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter++) {                             //Clear the old data from the incomming buffer array.
        incomming_message[message_counter] = '-';                                                        //Write a - at every position.
      }
      message_counter = 0;                                                                               //Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter++;                                                   //If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               //Write the new received byte to the new position in the incomming_message array.
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
  }

  //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             //If a new NMEA line is found.
    new_line_found = 0;                                                                                  //Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //When there is no GPS fix or latitude/longitude information available.
      number_used_sats = 0;
    }
    //If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      // Latitude (10e7)
      int32_t lat_gps;
      int32_t lat_gps_mins;
      int32_t lon_gps;
      int32_t lon_gps_mins;
      lat_gps = ((int32_t)incomming_message[17] - 48) * (int32_t)10e7;
      lat_gps += ((int32_t)incomming_message[18] - 48) * (int32_t)10e6;

      lat_gps_mins = ((int32_t)incomming_message[19] - 48) * (long)10e7;
      lat_gps_mins += ((int32_t)incomming_message[20] - 48) * (long)10e6;
      lat_gps_mins += ((int32_t)incomming_message[22] - 48) * 10e5;
      lat_gps_mins += ((int32_t)incomming_message[23] - 48) * 10e4;
      lat_gps_mins += ((int32_t)incomming_message[24] - 48) * 10e3;
      lat_gps_mins += ((int32_t)incomming_message[25] - 48) * 10e2;
      lat_gps_mins += ((int32_t)incomming_message[26] - 48) * 10e1;
      lat_gps_mins /= 60;
      lat_gps += lat_gps_mins;

      if (incomming_message[28] == 'S') lat_gps *= -1;

      lat_gps_fix = lat_gps;

      // longitude (10e7)
      lon_gps = ((int32_t)incomming_message[30] - 48) * (int32_t)10e8;
      lon_gps += ((int32_t)incomming_message[31] - 48) * (int32_t)10e7;
      lon_gps += ((int32_t)incomming_message[32] - 48) * (int32_t)10e6;

      lon_gps_mins = ((int32_t)incomming_message[33] - 48) * (long)10e7;
      lon_gps_mins += ((int32_t)incomming_message[34] - 48) * (long)10e6;
      lon_gps_mins += ((int32_t)incomming_message[36] - 48) * 10e5;
      lon_gps_mins += ((int32_t)incomming_message[37] - 48) * 10e4;
      lon_gps_mins += ((int32_t)incomming_message[38] - 48) * 10e3;
      lon_gps_mins += ((int32_t)incomming_message[39] - 48) * 10e2;
      lon_gps_mins += ((int32_t)incomming_message[40] - 48) * 10e1;
      lon_gps_mins /= 60;
      lon_gps += lon_gps_mins;
      if (incomming_message[42] == 'W') lon_gps *= -1;

      lon_gps_fix = lon_gps;

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   //Filter the number of satillites from the GGA line.
      number_used_sats += (int)incomming_message[47] - 48;                                               //Filter the number of satillites from the GGA line.
    }

    //If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A')fix_type = (int)incomming_message[9] - 48;
  }
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

   targetYaw = atan2((planelong - trackerlong),(planelat - trackerlat)) * 180 / 3.14159265359;
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
   double dist = sqrt(pow((planelat - trackerlat),2) + pow((planelong - trackerlong),2));
   
   double konstant =  11.0489137259;
  
   
   _PM("Dist in mm: ");
   _PM(dist*konstant);
   
   targetPitch = atan2((planeAlt-trackerAlt),dist*konstant) * 180 / 3.14159265359;
   _PM("targetPitch : ");
   _PM(targetPitch);
   double errPitch = targetPitch - currPitch;
    _PM("errPitch : ");
   _PM(errPitch);
   double servoOutPitch = map(errPitch*KpPitch,180 ,-180,servo_pitch_min, servo_pitch_max ); //idealnya pake PID sementara pake P dulu 
   _PM("servoOutPitch : ");
   _PM(servoOutPitch);
   servo_pitch.write(servoOutPitch);
   
}

double getDistanceFromLatLonInKm (double lat1,double lon1,double lat2,double lon2) {
  double R = 6371; // Radius of the earth in km
  double dLat = deg2rad(abs(lat2-lat1));  // deg2rad below
  double dLon = deg2rad(abs(lon2-lon1)); 
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
