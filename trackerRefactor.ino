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
Task t_MavlinkHandler(TASK_IMMEDIATE, TASK_FOREVER, & t_MavlinkHandler_callback, &ts, true);
Task t_SensorHandler(1000 * TASK_MILLISECOND, TASK_FOREVER, & t_SensorHandler_callback, &ts, true);
Task t_ServoYawHandler(1000 * TASK_MILLISECOND, TASK_FOREVER, & t_ServoYawHandler_callback, &ts, true);
//Task t_ServoPitchHandler(1000 * TASK_MILLISECOND, TASK_FOREVER, & t_ServoPitchHandler_callback, &ts, true);

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
void mavlinkSetup();
void TrackerPos();

void setup(){
 #ifdef _DEBUG_
 Serial.begin(57600);
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
 //compass.setCalibration(-698, 1973, -1256, 1137, -2069, 0);
 //compass.setCalibration(-438, 1726, -1265, 1137, -1248, 0);
 compass.setCalibration(-802, 2072, -1381, 858, -2162, 1227);



 compass.setSmoothing(10,true);
 mpu.boot();
 now = millis();
 
 
 //Calibration
 TrackerPos();
 Serial.println("Setup DONE ...");
}

void loop(){
  ts.execute();
  //t_MavlinkHandler_callback();
  //t_SensorHandler_callback();
  //t_ServoYawHandler_callback();
  //t_ServoPitchHandler_callback();
  //if (millis()-now>1000){
    //Mav_Request_Data();
    //now=millis();


}


void TrackerPos(){ //disini define trackerlong ama trackerlat
   ////_PM("SETUP TrackerPos");
   //Matek GPS
//   trackerlong = 1075699423; //shelter lanud
//   trackerlat = -69798433  ; //shelter lanud
  setupGPS();
  lat_gps_fix = 0;
  lon_gps_fix = 0;
  while(lat_gps_fix == 0 and lon_gps_fix == 0){
    read_gps();
    //_PM("SEARCHING GPS");
    //_PM(lat_gps_fix);
    //_PM(lon_gps_fix);
  }
  //_PM("GPS FOUND : ");
  trackerlong = lon_gps_fix;
  trackerlat = lat_gps_fix;
  //_PM(trackerlong);
  //_PM(trackerlat);
  
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
    //_PM("Connecting ");
    WiFi.begin(ssid, pass);
    delay(2000);
  }
  IPAddress ip = WiFi.localIP();
  //_PM("IP Address: ");
  //_PM(ip);
  while (!client.connected()) {
    client.connect(serverAddress, port);
    delay(2000);
  }
  if (client.available()) {
    //_PM("connected");
  }
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  client.write(buf, len);
  Mav_Request_Data();
}
void t_MavlinkHandler_callback(){ // harus define planelat planelong planeAlt disini
  //_PM("TASK CALLING t_MavlinkHandler_callback");

  //_PM("altitude (mm): ");
  //_PM(planeAlt);
   uint8_t c = client.read();
  //mavlink_message_t msg;  
  mavlink_status_t status;
  if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
    //_PM("Get messege");
    // Handle message
    switch(msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        {
          // Do nothing
          _PM("Get HB");
        }
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        {
          mavlink_msg_attitude_decode(&msg, &attitude);
          //_PM("roll: ");
          //_PM(attitude.roll);
          //_PM("Get attitude");
        }
        break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
          mavlink_msg_global_position_int_decode(&msg, &global);
          //if(global.lat != 0 and global.lon != 0){
            ////_PM("latitude: ");
            ////_PM(global.lat);
            planelat = global.lat;
           // planelat = -6.9197176;
            ////_PM("longitude: ");
            ////_PM(global.lon);
            planelong = global.lon;  
            //planelong = 107.5623504;
          //}
          //if(global.alt != 0){
            ////_PM("altitude: ");
            ////_PM(global.alt);
            planeAlt = global.relative_alt;
            planeAlt = max(0,planeAlt); 
              _PM("latitude: ");
              _PM(planelat);
              _PM("longitude: ");
              _PM(planelong);
              _PM("altitude: ");
              _PM(planeAlt);  
            _PM("global.lon");
            _PM(global.lon);
            _PM("global.lat: ");
            _PM(global.lat);
          //}
        }
      default:
        //_PP(msg.msgid);
        break;
    }
  }
}


 

 
void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int  maxStreams = 1;
  uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
  uint16_t MAVRates[maxStreams] = {0x2};
    
  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    client.write(buf, len);
  }
}


void t_SensorHandler_callback(){ //harus define currPitch dan currYaw disini
 ////_PM("TASK CALLING t_SensorHandler_callback");
 // MPU6050
  mpu.update();

  currPitch = mpu.getPitch()- 90; //dari Accelerometer
  
// if(heading > 180){
//  heading = heading - 360;
// }

  //_PM("pitch from MPU: ");
  //_PM(currPitch);

 
 //Compass

  compass.read();
  int heading = (360 - compass.getAzimuth())-90;//dari GPS(compass) + gyroscope -> idealnya pake compass tapi belum nemu library
  currYaw = heading;
  _PM("Yaw from compass: ");
  _PM(currYaw);
}

void t_ServoYawHandler_callback(){ // dari global var define target pitch dan target yaw dan hitung error dan write ke servo
   ////_PM("TASK CALLING t_ServoYawHandler_callback");

   //targetYaw = atan2((planelong - trackerlong),(planelat - trackerlat)) * 180 / 3.14159265359;
   //targetYaw = 180;
   //targetYaw = atan2((planelong - trackerlong),(planelat - trackerlat)) * 180 / 3.14159265359;
   targetYaw = 90;
   ///double y=sin(planelat-trackerlat)*cos(planelong);
   //double x=cos(trackerlong)*sin(planelong)-sin(trackerlong)*cos(planelong)*cos(planelat-trackerlat);
   //targetYaw = atan2(y,x)*180 / 3.14159265359;
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
   servoOutYaw = max(min(servoOutYaw, servo_yaw_max),servo_yaw_min);
   _PM(servoOutYaw);
   servo_yaw.write(servoOutYaw);

}

void t_ServoPitchHandler_callback(){
   ////_PM ("TASK CALLING t_ServoPITCHHandler_callback");
   //targetPitch = atan((planeAlt-trackerAlt)/sqrt((trackerlong-planelong)*(trackerlong-planelong) + (trackerlat-planelat)*(trackerlat-planelat))); //masi salah yang jarak trackder ama plane 
   double dist = sqrt(pow((planelat - trackerlat),2) + pow((planelong - trackerlong),2));
   
   double konstant =  11.0489137259;
  
   
   //_PM("Dist in mm: ");
   //_PM(dist*konstant);
   
   targetPitch = atan2((planeAlt-trackerAlt),dist*konstant) * 180 / 3.14159265359;
   
   
   //targetPitch = atan2((planeAlt-trackerAlt),dist*konstant) * 180 / 3.14159265359;
   //double d = 1000*6371*acos(sin(trackerlong)*sin(planelong)+cos(trackerlong)*cos(planelong)*cos(planelat-trackerlat));
   //targetPitch = atan2(planeAlt*1000-trackerAlt*1000,d);
   //_PM(d);
   //_PM("targetPitch : ");
   //_PM(targetPitch);
   double errPitch = targetPitch - currPitch;
    //_PM("errPitch : ");
   //_PM(errPitch);
   double servoOutPitch = map(errPitch*KpPitch,180 ,-180,servo_pitch_min, servo_pitch_max ); //idealnya pake PID sementara pake P dulu 
   //_PM("servoOutPitch : ");
   servoOutPitch = max(min(servoOutPitch, servo_pitch_max),servo_pitch_min);
   //_PM(servoOutPitch);
   servo_pitch.write(servoOutPitch);
   
}
