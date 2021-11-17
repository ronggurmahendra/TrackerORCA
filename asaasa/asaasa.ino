#include <QMC5883LCompass.h>
#include "./library/mpu/MPU.cpp"
#include <Servo.h>
#define _DEBUG_
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <TaskScheduler.h>
#include <WiFiNINA.h>
//#include <SPI.h>

#include <WiFiNINA.h>
//pinout
#define PinSCL A1
#define PinSDA A2
QMC5883LCompass compass;
#define PinServoYaw 7
#define PinServoPitch 5

//const
#define KpYaw 1
#define KdYaw 1
#define KiYaw 0.5

#define KpPitch 2
#define KdPitch 1
#define KiPitch 0.5
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
// global var
double   trackerlong; //dari GPS
double   trackerlat; //dari GPS
double   planelong; //dari Mavlink
double   planelat; //dari Mavlink

double   trackerAlt = 0; //dari GPS
double   planeAlt; // dari Mavlink

double   currPitch; //dari Accelerometer
double   currYaw; //dari GPS(compass) + gyroscope

double   targetPitch; //
double   targetYaw; //

double  servo_yaw_max = 160;
double  servo_yaw_min = 20;
double  servo_pitch_max = 160;
double  servo_pitch_min = 20;

//GPS
MPU_6050 mpu;

#define gpsSerial Serial1
uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
int32_t lat_gps_fix, lon_gps_fix;
uint16_t message_counter;
int16_t gps_add_counter;
uint8_t new_line_found;
Servo servo_yaw;
Servo servo_pitch;
uint32_t tlast_gps;


// WiFI
#define ssid "PeritusWiFi_Gnd"
#define pass "apiktenan123"
int status = WL_IDLE_STATUS;

// TCP server
#define serverAddress "11.11.11.211"
#define port 14553
WiFiClient client;

// Mavlink
int sysid = 1;
int compid = 255;
int type = 1;
uint8_t system_type = 0;
uint8_t autopilot_type = 12;
uint8_t system_mode = 0;
uint32_t custom_mode = 0;
uint8_t system_state = 3;

void setup() {
  // put your setup code here, to run once:
#ifdef _DEBUG_
 Serial.begin(9600);
 #endif
  //INISIALISASI SERVO
 servo_yaw.attach(PinServoYaw);
 servo_pitch.attach(PinServoPitch);
 //INISIALISASI MAVLINK
 //mavlinkSetup();
 // INISIALISASI SENSOR  
 Wire.begin();
 compass.init();
//compass.setCalibration(-209, 997, -378, 263, -1, 1519);
//compass.setCalibration(-427, 1771, -1137, 1118, -1257, 0);

//compass.setCalibration(-316, 0, -1, 1419, -938, 0);

 //compass.setCalibration(-975, 1113, -1220, 850, -455, 7);
 //compass.setCalibration(-1014, 1241, -2131, 961, -1633, 985);
 //compass.setCalibration(-1205, 1356, -1393, 1128, -1311, 0);
 compass.setCalibration(-802, 2072, -1381, 858, -2162, 1227);

 compass.setSmoothing(10,true);
 mpu.boot();
 
 
 //Calibration
 //PitchServoCalibration();
 //TrackerPos();
 Serial.println("Setup DONE ...");
 servo_yaw.write(20);
 //servo_pitch.write(80);
}

void loop() {
  // put your main code here, to run repeatedly:
compass.read();
  int heading = (360 - compass.getAzimuth()-90);//dari GPS(compass) + gyroscope -> idealnya pake compass tapi belum nemu library
  currYaw = heading;
  _PM("Yaw from compass: ");
  _PM(currYaw);
  
  mpu.update();
   //mpu.update();

  currPitch = mpu.getPitch()-90; //dari Accelerometer
  
// if(heading > 180){
//  heading = heading - 360;
// }

  _PM("pitch from MPU: ");
  _PM(currPitch);

//double  mpuYaw = mpu.getYaw(); 
  //_PM("Yaw from MPU: ");
  //_PM(mpuYaw);

    
}
