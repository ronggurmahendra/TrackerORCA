/* buat konstant */

#ifndef _PARAM_h
#define _PARAM_h

#include <WiFiNINA.h>
#include "./library/mavlink/mavlink.h"
//pinout
#define PinSCL A1
#define PinSDA A2
#define PinServoYaw 7
#define PinServoPitch 5

//const
#define KpYaw 1
#define KdYaw 1
#define KiYaw 0.5

#define KpPitch 2
#define KdPitch 1
#define KiPitch 0.5

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

#define gpsSerial Serial1
uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
int32_t lat_gps_fix, lon_gps_fix;
uint16_t message_counter;
int16_t gps_add_counter;
uint8_t new_line_found;
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
mavlink_attitude_t attitude;
mavlink_global_position_int_t global;
mavlink_message_t msg;
int sysid = 1;
int compid = 255;
int type = 1;
uint8_t system_type = 0;
uint8_t autopilot_type = 12;
uint8_t system_mode = 0;
uint32_t custom_mode = 0;
uint8_t system_state = 3;

//TASK SCHEDULER PARAM
// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
// #define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding


// Debug and Test options
#define _DEBUG_
//#define _TEST_



#endif
