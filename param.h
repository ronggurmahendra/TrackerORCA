/* buat konstant */

#ifndef _PARAM_h
#define _PARAM_h


//pinout
#define PinSCL A1
#define PinSDA A2
#define PinServoYaw 0
#define PinServoPitch 1

//const
#define KpYaw 2
#define KdYaw 1
#define KiYaw 0.5

#define KpPitch 2
#define KdPitch 1
#define KiPitch 0.5

// global var
double  trackerlong; //dari GPS
double  trackerlat; //dari GPS
double  planelong; //dari Mavlink
double  planelat; //dari Mavlink

double  trackerAlt; //dari GPS
double  planeAlt; // dari Mavlink

double  currPitch; //dari Accelerometer
double  currYaw; //dari GPS(compass) + gyroscope

double  targetPitch; //
double  targetYaw; //

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
