#include <mavlink.h>
#include <WiFiNINA.h>
#include <SPI.h>

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

 void setup() {
  Serial.begin(9600);
   while (WiFi.status() != WL_CONNECTED) {
     Serial.print("Connecting ");
     WiFi.begin(ssid, pass);
     delay(2000);
   }
   IPAddress ip = WiFi.localIP();
   Serial.print("IP Address: ");
   Serial.println(ip);
   while (!client.connected()) {
     client.connect(serverAddress, port);
     delay(2000);
   }
   if (client.available()) {
     Serial.println("connected");
   }
   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
   mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
   client.write(buf, len);
   Mav_Request_Data();
 }
 void loop() {
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
           Serial.print("roll: ");
           Serial.println(attitude.roll);
         }
         break;
       case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
         {
           mavlink_msg_global_position_int_decode(&msg, &global);
           Serial.print("latitude: ");
           Serial.println(global.lat);
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
