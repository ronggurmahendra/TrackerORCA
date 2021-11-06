
#include "./library/mpu/MPU.cpp"

MPU_6050 mpu;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu.boot();
}

void loop() {
  // put your main code here, to run repeatedly:
    mpu.update();
   Serial.println("Pitch : ",mpu.getPitch());
   Serial.println("Yaw : ",mpu.getYaw())
}
