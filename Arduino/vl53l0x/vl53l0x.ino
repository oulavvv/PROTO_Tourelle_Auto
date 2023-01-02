#include "Adafruit_VL53L0X.h"
#include <ros.h>
#include <std_msgs/Float32.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

ros::NodeHandle  nh;

std_msgs::Float32 distance_msg;
ros::Publisher pub_dist("tof_dist", &distance_msg);


void setup() {
  //Serial.begin(115200);

  // wait until serial port opens for native USB devices
//  while (! Serial) {
//    delay(1);
//  }
  
  //Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    //Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  //Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

   nh.initNode();
   nh.advertise(pub_dist);
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  //Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    //Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    distance_msg.data = measure.RangeMilliMeter;
    pub_dist.publish(&distance_msg);
    
  } else {
    //Serial.println(" out of range ");
  }

  nh.spinOnce();
  delay(100);
}
