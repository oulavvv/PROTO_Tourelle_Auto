#include <ros.h>
#include <std_msgs/String.h>
#include <BoundingBoxes.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher boxes("boxes", &str_msg);

void setup()
{
  nh.initNode();
  nh.advertise(boxes);
}

void publishBoxes(const std_msgs::String& boxes_str)
{
  boxes.publish(&str_msg);
}

ros::Subscriber<std_msgs::String> sub("/darknet_ros/bounding_boxes", &publishBoxes);


void loop()
{  
  nh.spinOnce();
  delay(1);
}
