#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

// This subscribes to a topic. More code is required to fully piece
// this together, but a first step is to say "every time I get this
// msg, do this". This method is the "do this".
void aimCb(const std_msgs::Float32MultiArray& msg)
{
  // The info is in an array in msg.data. Use this to write to the servos.
}

void setup()
{
}

void loop()
{
}
