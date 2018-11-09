#include <PWMServo.h>
#include <Wire.h>
#include <concaveteam/Spherical.h>
#include <concaveteam/Trigger.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ros.h>

ros::NodeHandle nh;
PWMServo polar_servo;
PWMServo azimuth_servo;
PWMServo trigger_servo;

const uint8_t polar_pin = PPOLAR;
const uint8_t azim_pin = PAZIM;
const uint8_t trigger_pin = PTRIG;


/**
  Subscribe to the aim topic.
  Every time the spherical angles are received, point the servos.
 
  @param msg the message containing the spherical angles in degrees.
 */
void aimCb(const concaveteam::Spherical& msg)
{
  polar_servo.write(msg.polar);
  azimuth_servo.write(msg.azimuth);
}

/**
   Subscribe to the trigger service.
   Every time the service is queried, pull the triggger.
*/
void triggerCb(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  trigger_servo.write(20);
  delay(10);
  trigger_servo.write(0);
}

ros::Subscriber<concaveteam::Spherical> aim_sub("point2d_to_spherical/aim", &aimCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> trigger("trigger_srv", &triggerCb);

void setup()
{
  polar_servo.attach(polar_pin);
  azimuth_servo.attach(azim_pin);
  trigger_servo.attach(trigger_pin);

  nh.initNode();
  nh.subscribe(aim_sub);
  nh.advertiseService(trigger);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
