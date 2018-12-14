#include <PWMServo.h>
#include <Wire.h>
#include <std_msgs/Bool.h>
#include <concaveteam/Spherical.h>
#include <std_srvs/Empty.h>
#include <ros.h>
#include <math.h>

ros::NodeHandle nh;
PWMServo polar_servo;
PWMServo azimuth_servo;
PWMServo left_trigger_servo;
PWMServo right_trigger_servo;

const uint8_t polar_pin = PPOLAR;
const uint8_t azim_pin = PAZIM;
const uint8_t left_trigger_pin = PLTRIG;
const uint8_t right_trigger_pin = PRTRIG;
bool in_range = false;

uint16_t last_shot;
int last_polar;
int last_azimuth;


/**
 *  Subscribe to the aim topic.
 * Every time the spherical angles are received, point the servos.
 *
 * @param msg the message containing the spherical angles in degrees.
 */
void aimCb(const concaveteam::Spherical& msg)
{
  int polar = 180 - msg.polar + 15;
  int azim = 90 + msg.azimuth + 6;

  if (polar > 110) polar = 110;


  polar_servo.write(polar);
  azimuth_servo.write(azim);

  if (abs(last_polar - msg.polar) == 0 || abs(last_azimuth - msg.azimuth) == 0)
    {
       last_shot = 0;
    }
  else
    {
      last_shot = 3000;
    }

  last_polar = msg.polar;
  last_azimuth = msg.azimuth;
}


void rangeCb(const std_msgs::Bool& msg)
{
  in_range = msg.data;
}

/**
 *  Subscribe to the trigger service.
 *  Every time the service is queried, pull the triggger.
 */
void triggerCb(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  left_trigger_servo.write(75);
  left_trigger_servo.write(160);

  delay(600);

  right_trigger_servo.write(105);
  right_trigger_servo.write(20);
}

ros::Subscriber<concaveteam::Spherical> aim_sub("point2d_to_spherical/aim", &aimCb);
ros::Subscriber<std_msgs::Bool> range_sub("/target_crit", &rangeCb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> trigger("trigger_srv", &triggerCb);

void setup()
{
  polar_servo.attach(polar_pin);
  azimuth_servo.attach(azim_pin);
  left_trigger_servo.attach(left_trigger_pin);
  right_trigger_servo.attach(right_trigger_pin);

  nh.initNode();
  nh.subscribe(aim_sub);
  nh.advertiseService(trigger);
}

void loop()
{
  // nh.spinOnce();
  // polar_servo.write(100);

  // if (last_shot > 1)
  //   {
      // left_trigger_servo.write(30);
      // right_trigger_servo.write(150);

      // delay(400);
      left_trigger_servo.write(90);
      right_trigger_servo.write(90);
      delay(400);
    
    // }

  delay(1);
}
