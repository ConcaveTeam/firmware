#include <ros.h>
#include <concaveteam/Spherical.h>
#include <PWMServo.h>
#include <Wire.h>

ros::NodeHandle nh;
PWMServo polar_servo;
PWMServo azimuth_servo;

const uint8_t polar_pin = PPOLAR;
const uint8_t azim_pin = PAZIM;


/*
 * Subscribe to the aim topic.
 * Every time the spherical angles are received, point the servos.
 *
 * @param msg the message containing the spherical angles in degrees.
 */
void aimCb(const concaveteam::Spherical& msg)
{
  polar_servo.write(msg.polar);
  azimuth_servo.write(msg.azimuth);
}

ros::Subscriber<concaveteam::Spherical> aim_sub("aim", &aimCb);

void setup()
{
  pinMode(polar_pin, OUTPUT);
  pinMode(azim_pin, OUTPUT);
  polar_servo.attach(polar_pin);
  azimuth_servo.attach(azim_pin);

  nh.initNode();
  nh.subscribe(aim_sub);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
