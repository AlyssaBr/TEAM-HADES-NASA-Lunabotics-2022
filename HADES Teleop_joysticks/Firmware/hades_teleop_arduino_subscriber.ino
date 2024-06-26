
// Adapted from Johan Schwind
//* AUTHOR   : Johan Schwind
//* WEBSITE  : www.johanschwind.xyz
//* EMAIL    : info@johanschwind.com

/********************************************************************************
* DESCRIPTION:
*Arduino code to control a differential drive robot via ROS 
*Twist messages using a PC and a USB/UART connection. This code
*is specific to using Cytron motor drivers but can easily be
*adapted to use any other PWM motor driver.
* 
*******************************************************************************/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "CytronMotorDriver.h"

//Configure the Cytron motor driver

CytronMD motorFL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2 // Front Left motor
CytronMD motorRL(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4  //Rear Left motors
CytronMD motorFR(PWM_DIR, 9, 8);  // PWM 1 = Pin 9, DIR 1 = Pin 8 //Front Right motors
CytronMD motorRR(PWM_DIR, 11, 10); // PWM 2 = Pin 11, DIR 2 = Pin 10 // Rear Right motors

//Define max reverse and forward speed
int cytronMin = -255;
int cytronMax = 255;

//Define speed variables
float linearVelocityX;
float angularVelocityZ;

//Configure ROS
ros::NodeHandle nh;
geometry_msgs::Twist msg;

//Create a message callback that updates the motor speeds
void messageCb(const geometry_msgs::Twist& cmd_vel)
{

    // Skid Steer Kinematics
    // Using equations from  http://robotsforroboticists.com/drive-kinematics/
    
    linearVelocityX = cmd_vel.linear.x;
    angularVelocityZ = cmd_vel.angular.z;

    // int velocity_left = (linearVelocityX - angularVelocityZ)*100;
    // int velocity_right = (linearVelocityX + angularVelocityZ)*100;
    
    int wheel_front_left = (linearVelocityX - angularVelocityZ) * 100;
    int wheel_rear_left = (linearVelocityX + angularVelocityZ) * 100; 
    int wheel_front_right = (linearVelocityX + angularVelocityZ) * 100; 
    int wheel_rear_right = (linearVelocityX - angularVelocityZ) * 100; 

    // Map wheel speeds to motors

      // motorFL.setSpeed(map(velocity_left, 100, -100, cytronMin, cytronMax));
      // motorFR.setSpeed(map(velocity_right, 100, -100, cytronMin, cytronMax));

    
    motorFL.setSpeed(map(wheel_front_left, 100, -100, cytronMin, cytronMax));
    motorRL.setSpeed(map(wheel_rear_left, 100, -100, cytronMin, cytronMax));
    motorFR.setSpeed(map(wheel_front_right, 100, -100, cytronMin, cytronMax));
    motorRR.setSpeed(map(wheel_rear_right, 100, -100, cytronMin, cytronMax));
}

//Subscribe to the Twist message
ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

//Setup the node and subscriber once
void setup()
{
    nh.initNode();
    nh.subscribe(sub);
}

//Loop forever
void loop()
{
  while (!nh.connected())
  {
    nh.spinOnce();
  }
    nh.spinOnce();
    delay(1);
}
