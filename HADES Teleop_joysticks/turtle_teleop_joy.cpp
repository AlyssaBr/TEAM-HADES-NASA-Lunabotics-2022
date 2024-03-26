#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <hades_teleop_joy/remoteControl.h>

class TeleopHades
{
public:
  TeleopHades();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  int linear, angular;
  double l_scale, a_scale;
  ros::Publisher vel_pub;
  ros::Publisher execevation_control_pub;
  ros::Publisher dep_control;
  ros::Subscriber joy_sub;

};


TeleopHades::TeleopHades():
  linear(1),
  angular(2)
{

  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("scale_angular", a_scale, a_scale);
  nh.param("scale_linear", l_scale, l_scale);


  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  execevation_control_pub = nh.advertise<geometry_msgs::Twist>("actuators", 1);
  dep_control = nh.advertise<std_msgs::Int8>("dep_control", 1);


  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 5, &TeleopHades::joyCallback, this);

}

void TeleopHades::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  geometry_msgs::Twist twist_exe;
  std_msgs::Int8 dep_toggle;
  twist_exe.angular.z = joy->axes[6];
  twist_exe.angular.x = joy->axes[7];
  twist.angular.z = a_scale*joy->axes[3];
  twist.linear.x = l_scale*joy->axes[1];
  dep_toggle.data = joy->buttons[2];
  dep_control.publish(dep_toggle);
  vel_pub.publish(twist);
  execevation_control_pub.publish(twist_exe);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_hades");
  TeleopHades teleop_hades;

  ros::spin();
}
