#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

// include Arduino Servo.h library, ros.h library, and ROS Twist library for cmd_vel messages from joystick
#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include "HX711.h"

#define LOADCELL_DOUT_PIN  9
#define LOADCELL_SCK_PIN  7

HX711 scale;
float calibration_factor = -206100;

// define Sabertooth Serial Commands
const char DRIVE_ADDRESS = 128;
const char ACTUATOR_ADDRESS = 129;
const char TURN = 0b00001101;
const char DRIVE = 0b00001100;
const char BAUD_SETTING = 4;
const char STOP = 64;


//Configure ROS
ros::NodeHandle nh;
geometry_msgs::Twist msg;
std_msgs::Int8 dep_msg;


//Create a message callback that updates the motor speeds with differential drive kinematics
void messageCb(const geometry_msgs::Twist& cmd_vel)
{
    // twist message linear and angular velocities
    float linearVelocityX = cmd_vel.linear.x;
    float angularVelocityZ = cmd_vel.angular.z;
    
    // motor differiential steering kinematics
    int lin = linearVelocityX * 100; // multiplied by 100 to use int for map()
    int ang = angularVelocityZ * 100;

    // drivetrain motors mapped from twist [-1.0 +1.0]*100 to servo() [0 180] 
    // velocity_left_cmd = (linear_velocity – angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS; lookng into this
    char x = map(lin, -100, 100, 0, 127);    
    char z = map(ang, -100, 100, 0, 127);

    // write to servo motor object with mapped servo pwm value, int [0 180]
    Serial3.write(DRIVE_ADDRESS);
    Serial3.write(DRIVE);
    Serial3.write(x);
    Serial3.write((DRIVE_ADDRESS + DRIVE + x) & 127);
  
    Serial3.write(DRIVE_ADDRESS);
    Serial3.write(TURN);
    Serial3.write(z);
    Serial3.write((DRIVE_ADDRESS + TURN + z) & 127);
    
}

void moveActuators(const geometry_msgs::Twist& actuators)
{
    // twist message linear and angular velocities
    float linearVelocityX = actuators.linear.x;
    float angularVelocityZ = actuators.angular.z;
    
    // motor differiential steering kinematics
    int lin = linearVelocityX * 100; // multiplied by 100 to use int for map()
    int ang = angularVelocityZ * 100;

    // drivetrain motors mapped from twist [-1.0 +1.0]*100 to servo() [0 180] 
    // velocity_left_cmd = (linear_velocity – angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS; lookng into this
    char x = map(lin, -100, 100, 0, 127);    
    char z = map(ang, -100, 100, 0, 127);
    
    Serial3.write(ACTUATOR_ADDRESS);
    Serial3.write(0b00000110);
    Serial3.write(x);
    Serial3.write((ACTUATOR_ADDRESS + 0b00000110 + x) & 127);
  
    Serial3.write(ACTUATOR_ADDRESS);
    Serial3.write(0b00000111);
    Serial3.write(z);
    Serial3.write((ACTUATOR_ADDRESS + 0b00000111 + z) & 127);
    
//    Serial.print("Reading: ");
//    Serial.print(scale.get_units(), 1);
//    Serial.print(" lbs"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
//    Serial.print(" calibration_factor: ");
//    Serial.print(calibration_factor);
//    Serial.println();
}

void dep_control_move(const std_msgs::Int8& dep)
{
  if(dep.data == 1)
  {
    Serial3.write(ACTUATOR_ADDRESS);
    Serial3.write(0b00000111);
    Serial3.write(127);
    Serial3.write((ACTUATOR_ADDRESS + 0b00000111 + 127) & 127);
    
    delay(12000);

    Serial3.write(ACTUATOR_ADDRESS);
    Serial3.write(0b00000111);
    Serial3.write(0);
    Serial3.write((ACTUATOR_ADDRESS + 0b00000111 + 0) & 127);

    while(scale.get_units() >= -2.0) {}
    
    Serial3.write(ACTUATOR_ADDRESS);
    Serial3.write(0b00000111);
    Serial3.write(STOP);
    Serial3.write((ACTUATOR_ADDRESS + 0b00000111 + STOP) & 127);
  }
}

//Subscribe to the Twist message
ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
ros::Subscriber <geometry_msgs::Twist> actuators("/actuators", &moveActuators);
ros::Subscriber <std_msgs::Int8> dep_control("/dep_control", &dep_control_move);

//Setup the node and subscriber once
void setup()
{
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();
  
  Serial3.begin(38400);
  
//  Serial3.write(DRIVE_ADDRESS); // Used to set sabertooth baud rate
//  Serial3.write(0b00001111);
//  Serial3.write(BAUD_SETTING);
//  Serial3.write((DRIVE_ADDRESS + 0b00001111 + BAUD_SETTING) & 127);
  
  Serial3.write(DRIVE_ADDRESS);
  Serial3.write(DRIVE);
  Serial3.write(STOP);
  Serial3.write((DRIVE_ADDRESS + DRIVE + STOP) & 127);
  
  Serial3.write(DRIVE_ADDRESS);
  Serial3.write(TURN);
  Serial3.write(STOP);
  Serial3.write((DRIVE_ADDRESS + TURN + STOP) & 127);
  
  Serial3.write(ACTUATOR_ADDRESS);
  Serial3.write(0b00000110);
  Serial3.write(STOP);
  Serial3.write((ACTUATOR_ADDRESS + 0b00000110 + STOP) & 127);
  
  Serial3.write(ACTUATOR_ADDRESS);
  Serial3.write(0b00000111);
  Serial3.write(STOP);
  Serial3.write((ACTUATOR_ADDRESS + 0b00000111 + STOP) & 127);
  
  nh.initNode();       // initialize ROS node
  nh.subscribe(sub);   // initialize ROS subscriber
  nh.subscribe(actuators);   // initialize ROS subscriber
  nh.subscribe(dep_control);
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
