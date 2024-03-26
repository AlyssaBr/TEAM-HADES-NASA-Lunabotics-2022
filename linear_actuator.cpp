#include <Servo.h>

int pin = A0;          // OUT pin of ACHS-7121 Current Sensor is connected to analog pin 6 of the Arduino. 
int value = 0;
int voltage = 1023;   // default max value that can be read is 4.9mv per unit 
//byte servoPin = 7; //Pin 7 Mega to ESC 
//Servo servo; 

void setup(){
  Serial.begin(9600);
  //servo.attach(servoPin);
  //servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal
}

int signal=1; // Set signal value, which should be between 1100 and 1900

void loop(){
  int value = analogRead(pin);
  //float voltage = (value*5)/1023.0;
  //float current = (voltage - 2.5) / 0.185;
  //servo.writeMicroseconds(signal); // Send signal to ESC.
  Serial.println(value);
  delay(200); 
  if(value <=475){
   signal = 1500;
  }
}
 
