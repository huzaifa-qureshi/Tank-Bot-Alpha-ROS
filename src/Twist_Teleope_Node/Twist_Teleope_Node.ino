#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;
geometry_msgs::Twist msg;

float move1;
float move2;


void callback(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.angular.z;
  if (move1 > 0 && move2 == 0)
  {
    moveForward();
  }
  else if (move1 > 0 && move2 > 0 )
  {
    turnLeft();
  }
  else if (move1 > 0 && move2 < 0 )
  {
    turnRight();
  }
  else if (move1 < 0)
  {
    moveBackward();
  }
  else
  {
    Stop();
  }
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", callback);

int rm_r_en = 2;
int rm_l_en = 3;
int lm_r_en = 8;
int lm_l_en = 9;

// Use PWM pins for two motor drivers
const int rightforw = 5;
const int leftback = 10;
const int leftforw = 11;
const int rightback = 6;

void setup() {
  pinMode(rm_r_en, OUTPUT);
  pinMode(rm_l_en, OUTPUT);
  pinMode(lm_r_en, OUTPUT);
  pinMode(lm_l_en, OUTPUT);

  pinMode(leftforw, OUTPUT);
  pinMode(leftback, OUTPUT);
  pinMode(rightforw, OUTPUT);
  pinMode(rightback, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  digitalWrite(rm_r_en, HIGH);
  digitalWrite(rm_l_en, HIGH);
  digitalWrite(lm_r_en, HIGH);
  digitalWrite(lm_l_en, HIGH);
  
  nh.spinOnce();
  delay(1);
}

void moveForward() {
    digitalWrite(leftforw, HIGH);
    digitalWrite(rightforw, HIGH);
    digitalWrite(leftback, LOW);
    digitalWrite(rightback, LOW);
    delay(100);
    Stop(); 
}

void moveBackward() {
    digitalWrite(leftforw, LOW);
    digitalWrite(rightforw, LOW);
    digitalWrite(leftback, HIGH);
    digitalWrite(rightback, HIGH);
    delay(100);
    Stop();   
}

void turnRight(){
    digitalWrite(leftforw, HIGH);
    digitalWrite(rightforw, LOW);
    digitalWrite(leftback, LOW);
    digitalWrite(rightback, LOW);
    delay(100);
    Stop(); 
}

void turnLeft() {
    digitalWrite(leftforw, LOW);
    digitalWrite(rightforw, HIGH);
    digitalWrite(leftback, LOW);
    digitalWrite(rightback, LOW);
    delay(100);
    Stop(); 
}

void Stop(){
    digitalWrite(leftforw, LOW);
    digitalWrite(rightforw, LOW);
    digitalWrite(leftback, LOW);
    digitalWrite(rightback, LOW);
}
