#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

int16_t input_value = 0; // Initialize desired RPM for two motors
int rm_r_en = 2;
int rm_l_en = 3;
int lm_r_en = 8;
int lm_l_en = 9;

// Use PWM pins for two motor drivers
int rm_r_pwm = 5;
int rm_l_pwm = 6;
int lm_r_pwm = 10;
int lm_l_pwm = 11;

void messageCb(const std_msgs::Int16& msg) {
  // Extract the desired RPM values for two motors from the message
  if (msg.data >= 0) {
    input_value = msg.data;
  } else {
    input_value = -msg.data;
  }
}

ros::Subscriber<std_msgs::Int16> s("rpm_listener_topic", messageCb);

void setup() {
  pinMode(rm_r_en, OUTPUT);
  pinMode(rm_l_en, OUTPUT);
  pinMode(lm_r_en, OUTPUT);
  pinMode(lm_l_en, OUTPUT);

  nh.initNode();
  nh.subscribe(s);
  Serial.begin(57600);

  // Add your setup code here (if any).
}

void loop() {
  digitalWrite(rm_r_en, HIGH);
  digitalWrite(rm_l_en, HIGH);
  digitalWrite(lm_r_en, HIGH);
  digitalWrite(lm_l_en, HIGH);

  if (nh.connected()) {
    if (input_value == 1) {
      moveForward();
    } else if (input_value == 2) {
      moveBackward();
    } else if (input_value == 3) {
      turnRight();
    } else if (input_value == 4) {
      turnLeft();
    } else {
      Stop();
    }

    nh.spinOnce();
  }

  // Add your loop code here (if any).
  delay(1);
}

void moveForward() {
  analogWrite(rm_r_pwm, 0);
  analogWrite(rm_l_pwm, 180);
  analogWrite(lm_r_pwm, 180);
  analogWrite(lm_l_pwm, 0);
}

void moveBackward() {
  analogWrite(rm_r_pwm, 180);
  analogWrite(rm_l_pwm, 0);
  analogWrite(lm_r_pwm, 0);
  analogWrite(lm_l_pwm, 180);
}

void turnRight() {
  analogWrite(rm_r_pwm, 0);
  analogWrite(rm_l_pwm, 180);
  analogWrite(lm_r_pwm, 120);
  analogWrite(lm_l_pwm, 0);
}

void turnLeft() {
  analogWrite(rm_r_pwm, 0);
  analogWrite(rm_l_pwm, 120);
  analogWrite(lm_r_pwm, 180);
  analogWrite(lm_l_pwm, 0);
}

void Stop() {
  analogWrite(rm_r_pwm, 0);
  analogWrite(rm_l_pwm, 0);
  analogWrite(lm_r_pwm, 0);
  analogWrite(lm_l_pwm, 0);
}
