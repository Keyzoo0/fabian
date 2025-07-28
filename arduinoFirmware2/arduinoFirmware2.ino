#include "library.h"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;

void pwmCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length >= 2) {
    pwm1 = constrain(msg.data[0], -1023, 1023);
    pwm2 = constrain(msg.data[1], -1023, 1023);
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> pwm_sub("pwm", pwmCallback);

void setup() {
  Serial.begin(115200);
  setupMotor();
  nh.initNode();
nh.subscribe(pwm_sub);


}

void loop() {
  writePwm();
  command();
  nh.spinOnce();
delay(10);

}
