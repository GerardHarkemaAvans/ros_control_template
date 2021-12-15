#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

unsigned long lastTime,now;
std_msgs::Float64 setpointValue;
std_msgs::Float64 actualValue;

void setSetpointValueCallback( const std_msgs::Float64& cmd_msg){
  setpointValue.data = cmd_msg.data;
}

ros::Subscriber<std_msgs::Float64> sub("my_controller/setpoint_value", setSetpointValueCallback);
ros::Publisher pub("my_controller/actual_value", &actualValue);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  delay(100);

  actualValue.data = 0;

  lastTime = millis();
}

void loop(){

  now = millis();
  if ((now - lastTime)> 100)
  {


    double error = setpointValue.data - actualValue.data;
    if(error > 0.099999999999)
        actualValue.data = actualValue.data + 0.1;
    if(error < -0.099999999999)
        actualValue.data = actualValue.data - 0.1;

    pub.publish(&actualValue);
    lastTime=now;
  }

  nh.spinOnce();
  //delay(10);
}
