#include "ros/ros.h"
#include "string.h"
#include "std_msgs/Float64.h"


int main(int argc, char *argv[])
{
  ros::init(argc,argv,"publisher");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/simpledog/joint1_position_controller/command",1000);
  ros::Rate loop_rate(1);

  std_msgs::Float64 msgs;

  double count = 0.5;

  while (ros::ok()) {

    msgs.data = count;

    ROS_INFO("%f", msgs.data);

    chatter_pub.publish(msgs);

    ros::spinOnce();

    loop_rate.sleep();

    count = count + 0.2;

  }

  return 0;
}

