#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <std_msgs/Int32.h>
 
geometry_msgs::Twist movemsg;

void messageCallback(const std_msgs::String::ConstPtr& msg)
{

	switch(msg.data){
		case 0: 
			movemsg.linear.x = 1;
			movemsg.angular.z = 0;
			break;
		case 1: 
			movemsg.linear.x = 0;
			movemsg.angular.z = 1;
			break;
		case 2: 
			movemsg.linear.x = 0;
			movemsg.angular.z = -1;
			break;
		default: 
			movemsg.linear.x = 0;
			movemsg.angular.z = 0;
			break;
	}
	

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mover");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("color_topic", 10, messageCallback);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pub.publish(movemsg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
