#include "sensornode.h"

//Constructor and destructor
SensorNode::SensorNode(ros::Publisher pub, double angleC, double speed, double _min_dis, double _max_dis, int _min_degree, int _max_degree)
{
	angleCoef = angleC;
	robotSpeed = speed;
	pubMessage = pub;
	min_degree = _min_degree;
	max_degree = _max_degree;
	min_dis = _min_dis;
	max_dis = _max_dis;

}

SensorNode::~SensorNode()
{
}

//Publisher
void SensorNode::publishRadarMessage()
{
	//preparing message
	geometry_msgs::Twist msg;

	if (distMinLeft >= distMinRight)
	{
		msg.angular.z = -angleCoef * (distMinLeft / distMinRight - 1);
	}
	else
	{
		msg.angular.z = angleCoef * (distMinRight / distMinLeft - 1);
	}
	
	
	ROS_INFO("distMinRight = %f",distMinRight);
	ROS_INFO("distMinLeft = %f",distMinLeft);
	ROS_INFO("angle = %f",msg.angular.z);
	msg.linear.x = robotSpeed;
	//if (distMinLeft < 0.25 && distMinRight < 0.25 && angleMinLeft < 0.7 && angleMinRight < 0.7)
	//{
	//  msg.angular.z*=50;
	//  msg.linear.x*=0.5;
	//}

	//publishing message
	pubMessage.publish(msg);
}


//Subscriber
void SensorNode::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Calculation of array size from angle range and angle increment.
	ROS_INFO("scanCallBack called");
	int size = msg->ranges.size(); 
	ROS_INFO("size = %d",size);
	
	bool isStop = 0;
	
	int min_index = size / 360 * min_degree;
	int max_index = size / 360 * max_degree;
	
	//This cycle goes through array and finds minimum on the left and right
	int minIndexRight = max_index;
	int minIndexLeft = size - max_index;
	distMinLeft = max_dis;
	distMinRight = max_dis;
	for (int i = min_index; i < max_index; i++)
	{
		if (msg->ranges[i] < msg->ranges[minIndexRight] && msg->ranges[i] > min_dis) {
			minIndexRight = i;
		}
	}
	for (int i = size - max_index; i < size - min_index; i++)
	{
		if (msg->ranges[i] < msg->ranges[minIndexLeft] && msg->ranges[i] > min_dis) {
			minIndexLeft = i;
		}
	}

	// int counter=0;
	// for (int i = 0; i <360; i++)
	// {
	// 	if(i>60 && i<300)
	// 	continue;
	// 	if (msg->ranges[size*i/360] < 0.40 && msg->ranges[size*i/360] > 0.05) {
	// 		counter++;
	// 	}
	// }
	// if(counter>15)
	// isStop=1;
        ROS_INFO("minIndexLeft = %d",minIndexLeft);
        ROS_INFO("minIndexRight = %d",minIndexRight);
	//Calculation of angle from indexes and storing data to class variables.
	angleMinLeft = (size - minIndexLeft) / (double)size * 360;
	distMinLeft = (msg->ranges[minIndexLeft]>max_dis)?max_dis:msg->ranges[minIndexLeft];
	angleMinRight = minIndexRight / (double)size *360;
	distMinRight = (msg->ranges[minIndexRight]>max_dis)?max_dis:msg->ranges[minIndexRight];
	ROS_INFO("angleMinRight = %f", angleMinRight);
	ROS_INFO("angleMinLeft = %f", angleMinLeft);
	publishRadarMessage();
}
