#include "sensornode.h"

//Constructor and destructor
SensorNode::SensorNode(ros::Publisher pub, double angleC, double speed, double _min_dis, double _max_dis, int _min_degree, 
	int _max_degree, double angleC_f, double _max_dis_f)
{
	angleCoef = angleC;
	robotSpeed = speed;
	pubMessage = pub;
	min_degree = _min_degree;
	max_degree = _max_degree;
	min_dis = _min_dis;
	max_dis = _max_dis;
	angleCoef_f = angleC_f;
	max_dis_f = _max_dis_f;

}

SensorNode::~SensorNode()
{
}

//Publisher
void SensorNode::publishTwist(double v, double angle)
{
	//preparing message
	geometry_msgs::Twist msg;
    msg.angular.z = angle;
    msg.linear.x = v;
	//publishing message
	pubMessage.publish(msg);
}


//Subscriber
void SensorNode::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Calculation of array size from angle range and angle increment.
	ROS_INFO("scanCallBack called");
	size = msg->ranges.size(); 
	ROS_INFO("size = %d",size);
	int min_index = size / 360 * min_degree;
	int max_index = size / 360 * max_degree;
	
	//This cycle goes through array and finds minimum on the left and right
	distMinLeft = minDisLeft(min_index, max_index, msg);
	distMinRight = minDisRight(min_index, max_index, msg);
	distMinLeft_f = minDisLeft(0,size/36, msg);
	distMinRight_f = minDisRight(0,size/36, msg);

	ROS_INFO("distMinRight = %f",distMinRight);
	ROS_INFO("distMinLeft = %f",distMinLeft);
	ROS_INFO("distMinRight_f = %f",distMinRight_f);
	ROS_INFO("distMinLeft_f = %f",distMinLeft_f);
	
	double angle = 0;
	if (distMinLeft >= distMinRight)
	{
		angle += -angleCoef * (distMinLeft / distMinRight - 1);
	}
	else
	{
		angle += angleCoef * (distMinRight / distMinLeft - 1);
	}

	if (distMinLeft_f >= distMinRight_f)
	{
		angle += -angleCoef_f * (distMinLeft_f / distMinRight_f - 1);
	}
	else
	{
		angle += angleCoef_f * (distMinRight_f / distMinLeft_f - 1);
	}
	
	ROS_INFO("angle = %f",angle);

	double v = robotSpeed;
	
	publishTwist(v,angle);
}

double SensorNode::minDisRight(int startIndex, int endIndex, const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	int minIndex = endIndex;
	double dist = max_dis;
	for (int i = startIndex; i < endIndex; i++)
	{
		if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > min_dis) {
			minIndex = i;
		}
	}
	dist = (msg->ranges[minIndex]>max_dis)?max_dis:msg->ranges[minIndex];
	return dist;
}

double SensorNode::minDisLeft(int startIndex, int endIndex, const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	int minIndex = size - startIndex;
	double dist = max_dis;
	for (int i = size - endIndex; i < size - startIndex; i++)
	{
		if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > min_dis) {
			minIndex = i;
		}
	}
	dist = (msg->ranges[minIndex]>max_dis)?max_dis:msg->ranges[minIndex];
	return dist;
}