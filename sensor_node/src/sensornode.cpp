#include "sensornode.h"

//Constructor and destructor
SensorNode::SensorNode(ros::Publisher pub,ros::Publisher frontpub, double angleC, double speed, double _min_dis, double _max_dis, int _min_degree, 
	int _max_degree, double angleC_f, double _max_dis_f, double _decelerator)
{
	angleCoef = angleC;
	robotSpeed = speed;
	pubMessage = pub;
	pubFrontMsg = frontpub;
	min_degree = _min_degree;
	max_degree = _max_degree;
	min_dis = _min_dis;
	max_dis = _max_dis;
	angleCoef_f = angleC_f;
	max_dis_f = _max_dis_f;
	decelerator = _decelerator;
	state = 0;
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

void SensorNode::stateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
        ROS_INFO("stateCallBack called");
        state = msg->data;
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
	distMinLeft_f = minDisLeft(0,min_index, msg);
	distMinRight_f = minDisRight(0,min_index, msg);

	ROS_INFO("distMinRight = %f",distMinRight);
	ROS_INFO("distMinLeft = %f",distMinLeft);
	ROS_INFO("distMinRight_f = %f",distMinRight_f);
	ROS_INFO("distMinLeft_f = %f",distMinLeft_f);
	
	geometry_msgs::Twist frontMsg;
	frontMsg.linear.x = distMinLeft_f;
	frontMsg.linear.y = distMinRight_f;
	frontMsg.angular.x = distMinLeft;
	frontMsg.angular.y = distMinRight;
	pubFrontMsg.publish(frontMsg);
        
        if (state == 1)
            distMinLeft /= angleCoef_f;
        else if (state == 2)
            distMinRight /=angleCoef_f;
        else if (state == 3)
            distMinLeft = 0.35;
        else if (state == 4)
            distMinRight = 0.35;
             
	double angle = 0;
	if (distMinLeft >= distMinRight)
	{
		angle += -angleCoef * (distMinLeft / distMinRight - 1);
	}
	else
	{
		angle += angleCoef * (distMinRight / distMinLeft - 1);
	}

	angle = min(max(-0.48,angle),0.48);
	
	ROS_INFO("angle = %f",angle);
	double v = robotSpeed;
	if (0 && min(distMinLeft_f,distMinRight_f) < min_dis)
	{
		v = 0;
		angle = 0;
	}
	else
	{
		v = robotSpeed - decelerator * abs(angle);
	}
	ROS_INFO("v = %f",v);
	publishTwist(v,angle);
}

double SensorNode::minDisRight(int startIndex, int endIndex, const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	int minIndex = endIndex;
	printf("calculating minDisRight, ");
	return minDis(startIndex, endIndex, minIndex, msg);
}

double SensorNode::minDisLeft(int startIndex, int endIndex, const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	int minIndex = size - startIndex;
	int _startIndex = size - endIndex;
	int _endIndex = size - startIndex;
	printf("calculating minDisLeft, ");
	return minDis(_startIndex, _endIndex, minIndex, msg);

}

double SensorNode::minDis(int startIndex,int endIndex,int minIndex, const sensor_msgs::LaserScan::ConstPtr& msg)
{
	printf("startIndex = %d, endIndex = %d, minIndex = %d\n",startIndex,endIndex,minIndex);
	int counter = 0;
	double dist = max_dis;
	double temp_dist = 999;
	for (int i = startIndex; i <= endIndex; i++)
	{
		if (msg->intensities[i] < 0.1)
		{
			continue;
		}
		else
		{
			printf("ranges[%d] = %f\n",i,msg->ranges[i]);
			if (msg->ranges[i] < temp_dist && msg->ranges[i] > 0.01) 
			{
				temp_dist = msg->ranges[i];
				counter++;
			}
			printf("counter = %d\n",counter);
		}
	}

	
	dist = min(temp_dist,max_dis);
	printf("dist is %f\n",dist);
	return dist;
}

double max(double a,double b)
{
	double dist = (a>b)?a:b;
	return dist;
}

double min(double a,double b)
{
	double dist = (a>b)?b:a;
	return dist;
}

double abs(double data)
{
	if (data>0)
	{
		return data;
	}
	else
	{
		return - data;
	}
}
