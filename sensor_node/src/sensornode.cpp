#include "sensornode.h"

//Constructor and destructor
SensorNode::SensorNode(ros::Publisher pub,ros::Publisher frontpub, 
	double angleP,double angleI,double angleD, double speed, 
	int _max_degree_avoid, double _max_dis, int _min_degree, 
	int _max_degree, double _cut_off_ratio, double _side_dis, double _decelerator)
{
	KP = angleP;
	KI = angleI;
	KD = angleD;
	robotSpeed = speed;
	pubMessage = pub;
	pubFrontMsg = frontpub;
	min_degree = _min_degree;
	max_degree = _max_degree;
	max_degree_avoid = _max_degree_avoid;
	max_dis = _max_dis;
	cut_off_ratio = _cut_off_ratio;
	side_dis = _side_dis;
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
        //ROS_INFO("stateCallBack called");
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
	int max_index_avoid = size /360 * max_degree_avoid;
	
	//This cycle goes through array and finds minimum on the left and right
	distMinLeft = minDisLeft(min_index, max_index, msg);
	distMinRight = minDisRight(min_index, max_index, msg);
	distMinLeftAvoid = minDisLeft(min_index, max_index_avoid, msg);
	distMinRightAvoid = minDisRight(min_index, max_index_avoid, msg);
	distMinLeft_f = minDisLeft(0,min_index, msg);
	distMinRight_f = minDisRight(0,min_index, msg);

	ROS_INFO("distMinRight = %f",distMinRight);
	ROS_INFO("distMinLeft = %f",distMinLeft);
	ROS_INFO("distMinRightAvoid = %f",distMinRightAvoid);
	ROS_INFO("distMinLeftAvoid = %f",distMinLeftAvoid);
	ROS_INFO("distMinRight_f = %f",distMinRight_f);
	ROS_INFO("distMinLeft_f = %f",distMinLeft_f);
	
	geometry_msgs::Twist frontMsg;
	frontMsg.linear.x = distMinLeft_f;
	frontMsg.linear.y = distMinRight_f;
	frontMsg.angular.x = distMinLeft;
	frontMsg.angular.y = distMinRight;
	
        
        if (state == 1)
            distMinLeftAvoid /= cut_off_ratio;
        else if (state == 2)
            distMinRightAvoid /= cut_off_ratio;
        else if (state == 3)
            distMinLeftAvoid = side_dis;
        else if (state == 4)
            distMinRightAvoid = side_dis;
             
	double angle = 0;

	if (distMinLeftAvoid >= distMinRightAvoid)
	{
		angle = -(distMinLeftAvoid / distMinRightAvoid - 1);
	}
	else
	{
		angle = (distMinRightAvoid / distMinLeftAvoid - 1);
	}
	
	angle*= KP;

	pubFrontMsg.publish(frontMsg);

	angle = min(max(-0.48,angle),0.48);
	
	ROS_INFO("angle = %f",angle);
	double v = robotSpeed;
	if (state == 5)
		v = robotSpeed - decelerator;
	else
		v = robotSpeed;

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
			//printf("ranges[%d] = %f\n",i,msg->ranges[i]);
			if (msg->ranges[i] < temp_dist && msg->ranges[i] > 0.01) 
			{
				temp_dist = msg->ranges[i];
				counter++;
			}
			//printf("counter = %d\n",counter);
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
