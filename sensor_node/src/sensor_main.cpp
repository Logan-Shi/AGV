# include "sensornode.h"

# define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
# define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
double ANGLE_P = 2; // Coeficient for angles.
double ANGLE_I = 0;
double ANGLE_D = 0;
double CUT_OFF_RATIO = 2;
double ROBOT_SPEED = 0.7; // Speed of robot [m/s].
int MAX_ANGLE_AVOID = 60;
double MAX_DIS = 1.4;
double SIDE_DIS = 0.45;
int MIN_ANGLE = 15;
int MAX_ANGLE = 75;
double DECEL = 0.1;
						   //define PUBLISHER_TOPIC "/syros/base_cmd_vel"
# define PUBLISHER_TOPIC "/cmd_vel_mux/input/laser"
						   // #define SUBSCRIBER_TOPIC "/syros/laser_laser"
# define SUBSCRIBER_TOPIC "/scan_filtered"

						   /* task: "Braitenberg vehicle 2"
						   *
						   * In main function is created Subscribing node, which transmits messages
						   * to SensorNode object. There are the messages proce ssed and commands
						   * generated.
						   */

int main(int argc, char **argv)
{

	//Initialization of node
	ros::init(argc, argv, "braitenberg2");
	ros::NodeHandle n;

	//Creating publisher
	ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
	ros::Publisher pubFrontMsg = n.advertise<geometry_msgs::Twist>("isFront", PUBLISHER_BUFFER_SIZE);

	ros::param::get("~ANGLE_P", ANGLE_P);
	ros::param::get("~ANGLE_I", ANGLE_I);
	ros::param::get("~ANGLE_D", ANGLE_D);	
	ros::param::get("~CUT_OFF_RATIO", CUT_OFF_RATIO);
	ros::param::get("~ROBOT_SPEED", ROBOT_SPEED);
	ros::param::get("~MAX_ANGLE_AVOID", MAX_ANGLE_AVOID);
	ros::param::get("~MAX_DIS", MAX_DIS);
	ros::param::get("~SIDE_DIS", SIDE_DIS);
	ros::param::get("~MIN_ANGLE", MIN_ANGLE);
	ros::param::get("~MAX_ANGLE", MAX_ANGLE);
	ros::param::get("~DECEL", DECEL);
	
	//Creating object, which stores data from sensors and has methods for
	//publishing and subscribing
	SensorNode *nodeBraitenberg2 = new SensorNode(pubMessage,pubFrontMsg,
	 ANGLE_P,ANGLE_I,ANGLE_D, ROBOT_SPEED, 
	 MAX_ANGLE_AVOID, MAX_DIS, MIN_ANGLE, 
	 MAX_ANGLE,CUT_OFF_RATIO, SIDE_DIS,DECEL);

	//Creating subscriber
	ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &SensorNode::messageCallback, nodeBraitenberg2);
	ros::Subscriber subState = n.subscribe("/assignerState", SUBSCRIBER_BUFFER_SIZE, &SensorNode::stateCallback, nodeBraitenberg2);
	ros::spin();

	return 0;
}
