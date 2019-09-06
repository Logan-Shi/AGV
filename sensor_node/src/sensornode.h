#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

// #include "opencv2/opencv.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/features2d/features2d.hpp"

#include <cmath>
#include <string>


/* Demonstration task: "Breitenberg Vehicle"
*
* This class controls robot. It's behavior is inspired by Breitenberg's
* vehicle. In this case robot find minimal value on the left and right
* side and goes, where the value is higher.
*/
double max(double,double);
double min(double,double);
double abs(double);
class SensorNode
{
public:

	/* Constructor:
	*
	* pub    Publisher, which can send commands to robot.
	* angleC Value, which will be stored in angleCoef.
	* speed     Value, which will be stored in robotSpeed.
	*/

	SensorNode(ros::Publisher pub,ros::Publisher frontpub, double angleC, double speed, double ,double, int, int , double, double, double);

	~SensorNode();

	/* This method reads data from sensor and processes them to variables.
	*
	* This method finds minimal distances on the left and right side
	* and saves them to variables distMinLeft, distMinRight.
	*
	* @param msg Message, which came from robot and contains data from
	* laser scan.
	*/

	void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void stateCallback(const std_msgs::UInt8::ConstPtr& msg);

private:

	/* This method publishes commands for robot.
	*
	* Commands are generated from data, which are stored in variables
	* (distMinLeft, distMinRight). Robot turns to direction, which has higher
	* value. Robot turns sharper, if higher value >> lower value.
	*/

	void publishTwist(double,double);
	double robotSpeed;        // Speed of robot [m/s].
	double angleCoef;       // Coeficient for transfering angles to speed.
	int min_degree;             // detect angle in degrees
	int max_degree;             // detect angle in degrees
	int size;
	double min_dis;            // detect distance
	double max_dis;            // detect distance
	double distMinLeft;        // Minimum distance masured by sensor on the left.
	double distMinRight;
	double distFront;

	double angleCoef_f;       // Coeficient for transfering angles to speed.
	double max_dis_f;            // detect distance
	double distMinLeft_f;        // Minimum distance masured by sensor on the left.
	double distMinRight_f;
	double decelerator;
	double minDisRight(int, int, const sensor_msgs::LaserScan::ConstPtr& msg);
	double minDisLeft(int, int, const sensor_msgs::LaserScan::ConstPtr& msg);
	double minDis(int, int, int, const sensor_msgs::LaserScan::ConstPtr& msg);
	int state;
	
	//double angleMinFront1;
	//double angleMinFront2;      // Angle, at which was measured the shortest distance on the right.
	//double distMinFront1;
	//double distMinFront2;       // Minimum distance masured by sensor on the right.



	// cv::VideoCapture cap;
	// cv::Mat frame, capped;
	// cv::Mat test_mapx, test_mapy;
	// std::vector<std::vector<double> >Pts;
	// double angle;

	// void publishCameraMessage();
	// void CameraStart();
	// bool Detector(cv::Mat frame);
	// double filter(double in, std::vector<double> &Pt, int boundary);

	ros::Publisher pubMessage; // Object for publishing messages.
	ros::Publisher pubFrontMsg;
};


