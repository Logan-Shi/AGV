#include "sensornode.h"


#define ImgWidth 960
#define ImgHeigh 540

//Constructor and destructor
SensorNode::SensorNode(ros::Publisher pub, double angleC, double speed)
{
	angleCoef = angleC;
	robotSpeed = speed;
	pubMessage = pub;

	// cap.open(1);
	// cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	// cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	// cap >> capped;

	// cv::Matx33d test_intrinsic_matrix;
	// cv::Vec4d test_distortion_coeffs;
	// cv::Size test_image_size = frame.size();

	// FILE *test_camParam = fopen("/home/nvidia/catkin_ws/src/sensor_node/src/camParam.dat", "rb");
	
	// fread(&test_intrinsic_matrix, sizeof(cv::Matx33d), 1, test_camParam);
	// fread(&test_distortion_coeffs, sizeof(cv::Vec4d), 1, test_camParam);
	// fread(&test_image_size, sizeof(cv::Size), 1, test_camParam);
	// fclose(test_camParam);

	// test_mapx = cv::Mat(test_image_size, CV_32FC1);
	// test_mapy = cv::Mat(test_image_size, CV_32FC1);
	// cv::Mat test_R = cv::Mat::eye(3, 3, CV_32F);

	// cv::fisheye::initUndistortRectifyMap(test_intrinsic_matrix, test_distortion_coeffs, test_R, test_intrinsic_matrix, test_image_size, CV_32FC1, test_mapx, test_mapy);

	// std::vector<double> p1, p2, p3, p4;
	// Pts.push_back(p1);
	// Pts.push_back(p2);
	// Pts.push_back(p3);
	// Pts.push_back(p4);
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
		msg.angular.z = -(angleCoef*distMinLeft / distMinRight - angleCoef);
	}
	else
	{
		msg.angular.z = angleCoef*distMinRight / distMinLeft - angleCoef;
	}

	msg.linear.x = robotSpeed;
	//if (distMinLeft < 0.25 && distMinRight < 0.25 && angleMinLeft < 0.7 && angleMinRight < 0.7)
	//{
	//  msg.angular.z*=50;
	//  msg.linear.x*=0.5;
	//}

	//publishing message
	pubMessage.publish(msg);
}



// void SensorNode::publishCameraMessage()
// {
// 	//preparing message
// 	geometry_msgs::Twist msg;
// 	//if (angle<-0.6)
// 		//msg.angular.z = angle *1.2;
// 	//else if (angle<-0.4)
// 		//msg.angular.z = angle *0.98;
//       //  if (angle<0)
// 	//	msg.angular.z = angle *0.8;
// 	//else if (angle>0)
// 	msg.angular.z = angle;
// 	msg.linear.x = 0.3;
// 	pubMessage.publish(msg);
// }

// void SensorNode::CameraStart()
// {
// 	cap >> capped;
// 	if (!capped.empty())
// 	{
// 		frame = capped.clone();
// 		cv::remap(capped, frame, test_mapx, test_mapy, CV_INTER_LINEAR);
// 		resize(frame, frame, cv::Size(960, 540));
// 		if (Detector(frame))
// 			publishCameraMessage();
// 		else
// 		{
// 		geometry_msgs::Twist msg;
// 		msg.linear.x = 0;
// 		pubMessage.publish(msg);
// 		}
// 		cv::imshow("view", frame);
// 		cv::waitKey(10);
// 	}
// }

// bool SensorNode::Detector(cv::Mat frame)
// {
// 	cv::Mat grayimg, bgr, hsv, mark;
// 	frame.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
// 	cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
// 	//cv::inRange(hsv,cv::Scalar(0,144.0/255,214.0/255),cv::Scalar(160,1,1),mark);
// 	cv::inRange(hsv, cv::Scalar(255, 144.0 / 255, 214.0 / 255), cv::Scalar(360, 255.0 / 255, 255.0 / 255), mark);

// 	cv::Mat structureElement = getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15), cv::Point(-1, -1));
// 	dilate(mark, mark, structureElement, cv::Point(-1, -1));
// 	erode(mark, mark, structureElement, cv::Point(-1, -1));

// 	std::vector<cv::Vec3f> circles;
// 	cv::HoughCircles(mark, circles, CV_HOUGH_GRADIENT, 1, mark.rows / 5, 20, 8, 40, 60);
// 	if (circles.size() != 0)
// 	{
// 		return false;
// 	}
// 	for (size_t i = 0; i < circles.size(); i++)
// 	{
// 		cv::Point center(round(circles[i][0]), round(circles[i][1]));
// 		int radius = round(circles[i][2]);
// 		//circle(frame, center, 3, Scalar(0, 255, 0), -1, 4, 0);
// 		cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 3, 4, 0);
// 	}

// 	//lines
// 	cv::cvtColor(frame, grayimg, CV_RGB2GRAY);
// 	cv::threshold(grayimg, grayimg, 110, 255, cv::THRESH_BINARY);
// 	int desize = 8;
// 	//cv::imshow("threshold",grayimg);
// 	cv::Mat structureElement2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(desize, desize), cv::Point(-1, -1));
// 	dilate(grayimg, grayimg, structureElement2, cv::Point(-1, -1), 1);
// 	erode(grayimg, grayimg, structureElement2, cv::Point(-1, -1), 1);
// 	cv::Canny(grayimg, grayimg, 3, 9, 3);

// 	cv::Mat mask = cv::Mat::zeros(grayimg.size(), grayimg.type());
// 	int highv = ImgHeigh*2/ 3, lowv = ImgHeigh, thew = 100;//100 330
// 	cv::Point pts[4] = { cv::Point(0, lowv),cv::Point(thew, highv),cv::Point(ImgWidth - thew, highv),cv::Point(ImgWidth, lowv) };
// 	cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
// 	cv::bitwise_and(grayimg, mask, grayimg);
// 	cv::imshow("output", grayimg);

// 	std::vector<cv::Vec4i> lines;
// 	HoughLinesP(grayimg, lines, 1, CV_PI / 180, 20, 20, 50);

// 	std::vector<cv::Point> left_pts, right_pts;
// 	std::vector<cv::Vec4i>::const_iterator it = lines.begin();
// 	double img_center = ImgWidth / 2,the_ce=0;
// 	while (it != lines.end())
// 	{
// 		cv::Point pt1((*it)[0], (*it)[1]);
// 		cv::Point pt2((*it)[2], (*it)[3]);
// 		double slope = (static_cast<double>(pt2.y) - static_cast<double>(pt1.y)) / (static_cast<double>(pt2.x) - static_cast<double>(pt1.x) + 0.00001);
// 		if (fabs(slope) > 0.15)
// 		{
// 			if (slope > 0 && (pt1.x > img_center-the_ce || pt2.x > img_center-the_ce)) {
// 				right_pts.push_back(pt1);
// 				right_pts.push_back(pt2);
// 			}
// 			if (slope < 0 && (pt1.x < img_center+the_ce || pt2.x < img_center+the_ce)) {
// 				left_pts.push_back(pt1);
// 				left_pts.push_back(pt2);
// 			}
// 		}
// 		it++;
// 	}
// 	int ini_y = ImgHeigh * 3 / 4;
// 	int fin_y = ImgHeigh;
// 	double right_ini_x, right_fin_x, left_ini_x, left_fin_x;
// 	cv::Vec4d left_line, right_line;
// 	if (right_pts.size()<3)
// 	{
// 		right_ini_x = ImgWidth;
// 		right_fin_x = ImgWidth;
// 	}
// 	else
// 	{
// 		cv::fitLine(right_pts, right_line, CV_DIST_HUBER, 0, 0.01, 0.01);
// 		double right_m = right_line[1] / right_line[0];
// 		cv::Point right_b = cv::Point(right_line[2], right_line[3]);
// 		right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
// 		right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;
// 	}
// 	if (left_pts.size() < 3)
// 	{
// 		left_ini_x = 0;
// 		left_fin_x = 0;
//                 //cv::putText(frame, std::to_string(left_pts.size()), cv::Point(50, 150), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
// 	}
// 	else
// 	{
// 		cv::fitLine(left_pts, left_line, CV_DIST_HUBER, 0, 0.01, 0.01);
// 		double left_m = left_line[1] / left_line[0];
// 		cv::Point left_b = cv::Point(left_line[2], left_line[3]);
// 		left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
// 		left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;
// 	}

// 	right_ini_x = filter(right_ini_x, Pts[0], ImgWidth);
// 	right_fin_x = filter(right_fin_x, Pts[1], ImgWidth);
// 	left_ini_x = filter(left_ini_x, Pts[2], 0);
// 	left_fin_x = filter(left_fin_x, Pts[3], 0);

// 	double thr_vp = 0.55, thr_vp2=0.35;
// 	if(fabs(right_fin_x-ImgWidth)<5.0001)
// 	{
// 		img_center=left_fin_x+435;
// 		right_fin_x=left_fin_x+870;
// 		right_ini_x+=fabs(left_fin_x-left_ini_x);
		
// 	}
// 	else if (fabs(left_fin_x)<5.0001)
// 	{
// 	 	img_center=right_fin_x-435;
// 		left_fin_x=right_fin_x-870;
// 		left_ini_x-=fabs(right_fin_x-right_ini_x);
// 	}
// 	else img_center = right_fin_x / 2 + left_fin_x / 2;

// 	double vanish_x = fabs(img_center - left_ini_x) / fabs(img_center - right_ini_x)-1;
// 	vanish_x=(fabs(vanish_x)<0.35)?0:vanish_x;
//         angle=vanish_x-(img_center-520)/500;
// 	cv::putText(frame, std::to_string(vanish_x), cv::Point(50, 200), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
// 	cv::putText(frame, std::to_string(-(img_center-520)/500), cv::Point(50, 300), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//         cv::putText(frame, std::to_string(angle), cv::Point(50, 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
// 	std::vector<cv::Point> poly_points;
// 	cv::Mat output;
// 	frame.copyTo(output);
// 	poly_points.push_back(cv::Point(left_ini_x, ini_y));
// 	poly_points.push_back(cv::Point(right_ini_x, ini_y));
// 	poly_points.push_back(cv::Point(right_fin_x, fin_y));
// 	poly_points.push_back(cv::Point(left_fin_x, fin_y));

// 	cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
// 	cv::addWeighted(output, 0.3, frame, 1.0 - 0.3, 0, frame);
// 	cv::line(frame, poly_points[1], poly_points[2], cv::Scalar(0, 0, 255), 5, CV_AA);
// 	cv::line(frame, poly_points[0], poly_points[3], cv::Scalar(0, 255, 0), 5, CV_AA);
// 	cv::line(frame,cv::Point(img_center,ini_y), cv::Point(img_center, fin_y), cv::Scalar(255,0, 0), 5, CV_AA);
// 	cv::line(frame,cv::Point(ImgWidth/2,ini_y), cv::Point(ImgWidth/2, fin_y), cv::Scalar(0,0, 0), 5, CV_AA);
//         return true;
        
// }

// double SensorNode::filter(double in, std::vector<double> &Pt, int boundary)
// {
	// if (Pt.size() == 0)
	// {
	// 	Pt.push_back(in);
	// 	return in;
	// }
	// else
	// {
	// 	double sum = Pt[0];
	// 	int tbegin = (Pt.size() < 3) ? 1 : Pt.size() - 2;
	// 	for (size_t j = tbegin; j < Pt.size(); j++)
	// 		sum += Pt[j];
	// 	sum /= (Pt.size() - tbegin + 1);

	// 	if (fabs(sum - in) > 200 && fabs(in - boundary) > 5)
	// 	{
	// 		return  sum;
	// 	}
	// 	else
	// 	{
	// 		Pt.push_back(in);
	// 		return (in + sum ) / 2;
	// 	}

	// }
// }


//Subscriber
void SensorNode::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Calculation of array size from angle range and angle increment.
	int size = msg->ranges.size();
	int minIndexLeft = 0;
	bool isStop = 0;
	int minIndexRight = size / 2;
	//This cycle goes through array and finds minimum on the left and right
	for (int i = size/24; i<size / 6; i++)
	{
		if (msg->ranges[i] < msg->ranges[minIndexRight] && msg->ranges[i] > 0.05) {
			minIndexRight = i;
		}
	}
	for (int i = size / 6*5; i <size*23/24; i++)
	{
		if (msg->ranges[i] < msg->ranges[minIndexLeft] && msg->ranges[i] > 0.05) {
			minIndexLeft = i;
		}
	}

	int counter=0;
	for (int i = 0; i <360; i++)
	{
		if(i>60 && i<300)
		continue;
		if (msg->ranges[size*i/360] < 0.40 && msg->ranges[size*i/360] > 0.05) {
			counter++;
		}
	}
	if(counter>15)
	isStop=1;

	//Calculation of angle from indexes and storing data to class variables.
	angleMinLeft = (minIndexLeft - size / 2)*msg->angle_increment;
	distMinLeft = msg->ranges[minIndexLeft];
	angleMinRight = (minIndexRight - size / 2)*msg->angle_increment;
	distMinRight = msg->ranges[minIndexRight];
	//angleMinFront1 = (minIndexFront1 - size / 2)*msg->angle_increment;
	//angleMinFront2 = (minIndexFront2 - size / 2)*msg->angle_increment;
	//distMinFront1 = msg->ranges[minIndexFront1];
	//distMinFront2 = msg->ranges[minIndexFront2];
	if(isStop)
	{
		geometry_msgs::Twist msg;
		msg.angular.z *= 0;
		msg.linear.x *= 0;
		//publishing message
		pubMessage.publish(msg);
		// cv::waitKey(1000);
	}
	else if(distMinLeft<1&& distMinRight<1)
		publishRadarMessage();
	// else
	// 	CameraStart();
}
