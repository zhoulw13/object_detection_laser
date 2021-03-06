/*************************************************************************
	> File Name: object_detection_laser.cpp
	> Author: zhoulw
	> Mail: zhoulw063922@gmail.com
	> Created Time: 2015/3/25 15:29:28
 ************************************************************************/

#include<iostream>
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"find_edge.v2.h"
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <sstream>
#include <vector> 
using namespace cv;
using namespace std;

#define Z -120
#define dX -50
#define dY 50
#define F1 7.1695537050642884e+02
#define F2 7.1275153332443017e+02
#define D1 3.3311050187491378e+02
#define D2 2.8336132788893195e+02
#define LENGTH 100
#define RANGE 10

class laser2Img{
public:
	int width;
    int count,num;
	int height;
	ros::NodeHandle nh;
	ros::Subscriber laser_sub;
    //float angle_min, angle_max, angle_increment;
    //vector<float> depth;
	Mat m_coord;
    sensor_msgs::LaserScan depth;
public:
	laser2Img(int iwidth, int iheight,const ros::NodeHandle& n){
		nh = n;
		width = iwidth;
		height = iheight;
		laser_sub = nh.subscribe("/scan",1000, &laser2Img::laserReceive,this);
        count = 0;
        num = 1;
	}
	void laserReceive(const sensor_msgs::LaserScan& data){
        //depth.clear();
        depth = data;//.range;
        //angle_increment = data.angle_increment;
        //angle_min = data.angle_min;
		/*int m_size = data.ranges.size();
		float angle_min, angle_max, angle_increment;
		angle_increment = data.angle_increment;
		float radian = data.angle_min;
		
		msg = Mat::zeros(0, 3, CV_32FC1);
		Mat temp = Mat::zeros(1,3,CV_32FC1);
		m_coord = Mat::zeros(m_size, 2, CV_32FC1);
		depth = Mat::zeros(m_size, 1, CV_32FC1);
		for (int i = 0; i < m_size; i++)
		{
			float l = data.ranges[i]*1000;
			depth.at<float>(i, 0) = l;
			float x = static_cast<float>(l * cos(radian));
			float y = static_cast<float>(l * sin(radian));
			m_coord.at<float>(i, 0) = x;
			m_coord.at<float>(i, 1) = y;
			radian += angle_increment;
		}
		int X = 0;
		float x, y;
		for (int i = 0; i < m_size; i++)
		{
			m_coord.at<float>(i,0) += dX;
			X = m_coord.at<float>(i, 0);
			x = m_coord.at<float>(i, 1) * F1 / X + D1;
			y = Z * F2 / X + D2;
	
			x = width - x;
			y = height - y;
			x += dY;
            		if(x < 0 || x > width || y < 0 || y > height)
            		    continue;
			temp.at<float>(0,0) = x;
			temp.at<float>(0,1) = y;
			temp.at<float>(0,2) = data.ranges[i]*1000;
			msg.push_back(temp);
		}*/
	};
};

int main(int argc,char** argv){
	ros::init(argc,argv,"test");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	CvCapture *capture = cvCreateCameraCapture(0);
	IplImage* frame;
	frame = cvQueryFrame(capture);
	CvPoint point;
	cvNamedWindow(" ");
	laser2Img app(frame->width, frame->height, n);
    FindEdge find;
	while(ros::ok()){
		ros::spinOnce();
		frame = cvQueryFrame(capture);
		//int size = app.msg.rows;
        find.mb_run(app.depth, frame);
		/*for(int i=0;i<size;i++){
			point.x = app.msg.at<float>(i, 0);
			point.y = app.msg.at<float>(i, 1);
			cvDrawLine(frame, point, point, cvScalar(255, 0, 0), 5);
		}*/
		cvShowImage(" ", frame);
		char c = cvWaitKey(1);
		if(c == 32)
			break;
		loop_rate.sleep();
	}
}
