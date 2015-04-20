#ifndef FINDEDGEV2
#define FINDEDGEV2

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

struct m_Obj
{
	int left_index;
	int right_index;
};

class FindEdge
{
public:
	int m_size;
	int m_min;
	int m_max;
	int m_standard;
	vector<int> m_mark;
	Mat depth;
	Mat m_coord;
	Mat m_plain;
	vector<m_Obj> m_object;
	vector<m_Obj> pre_object;
	vector<CvRect> m_rect;
	vector<CvRect> pre_rect;

	void mb_init(sensor_msgs::LaserScan& scan);
	void mb_mapping(IplImage *&img);
	void mb_cluster();
	void mb_findEdge();
	void mb_drawRec(IplImage *&img);
	void mb_run(sensor_msgs::LaserScan& scan, IplImage *&img);
};
#endif
