// title : vision.hpp
// date : 11.17. 2021 created, 4.26.2023 updated
// author : sungryul lee

#ifndef _VISION_HPP_
#define _VISION_HPP_

#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cmath>
using namespace std;
using namespace cv;
using namespace cv::dnn;

class Line {
private:
	Rect boundingbox;
	int area;
	Point2d centroid;
	double abserror;  // distance between current center and previous center
	double angle;
	double anglediff; // difference between current angle and previous angle
	Point2d uppercentroid;
	Point2d lowercentroid;

public:
	Line();
	~Line();
	void setBoundingbox(int x,int y,int width,int height);
	void setArea(int area);
	void setCentroid(double x,double y);
	void setAbserror(double x);
	void setAngle(double x);
	void setAnglediff(double x);
	bool operator>(const Line& line);
	bool operator<(const Line& line);

	Rect getBoundingbox(void);
	int	getArea(void);
	Point2d getCentroid(void);
	double getAbserror(void);
	double getAngle(void);
	double getAnglediff(void);
	Point2d getUpperCentroid(void);
	Point2d getLowerCentroid(void);
	void printAll(void);
	int computeAngle(Mat array);
};

bool lesscompareline(Line x, Line y);
bool greatercompareline(Line x, Line y);

class Linetracer
{
private:
	Point centerold;
	Point center;
	//Line lineold;
	//Line line;
	
	int thres;
	Line min;
	int error;
	int angle;
	int angleold;
	bool detectflag;
public:
	Linetracer();
	Linetracer(Point centerold, double angleold);
	void detectLineCandidates(Mat& frame, Mat& dst);
	vector<Line> detectLineCandidates2(Mat& frame, Mat& dst);
	void detectLine(Mat& frame, Mat& dst);
	void detectLine2(Mat& frame, Mat& dst);
	int getError();
	int getAngle();
	bool getDetectflag(void);
	Point getCenterold(void);
	Point getCenter(void);
	int getAngleold(void);
	void setCenterold(Point center, Point offset);
	void setCenter(Point center);
	void setAngleold(int angle);
};


#endif //_VISION_HPP_




