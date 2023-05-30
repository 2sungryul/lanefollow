// title : vision.cpp
// date : 11.17. 2021 created, 4.26.2023 updated
// author : sungryul lee

#include "vision.hpp"

// Line class implementation
Line::Line()
{
	boundingbox.x = 0;
	boundingbox.y = 0;
	boundingbox.width = 0;
	boundingbox.height = 0;
	area = 0;
	centroid.x = 0.0;
	centroid.y = 0.0;
	abserror = 0.0;
	angle = 0.0;
	anglediff = 0.0;
}
Line::~Line() {}
void Line::setBoundingbox(int x, int y, int width, int height)
{
	boundingbox.x = x;
	boundingbox.y = y;
	boundingbox.width = width;
	boundingbox.height = height;
}
void Line::setArea(int area)
{
	this->area = area;
}
void Line::setCentroid(double x, double y)
{
	centroid.x = x;
	centroid.y = y;
}
void Line::setAbserror(double abserror)
{
	this->abserror = abserror;
}
void Line::setAngle(double angle)
{
	this->angle = angle;
}
void Line::setAnglediff(double anglediff)
{
	this->anglediff = anglediff;
}
bool Line::operator>(const Line& line)
{
	return (this->abserror > line.abserror);
}
bool Line::operator<(const Line& line)
{
	return (this->abserror < line.abserror);
}
void Line::printAll(void)
{
	cout << boundingbox << "," << area << "," << centroid << "," << abserror << "," << angle << endl;

}
int Line::computeAngle(Mat array)
{
	if (boundingbox.width == 0 || boundingbox.height == 0)
	{
		cout << "boundingbox empty" << endl;
		return -1;
	}
	Mat tmp = array(boundingbox);
	Moments upper = moments(tmp(Rect(0, 0, tmp.cols, tmp.rows / 4)), true);
	Moments lower = moments(tmp(Rect(0, tmp.rows*3/4, tmp.cols, tmp.rows / 4)), true);
	double uppery = upper.m01 / upper.m00 + boundingbox.y;
	double upperx = upper.m10 / upper.m00 + boundingbox.x;
	double lowery = lower.m01 / lower.m00 + boundingbox.y+ tmp.rows * 3 / 4;
	double lowerx = lower.m10 / lower.m00 + boundingbox.x;
	uppercentroid = Point2d(upperx, uppery);
	lowercentroid = Point2d(lowerx, lowery);
	//cout << "upper:" << uppercentroid << " lower:" << lowercentroid << endl;
	Point2d delta = lowercentroid - uppercentroid;
	angle =  atan2(delta.y, delta.x)/3.14159*180 - 90;
	return 0;
}

Rect Line::getBoundingbox(void) { return boundingbox; }
int	Line::getArea(void) { return area; }
Point2d Line::getCentroid(void) { return centroid; }
double Line::getAbserror(void) { return abserror; }
double Line::getAngle(void) { return angle; }
double Line::getAnglediff(void) { return anglediff; }
Point2d Line::getUpperCentroid(void) { return uppercentroid; }
Point2d Line::getLowerCentroid(void) { return lowercentroid; }

// comparison fun for sort
bool lesscompareline(Line x, Line y)
{
	return x < y;
}
bool greatercompareline(Line x, Line y)
{
	return x > y;
}

// Linetracer class implementation
Linetracer::Linetracer()
{
	centerold = Point(-1,-1);
	angleold = 0;
	thres = 160;
	error = 0;
	angle = 0;
	detectflag = false;
}

Linetracer::Linetracer(Point centerold, double angleold)
{
	this->centerold = centerold;
	this->angleold = angleold;
	thres = 160;
	error = 0;
	angle = 0;
	detectflag = false;
}
void Linetracer::setCenterold(Point center, Point offset){ 	centerold = center + offset; }
void Linetracer::setAngleold(int angle){ angleold = angle; }
void Linetracer::setCenter(Point center) { this->center = center; }
int Linetracer::getError() { return error; }
int Linetracer::getAngle() { return angle; }
bool Linetracer::getDetectflag() { return detectflag; }
Point Linetracer::getCenterold() { return centerold; }
Point Linetracer::getCenter(void) { return center; }
int Linetracer::getAngleold() { return angleold; }

// detect every line candidates from frame and draw their bounding boxs in dst
// frame : BGR color image
// dst : binary image with bounding boxs
void Linetracer::detectLineCandidates(Mat& frame, Mat& dst)
{
	Mat gray, binary, result;
	vector<Line> lines;

	// graylevel correction and convert to binary image
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	//cout << "mean: " << mean(gray)[0] << endl;
	gray = gray + 100 - mean(gray)[0];
	//cout << "mean2: " << mean(gray)[0] << endl;
	threshold(gray, binary, thres, 255, THRESH_BINARY);
	//cout << "thres: " << thres << endl;
	//imshow("binary", binary);

	// detect line candidates
	Mat labels, stats, centroids;
	int cnt = connectedComponentsWithStats(binary, labels, stats, centroids);
	
	//cout << "detect:" << endl;
	for (int i = 1; i < cnt; i++)
	{
		Line tmp;
		tmp.setBoundingbox(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3));
		tmp.setArea(stats.at<int>(i, 4));
		tmp.setCentroid(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
		//tmp.printAll();
		lines.push_back(tmp);
	}

	// erase very small or large objects
	vector<Line>::iterator it;
	if (lines.size() > 0) {
		for (it = lines.begin(); it < lines.end();)
		{
			if (it->getArea() < 100 || it->getArea() > frame.rows*frame.cols/4) it = lines.erase(it);
			else it++;
		}
		//cout << "filering:" << endl;
		//for(it = lines.begin();it < lines.end(); it++) it->printAll();
	}
	//draw line information 
	cvtColor(binary, result, COLOR_GRAY2BGR);
	if (lines.size() > 0) {
		for (it = lines.begin(); it < lines.end(); it++)
		{
			rectangle(result, it->getBoundingbox(), Scalar(255,0,0));
			circle(result, it->getCentroid(), 2, Scalar(255,0,0), 2, LINE_AA);
		}		
	}
	cout << "cnt:"<<cnt<<",can:"<<lines.size()<<endl;
	dst = result;	
}

vector<Line> Linetracer::detectLineCandidates2(Mat& frame, Mat& dst)
{
	Mat gray, binary, result;
	vector<Line> lines;

	// graylevel correction and convert to binary image
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	//cout << "mean: " << mean(gray)[0] << endl;
	gray = gray + 100 - mean(gray)[0];
	//cout << "mean2: " << mean(gray)[0] << endl;
	threshold(gray, binary, thres, 255, THRESH_BINARY);
	//cout << "thres: " << thres << endl;
	//imshow("binary", binary);

	// detect line candidates
	Mat labels, stats, centroids;
	int cnt = connectedComponentsWithStats(binary, labels, stats, centroids);
	
	//cout << "detect:" << endl;
	for (int i = 1; i < cnt; i++)
	{
		Line tmp;
		tmp.setBoundingbox(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3));
		tmp.setArea(stats.at<int>(i, 4));
		tmp.setCentroid(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
		//tmp.printAll();
		lines.push_back(tmp);
	}

	// erase very small or large objects
	vector<Line>::iterator it;
	if (lines.size() > 0) {
		for (it = lines.begin(); it < lines.end();)
		{
			if (it->getArea() < 100 || it->getArea() > frame.rows*frame.cols/4) it = lines.erase(it);
			else it++;
		}
		//cout << "filering:" << endl;
		//for(it = lines.begin();it < lines.end(); it++) it->printAll();
	}
	//draw line information 
	cvtColor(binary, result, COLOR_GRAY2BGR);
	if (lines.size() > 0) {
		for (it = lines.begin(); it < lines.end(); it++)
		{
			rectangle(result, it->getBoundingbox(), Scalar(255,0,0));
			circle(result, it->getCentroid(), 2, Scalar(255,0,0), 2, LINE_AA);
		}		
	}
	cout << "cnt:"<<cnt<<",can:"<<lines.size()<<endl;
	dst = result;
	return lines;
}

// for line tracing
void Linetracer::detectLine(Mat& frame, Mat& dst)
{
	Mat gray, binary, result;
	vector<Line> lines;

	// at first, centerold must be set to the center of frame 
	if (centerold.x < 0 && centerold.y < 0)
	{
		//cout << "centerold: " << centerold << endl;
		centerold = Point(frame.cols / 2, frame.rows / 2);
		//cout << "centerold: " << centerold << endl;
	}

	// graylevel correction and convert to binary image
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	
	//cout << "mean1: " << mean(gray)[0] << endl;
	gray = gray + 100 - mean(gray)[0];
	//cout << "mean2: " << mean(gray)[0] << endl;
	//dst = gray;
	threshold(gray, binary, thres, 255, THRESH_BINARY);
	//cout << "thres: " << thres << endl;
	//imshow("binary", binary);

	// detect line candidates
	Mat labels, stats, centroids;
	int cnt = connectedComponentsWithStats(binary, labels, stats, centroids);
	
	//cout << "detect:" << endl;
	for (int i = 1; i < cnt; i++)
	{
		Line tmp;
		tmp.setBoundingbox(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3));
		tmp.setArea(stats.at<int>(i, 4));
		tmp.setCentroid(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
		//tmp.setAbserror(abs(centerold.x - centroids.at<double>(i, 0))); 
		tmp.setAbserror(norm(centerold - (Point)tmp.getCentroid())); 
		//tmp.setAngle(0);
		//tmp.computeAngle(binary);
		//tmp.setAnglediff(abs(angleold - tmp.getAngle()));
		//tmp.printAll();
		lines.push_back(tmp);
	}

	// erase line with very small or large area
	vector<Line>::iterator it;
	for (it = lines.begin(); it < lines.end();)
	{
		if (it->getArea() < 100 || it->getArea() > frame.rows*frame.cols/4) it = lines.erase(it);
		else it++;
	}
	//cout << "filering:" << endl;
	//for(it = lines.begin();it < lines.end(); it++) it->printAll();

	//draw line information and get the best candidate of lines
	cvtColor(binary, result, COLOR_GRAY2BGR);
	if (lines.size() > 0) {
		for (it = lines.begin(); it < lines.end(); it++)
		{
			rectangle(result, it->getBoundingbox(), Scalar(255, 0, 0));
			circle(result, it->getCentroid(), 2, Scalar(255, 0, 0), 2, LINE_AA);
			//circle(result, it->getUpperCentroid(), 2, Scalar(0, 255, 0), 2, LINE_AA);
			//circle(result, it->getLowerCentroid(), 2, Scalar(0, 255, 0), 2, LINE_AA);
		}

		//sort(lines.begin(), lines.end(), greatercompareline);
		//sort(lines.begin(), lines.end(), lesscompareline);
		sort(lines.begin(), lines.end());
		min = *(lines.begin());
		//cout << "min:" << endl;
		//min.printAll();

		if (min.getAbserror() <= frame.cols / 10)
		{
			center = min.getCentroid();	// detection succeeded
			detectflag = true;
		}
		else
		{
			center = centerold; // means that the best candidate is too far from previous line
			detectflag = false;
		}
	}
	else
	{
		center = centerold; // there is no line within roi
		detectflag = false;
	}

	/*if (detectflag)
	{
		circle(result, centerold, 2, Scalar(255, 0, 0), 2, LINE_AA);
		circle(result, center, 2, Scalar(0, 0, 255), 2, LINE_AA);
		rectangle(result, min.getBoundingbox(), Scalar(0, 0, 255));
	}
	else circle(result, center, 2, Scalar(255, 0, 0), 2, LINE_AA);
	*/
	//circle(result, centerold, 2, Scalar(255, 0, 0), 2, LINE_AA);
	circle(result, center, 2, Scalar(0, 0, 255), 2, LINE_AA);
	if (detectflag) rectangle(result, min.getBoundingbox(), Scalar(0, 0, 255));
	
	dst = result;
	//dst = binary;
	//dst = gray;
	//imshow("result", dst);
	error = center.x - binary.cols / 2;
	angle = min.getAngle();
	//cout << "error: " << error <<" angle:" << angle << endl;
	centerold = center;
	angleold = angle;
}

// for lane following
void Linetracer::detectLine2(Mat& frame, Mat& dst)
{
	Mat gray, binary, result;
	vector<Line> lines;

	// at first, centerold must be set to the center of frame 
	if (centerold.x < 0 && centerold.y < 0)
	{
		//cout << "centerold: " << centerold << endl;
		centerold = Point(frame.cols / 2, frame.rows / 2);
		//cout << "centerold: " << centerold << endl;
	}

	// graylevel correction and convert to binary image
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	//cout << "mean: " << mean(gray)[0] << endl;
	gray = gray + 100 - mean(gray)[0];
	//cout << "mean2: " << mean(gray)[0] << endl;
	threshold(gray, binary, thres, 255, THRESH_BINARY);
	//cout << "thres: " << thres << endl;
	//imshow("binary", binary);

	// detect line candidates
	Mat labels, stats, centroids;
	int cnt = connectedComponentsWithStats(binary, labels, stats, centroids);
	
	//cout << "detect:" << endl;
	for (int i = 1; i < cnt; i++)
	{
		Line tmp;
		tmp.setBoundingbox(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3));
		tmp.setArea(stats.at<int>(i, 4));
		tmp.setCentroid(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
		//tmp.setAbserror(abs(centerold.x - centroids.at<double>(i, 0))); 
		tmp.setAbserror(norm(centerold - (Point)tmp.getCentroid())); 
		//tmp.setAngle(0);
		//tmp.computeAngle(binary);
		//tmp.setAnglediff(abs(angleold - tmp.getAngle()));
		//tmp.printAll();
		lines.push_back(tmp);
	}

	// erase line with very small or large area
	vector<Line>::iterator it;
	for (it = lines.begin(); it < lines.end();)
	{
		if (it->getArea() < 100 || it->getArea() > frame.rows*frame.cols/4) it = lines.erase(it);
		else it++;
	}
	//cout << "filering:" << endl;
	//for(it = lines.begin();it < lines.end(); it++) it->printAll();

	//draw line information and get the best candidate of lines
	cvtColor(binary, result, COLOR_GRAY2BGR);
	if (lines.size() > 0) {
		for (it = lines.begin(); it < lines.end(); it++)
		{
			rectangle(result, it->getBoundingbox(), Scalar(255, 0, 0));
			circle(result, it->getCentroid(), 2, Scalar(255, 0, 0), 2, LINE_AA);
			//circle(result, it->getUpperCentroid(), 2, Scalar(0, 255, 0), 2, LINE_AA);
			//circle(result, it->getLowerCentroid(), 2, Scalar(0, 255, 0), 2, LINE_AA);
		}

		//sort(lines.begin(), lines.end(), greatercompareline);
		//sort(lines.begin(), lines.end(), lesscompareline);
		sort(lines.begin(), lines.end());
		min = *(lines.begin());
		//cout << "min:" << endl;
		//min.printAll();

		//if (min.getAbserror() <= frame.cols / 10)  // line tracing
		if (min.getAbserror() <= frame.rows) // for lane following
		{
			center = min.getCentroid();	// detection succeeded
			detectflag = true;
		}
		else
		{
			center = centerold; // means that the best candidate is too far from previous line
			detectflag = false;
		}
	}
	else
	{
		center = centerold; // there is no line within roi
		detectflag = false;
	}

	/*if (detectflag)
	{
		circle(result, centerold, 2, Scalar(255, 0, 0), 2, LINE_AA);
		circle(result, center, 2, Scalar(0, 0, 255), 2, LINE_AA);
		rectangle(result, min.getBoundingbox(), Scalar(0, 0, 255));
	}
	else circle(result, center, 2, Scalar(255, 0, 0), 2, LINE_AA);
	*/
	//circle(result, centerold, 2, Scalar(255, 0, 0), 2, LINE_AA);
	circle(result, center, 2, Scalar(0, 0, 255), 2, LINE_AA);
	if (detectflag) rectangle(result, min.getBoundingbox(), Scalar(0, 0, 255));
	
	//imshow("result", result);

	dst = result;
	//imshow("result", dst);
	error = center.x - binary.cols / 2;
	angle = min.getAngle();
	//cout << "error: " << error <<" angle:" << angle << endl;
	centerold = center;
	angleold = angle;
}
















