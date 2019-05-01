
#include <ros/ros.h>
#include </opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "std_msgs/String.h"
#include <sstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    imshow("view", cv_bridge::toCvShare(msg, enc::BGR8)->image);
    waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

Mat src, frame;

int D = 1;

void setLabel(Mat& im, const string label, vector<Point>& contour)
{

	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	Size text = getTextSize(label, fontface, scale, thickness, &baseline);
	Rect br = boundingRect(contour);

	Point pt(br.x + ((br.width - text.width) / 2), br.y + ((br.height + text.height) / 2));
	rectangle(im, pt*D + Point(0, baseline), pt*D + Point(text.width, -text.height), CV_RGB(255, 255, 255), FILLED);
	putText(im, label, pt*D, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

int minArr(double arr[], int n) {
	double temp = arr[0];
	int pos = 0;
	for (int i = 0; i < n; i++) {
		if (temp > arr[i]) {
			temp = arr[i];
			pos = i;
		}
	}
	return pos;
}

int match(Mat templateArray[], double ITS[], int images, Rect sqr1, double bestFit[], int colourshape, vector<Point>& contour1, int sign[]) {

	for (int i = 0; i < images; i++) {
		Mat Temp_1R = Mat(templateArray[i].size(), CV_8U);
		Mat crop = Mat(sqr1.size(), CV_8U);
		crop = frame(sqr1);
		resize(templateArray[i], Temp_1R, Size(crop.cols*ITS[i], crop.rows*ITS[i]));

		cvtColor(Temp_1R, Temp_1R, COLOR_BGR2GRAY);
		cvtColor(crop, crop, COLOR_BGR2GRAY);
		threshold(crop, crop, 80, 255, THRESH_BINARY_INV);



		Mat result_1;
		matchTemplate(crop, Temp_1R, result_1, TM_SQDIFF);
		double minVal, maxVal;
		Point minLoc, maxLoc, matchLoc;
		minMaxLoc(result_1, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

		bestFit[i] = minVal;
	}
	int fitIndex = minArr(bestFit, images);
	return (sign[fitIndex] + colourshape);
}


void inside(Rect sqr, int shapetype, int colourshape, vector<Point>& contour){

	int signIndex[] = { 111, 112, 113, 131, 132, 221 };
	string signLabel[] = { "MainSideRoad", "Yield", "Kids", "DontGoLeft", "70", "Cross" };
	int sign;

	if (shapetype == 10) {

		Mat triangles[] = {imread("//home//floe//Downloads//templates//mainsideroad.png"), imread("//home//floe//Downloads//templates//yield.png"), imread("//home//floe//Downloads//templates//kids.png")};
		double triITSratio[] = {0.345368, 0.516129032, 0.3577405858};
		int triSign[] = {1, 2, 3};
		double bestFitTri[3] = {};

		int triimages = 0;
		for (Mat element : triangles) {
			triimages++;
		}
		sign = match(triangles, triITSratio, triimages, sqr, bestFitTri, colourshape, contour, triSign);
	}

	if(shapetype == 20){

		Mat rectangles[] = {imread("//home//floe//Downloads//templates//WalkerTemp.png")};
		double rectITSratio[] = {0.627659};
		int rectSign[] = {1};
		double bestFitRect[1] = {};

		int rectimages = 0;
		for (Mat element : rectangles) {
			rectimages++;
		}
		sign = match(rectangles, rectITSratio, rectimages, sqr, bestFitRect, colourshape, contour, rectSign);
	}

	if (shapetype == 30) {


		Mat circles[] = {imread("//home//floe//Downloads//templates//dontgoleft.png"), imread("//home//floe//Downloads//templates//70.png") };
		double cirITSratio[] = { 0.5091743, 0.5019762 };
		int cirSign[] = { 1, 2 };
		double bestFitCir[2] = {};

		int cirimages = 0;
		for (Mat element : circles) {
			cirimages++;
		}
		sign = match(circles, cirITSratio, cirimages, sqr, bestFitCir, colourshape, contour, cirSign);
	}

	int tempimages = 0;
	for (string element : signLabel) {
		tempimages++;
	}

	int j = 0;
	for (int i = 0; i < tempimages+1; i++) {
		j = i;
		if (signIndex[i] == sign) {
			break;
		}
	}

	if (j < tempimages) {
		setLabel(src, signLabel[j], contour);
	}
}

void findShapes(Mat& im, int colour) {
	Mat element1 = getStructuringElement(MORPH_RECT, Size(19, 19));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(im, im, MORPH_CLOSE, element1);
	morphologyEx(im, im, MORPH_OPEN, element2);

	Canny(im, im, 255, 250, 3);
	imshow("kmo", im);cv2
	vector<vector<Point> > contours;
	findContours(im, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	vector<Point> approx;

	for (int k = 0; k < contours.size(); k++) {

		approxPolyDP(Mat(contours[k]), approx, arcLength(Mat(contours[k]), true)*0.02, true);

		if ((approx.size() == 3)&& (contourArea(contours[k]) > 1000)) {
			Rect bbTri = boundingRect(contours[k]);

			rectangle(src, Point(bbTri.x, bbTri.y)*D, Point(bbTri.x + bbTri.width, bbTri.y + bbTri.height)*D, CV_RGB(255, 0, 255), 3);

			inside(bbTri, 10, colour + 10, contours[k]);
		}
		else if ((approx.size() == 4) && (contourArea(contours[k]) > 1000)) {
			Rect bbRect = boundingRect(contours[k]);

			rectangle(src, Point(bbRect.x, bbRect.y)*D, Point(bbRect.x + bbRect.width, bbRect.y + bbRect.height)*D, CV_RGB(255, 0, 255), 3);

			inside(bbRect, 20, colour + 20, contours[k]);
		}
		else {
			double V = contourArea(contours[k]);
			Rect bbCir = boundingRect(contours[k]);
			double r = bbCir.width / 2;

			if (abs(1 - ((double)bbCir.width / bbCir.height)) <= 0.3 && (abs(1 - (V / (CV_PI * pow(r, 2)))) <= 0.3) && (contourArea(contours[k]) > 1000)) {

				rectangle(src, Point(bbCir.x, bbCir.y)*D, Point(bbCir.x + bbCir.width, bbCir.y + bbCir.height)*D, CV_RGB(255, 0, 255), 3);

				inside(bbCir, 30, colour + 30, contours[k]);
			}
		}
	}
}

void findColour(Mat& im) {


	Mat red = Mat(im.rows/D, im.cols/D, CV_8U);
	Mat blue = Mat(im.rows/D, im.cols/D, CV_8U);

	int k;
	int l;

	int s1 = 0.60*255;
	int s2 = 0.95*255;
	int v1 = 0.36*255;
	int v2 = 0.85*255;

	cvtColor(im, red, COLOR_RGB2HSV);
	cvtColor(im, blue, COLOR_BGR2HSV);
	inRange(red, Scalar(100, s1, v1), Scalar(140, s2, v2), red);
	inRange(blue, Scalar(100, s1, v1), Scalar(140, s2, v2), blue);
	findShapes(red, 100);
	findShapes(blue, 200);



}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  namedWindow("view");

  startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw",1,imageCallback) ;

   Mat frame;

  sub  >> frame ;




     findColour(frame);
     imshow("frame", frame);

		waitKey(1);
  ros::spin();
  cv::destroyWindow("view");
}
