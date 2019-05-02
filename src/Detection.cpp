#include <ros/ros.h>
#include <iostream>

#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// creates Matrix src andd frame, used in the imageCallback function
cv::Mat frame,src;
//setabel funktion, takes image to print label on, the string to print, and object index as input
//The entire funktion is a placeholder for the output the TurtleBot needs to act on the sign given
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour){

	//Choosing font for the text
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	//Scale of the text
	double scale = 0.4;
	//Thickness of the text
	int thickness = 1;
	//y-coordinate of the baseline relative to the bottom most text point.
	int baseline = 0;

	//Calculates the size of the box the text is contained in
	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	//Creates a bounding box of the object need labeling
	cv::Rect br = boundingRect(contour);

	//reates a point that is the top left corner of the label bagground
	cv::Point pt(br.x + ((br.width - text.width) / 2), br.y + ((br.height + text.height) / 2));
	//Creates a white rectangle label, in the middle of the sign, to put the text on
	rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
	//Draws the text string onto the inout image
	putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

//minArr funtion. Finds the smalles number in an array, taking the araay and number of elements in the array as inputs
int minArr(double arr[], int n) {

	//A double to store the minimum value of the array
	double temp = arr[0];
	//Saving the position of the smallest value, only gets changed is a smaller one comes along
	int pos = 0;
	//for loop for checking all element in the array and saving the index of the smallest one
	for (int i = 0; i < n; i++) {
		//If an element is smaller than the one saved, update it
		if (temp > arr[i]) {
			//updates the smallest element
			temp = arr[i];
			//Saves the position of the smallet element
			pos = i;
		}
	}
	//Returns the position of the smallest element
	return pos;
}

//match funtion, takes the array containg the template images, ratio of sign and template, number of templates in the array, bounding box of object,
//an array used to find the best fitting template, the colour index and the shape index added together, the object index number, and the array containing the template indexes
//phuu... that was a long one
int match(cv::Mat templateArray[], double ITS[], int images, cv::Rect sqr1, double bestFit[], int colourshape, std::vector<cv::Point>& contour1, int sign[]) {

	//A for loop that will runs once for each template
	for (int i = 0; i < images; i++) {
		//Creating a matrix the same size as the current template
		cv::Mat Temp_1R = cv::Mat(templateArray[i].size(), CV_8U);
		//Creating a matrix the same size as the bounding box
		cv::Mat crop = cv::Mat(sqr1.size(), CV_8U);
		//Puts whatevver is inside the bounding box in the inout image into the matrix "crop"
		crop = frame(sqr1);
		//Resizes the template to the detected sign to get the best match, and puts the image in the Temp_1R matrix
		resize(templateArray[i], Temp_1R, cv::Size(crop.cols*ITS[i], crop.rows*ITS[i]));

		//Converts the template matrix to greyscale
		cvtColor(Temp_1R, Temp_1R, cv::COLOR_BGR2GRAY);
		//Converts the image to greyscale
		cvtColor(crop, crop, cv::COLOR_BGR2GRAY);
		//Does a threshold for the cropped image to make it binary
		threshold(crop, crop, 80, 255, cv::THRESH_BINARY_INV);

		//Creates the output matrix for the mathing funtion
		cv::Mat result_1;
		//Check how well the second input mathes the first input
		matchTemplate(crop, Temp_1R, result_1, cv::TM_SQDIFF);
		//Used to store the best and worst match of the single template in the given bounding box
		double minVal, maxVal;
		//Used to determin where on the image minVal and maxVal is located
		cv::Point minLoc, maxLoc;
		//Saves the values to the corrosponding int's
		minMaxLoc(result_1, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		//Saves the minVal in an array initialised in the funtion inside
		bestFit[i] = minVal;
	}
	//Calls for the funtion minArr and put the result into fitIndex
	int fitIndex = minArr(bestFit, images);
	//Returns the colour and shape value added togeter with the sign index
	return (sign[fitIndex] + colourshape);
}

//inside funtion, takes bounding box, the shape-type index, colour + shape-type, and the contour index as input
void inside(cv::Rect sqr, int shapetype, int colourshape, std::vector<cv::Point>& contour){

	//Creates an array containing the index number of all the signs recognizable
	int signIndex[] = { 111, 112, 113, 131, 132, 221 };
	//corresponding string names for each sign
	std::string signLabel[] = { "MainSideRoad", "Yield", "Kids", "DontGoLeft", "70", "Cross" };
	//Creates an int called sign, this is not used right here, but will be later on
	int sign;

	// if statement checking what kind of sign we are dealing with
	if (shapetype == 10) {

		//Array containing the paths to all the templates, it is important they are in order
		cv::Mat triangles[] = { cv::imread("/home/usr/catkin_ws/detection/src/templates/mainsideroad.png"), cv::imread("/home/usr/catkin_ws/detection/src/templates/yield.png"), cv::imread("/home/usr/catkin_ws/detection/src/templates/kids.png")};
		//Array containg the ratio between the sign and the image insige it, this is used to scale the template later
		double triITSratio[] = {0.345368, 0.516129032, 0.3577405858};
		//Index number of each template in an array
		int triSign[] = {1, 2, 3};
		//An array with as many elements as templates for the specific shape
		double bestFitTri[3] = {};

		// this int is used to count number of templates
		int triimages = 0;
		//for loop that runs as many times as the number of templates
		for (cv::Mat element : triangles) {
			//Will end up to be equal to the number of templates
			triimages++;
		}
		//Here the int sign is used and set equal to the out put of the function match, wich is also called. Inputs are explained at the funtion
		sign = match(triangles, triITSratio, triimages, sqr, bestFitTri, colourshape, contour, triSign);
	}

	// if statement checking what kind of sign we are dealing with
	if(shapetype == 20){

		//Array containing the paths to all the templates, it is important they are in order
		cv::Mat rectangles[] = { cv::imread("/home/usr/catkin_ws/detection/src/templates/WalkerTemp.png")};
		//Array containg the ratio between the sign and the image insige it, this is used to scale the template later
		double rectITSratio[] = {0.627659};
		//Index number of each template in an array
		int rectSign[] = {1};
		//An array with as many elements as templates for the specific shape
		double bestFitRect[1] = {};

		// this int is used to count number of templates
		int rectimages = 0;
		//for loop that runs as many times as the number of templates
		for (cv::Mat element : rectangles) {
			//Will end up to be equal to the number of templates
			rectimages++;
		}
		//Here the int sign is used and set equal to the out put of the function match, wich is also called. Inputs are explained at the funtion
		sign = match(rectangles, rectITSratio, rectimages, sqr, bestFitRect, colourshape, contour, rectSign);
	}

	// if statement checking what kind of sign we are dealing with
	if (shapetype == 30) {

		//Array containing the paths to all the templates, it is important they are in order
		cv::Mat circles[] = { cv::imread("/home/usr/catkin_ws/detection/src/templates/dontgoleft.png"), cv::imread("/home/usr/catkin_ws/detection/src/templates/70.png") };
		//Array containg the ratio between the sign and the image insige it, this is used to scale the template later
		double cirITSratio[] = { 0.5091743, 0.5019762 };
		//Index number of each template in an array
		int cirSign[] = { 1, 2 };
		//An array with as many elements as templates for the specific shape
		double bestFitCir[2] = {};

		// this int is used to count number of templates
		int cirimages = 0;
		//for loop that runs as many times as the number of templates
		for (cv::Mat element : circles) {
			//Will end up to be equal to the number of templates
			cirimages++;
		}
		//Here the int sign is used and set equal to the out put of the function match, wich is also called. Inputs are explained at the funtion
		sign = match(circles, cirITSratio, cirimages, sqr, bestFitCir, colourshape, contour, cirSign);
	}

	// this int is used to count number of signs detectable
	int tempimages = 0;
	//for loop that runs as many times as the number of signs that can be detected
	for (std::string element : signLabel) {
		//Will end up to be equal to the number of detectable signs
		tempimages++;
	}

	//Used to determin wich sign was detected
	int j = 0;
	//The for loop will run untill the output from match, matches the i'th element in signIndex
	for (int i = 0; i < tempimages+1; i++) {
		// j = i
		j = i;
		//Checking if sign matches the i'th element in signIndex
		if (signIndex[i] == sign) {
			//Breaks the for loop
			break;
		}
	}
	//If j is smaller tempimages (number of elements in signLabel (array containing the string names of detectable signs))
	if (j < tempimages) {
		//Calls for setLabel with the image input, the name of the detected sign, and the specific object index number as input
		//The entire funktion is a placeholder for the output the TurtleBot needs to act on the sign given
		setLabel(src, signLabel[j], contour);
	}
}

//findShapes funtion. Takes thresholded image and a colour index as input
void findShapes(cv::Mat& im, int colour) {

	//Creating morphing elements
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(19, 19));
	cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// Using previously made morphing elements to morph with
	morphologyEx(im, im, cv::MORPH_CLOSE, element1);
	morphologyEx(im, im, cv::MORPH_OPEN, element2);

	//The canny funtion is purely to get a better idea what is happening to the image when tweaking it, by showing only the outline of the detected object, can be removed
	Canny(im, im, 255, 250, 3);
	imshow("Canny", im);

	//IDFK, something somthing contours...
	std::vector<std::vector<cv::Point> > contours;
	// Finds contours... who would have guessed...
	findContours(im, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	//Approximates the vertices (corners)
	std::vector<cv::Point> approx;

	// A for loop that runs for as many objects that was detected
	for (int k = 0; k < contours.size(); k++) {

		//counts the vertices (corners) with in the specified range
		approxPolyDP(cv::Mat(contours[k]), approx, cv::arcLength(cv::Mat(contours[k]), true)*0.02, true);

		// Ask if the object has 3 vertices (corners) and the size of the object is more than er certain threshold
		if ((approx.size() == 3)&& (contourArea(contours[k]) > 1000)) {
			//Creates bounding box of the specific object
			cv::Rect bbTri = boundingRect(contours[k]);

			//Draws the bounding box on the image input   ----   At the moment in the wrong place, but correct size ---  Can be removed
			rectangle(src, cv::Point(bbTri.x, bbTri.y), cv::Point(bbTri.x + bbTri.width, bbTri.y + bbTri.height), CV_RGB(255, 0, 255), 3);

			//Cals the inside function with he bounding box, shape-type index, the shape-type index added to the colour index, and the specific contour as input
			inside(bbTri, 10, colour + 10, contours[k]);
		}

		// Ask if the object has 3 vertices (corners) and the size of the object is more than er certain threshold
		else if ((approx.size() == 4) && (contourArea(contours[k]) > 1000)) {
			//Creates bounding box of the specific object
			cv::Rect bbRect = boundingRect(contours[k]);

			//Draws the bounding box on the image input   ----   At the moment in the wrong place, but correct size ---  Can be removed
			rectangle(src, cv::Point(bbRect.x, bbRect.y), cv::Point(bbRect.x + bbRect.width, bbRect.y + bbRect.height), CV_RGB(255, 0, 255), 3);

			//Cals the inside function with he bounding box, shape-type index, the shape-type index added to the colour index, and the specific contour as input
			inside(bbRect, 20, colour + 20, contours[k]);
		}

		//If it was not a triangle or a square, then is must be a circle
		else {
			//Gets the area of the object in pixels
			double V = contourArea(contours[k]);
			//Creates bounding box of the specific object
			cv::Rect bbCir = boundingRect(contours[k]);
			//Calculates the radius of the circle
			double r = bbCir.width / 2;

			//If statement checking hhow circely the circle is. Done by checking if the width and the hight are about the same size, and also if the area would corrospond to the
			//radius given
			if (abs(1 - ((double)bbCir.width / bbCir.height)) <= 0.3 && (abs(1 - (V / (CV_PI * pow(r, 2)))) <= 0.3) && (contourArea(contours[k]) > 1000)) {

				//Draws the bounding box on the image input   ----   At the moment in the wrong place, but correct size ---  Can be removed
				rectangle(src, cv::Point(bbCir.x, bbCir.y), cv::Point(bbCir.x + bbCir.width, bbCir.y + bbCir.height), CV_RGB(255, 0, 255), 3);

				//Cals the inside function with he bounding box, shape-type index, the shape-type index added to the colour index, and the specific contour as input
				inside(bbCir, 30, colour + 30, contours[k]);
			}
		}
	}
}

// findColour funtion. Takes the ROI as input
void findColour(cv::Mat& im) {

	//Creates a matrix for each colour to look for, with the same amount of colums and rows as the inout image
	cv::Mat red = cv::Mat(im.rows, im.cols, CV_8U);
	cv::Mat blue = cv::Mat(im.rows, im.cols, CV_8U);

	/*
		When usning the inRange funtion, it takes an HSV image (Hue, Saturation, Value) input.
		The ranges of the three values are, 0-180, 0-255, 0-255, in order.
		When mesuring the values ti tweak the code, hue is normaly given from 0-360 and saturation & value is given in percentage.
		The below four int's are calculationg that percentage og 255
	*/

	int s1 = 0.60*255;
	int s2 = 0.95*255;
	int v1 = 0.36*255;
	int v2 = 0.85*255;

	/*
		And image in opencv is in BGR form, but the hue for red is ranged at the very edges of the scale.
		To make it possible to find these values in one go, the image is treated as an RGB image, thus making the red hue take the place of the blue
	*/
	// converting from RGB to HSV
	cvtColor(im, red, cv::COLOR_RGB2HSV);
	// converting from BGR to HSV
	cvtColor(im, blue, cv::COLOR_BGR2HSV);
	// Using the inRange funtion to threshold a specific colour
	inRange(red, cv::Scalar(100, s1, v1), cv::Scalar(140, s2, v2), red);
	inRange(blue, cv::Scalar(100, s1, v1), cv::Scalar(140, s2, v2), blue);
	//Executing the findShapes function with the colour thresholded image and a colour index as an input
	findShapes(red, 100);
	findShapes(blue, 200);

}

void imageCallback(const sensor_msgs::ImageConstPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  //image pros
  //importing the recieved image as a frame matrix
  src=cv_ptr->image;
  // Specifying a rectangle to be the ROI, and sets the matrix frame to be that
	frame = src(cv::Rect(src.cols - (src.cols / 3), 0, src.cols / 3, src.rows / 3));
	//Executes the findColour function, with frame as input
	findColour(frame);
	// Shows frame and src after being processed
	imshow("frame", frame);
	imshow("src", src);
	//waits x amounts of millisecond before next runthrough of the for loop
	cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
