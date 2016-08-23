//C++
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <Windows.h>
//ARDUINO
#include "tserial.h"
#include "bot_control.h"
//OPENCV
#include "opencv\cv.h"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\imgcodecs.hpp"
#include "opencv2\videoio.hpp"
#include <opencv2\video.hpp>

//NAMESPACES
using namespace std;
using namespace cv;
///////////////////////////////////////////////////////////////////////////////////////
//VARIABLES
serial Comm;
char TrafficVolumeSerial = '\0';
int TrafficVolume = 0, NoTraffic = 1, LowTraffic = 3, MediumTraffic = 5, Delay = 0;
int CameraNumber = 0;
Mat Thresh, Threshblack, Threshorange, Threshred, Threshgreen, Threshblue, Threshwhite;
Mat Frame, HSV, Contour, Original;
Mat Background, BackgroundOneChannel;
//CONSTANTS
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS = 500;
const int MIN_OBJECT_AREA = 110 * 110;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
///////////////////////////////////////////////////////////////////////////////////////
//PRELOADING CLASSES
void EditFrame(Mat Frame);
void TrackObject(Mat Thresh);
void TrafficVolumeCalc(int TraffVol);
int DelayCalc(int TraffVol);

//////////////////////////////////////////
int main(int argc, char* argv[]){
	system("Color 1A");
	Sleep(10000);
	VideoCapture cap;
	for (; CameraNumber <= 3; CameraNumber++){ // Loop initiation for 4 cameras
		// OPENCV VIDEO PROCESSING AND VALUE FOR TRAFFIC VOLUME IS CALCULATED HERE........ ;D
		
		if (CameraNumber == 0)
			cap.open("High.mp4");
		else if (CameraNumber == 1)
			cap.open("Low.mp4");
		else if (CameraNumber == 2)
			cap.open("Medium.mp4");
		else if (CameraNumber == 3)
			cap.open("None.mp4");

		if (!cap.isOpened())
			return -1;

		cap >> Original;
		EditFrame(Original);

		// TRAFFIC VOLUME FIGURE IS GIVEN TO DELAY AND CHARACTER ASSISGNING CLASS TO PROVIDE ARDUINO WITH SWITCH SIGNAL AND THE PROGRAM WITH DELAY TIME
		if (CameraNumber == 3) // THIS IS USED TO KEEP THE CAMERA NUMBER CHANING IN A INFINITE LOOP
			CameraNumber = -1;

		cout << CameraNumber << endl;
		cout << DelayCalc(TrafficVolume) - 3000 << endl;
		Sleep(DelayCalc(TrafficVolume));
	}
	return 0;
}
///////////////////////////////////
void EditFrame(Mat Frame){
	cvtColor(Frame, HSV, CV_BGR2HSV);
	GaussianBlur(HSV, HSV, Size(9, 9), 1.5, 1.5);

	Background = imread("LaneOneBackground.JPG");
	cvtColor(Background, BackgroundOneChannel, COLOR_RGB2GRAY);

	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
	////////////////////////////////////////////////////////////////////////////////////
	inRange(HSV, Scalar(0, 220, 114), Scalar(13, 255, 255), Threshorange);
	inRange(HSV, Scalar(0, 159, 23), Scalar(10, 255, 255), Threshred);
	inRange(HSV, Scalar(36, 155, 17), Scalar(99, 255, 255), Threshgreen);
	inRange(HSV, Scalar(101, 211, 24), Scalar(179, 255, 255), Threshblue);
	inRange(HSV, Scalar(26, 0, 77), Scalar(114, 155, 255), Threshwhite);
	inRange(HSV, Scalar(0, 0, 0), Scalar(10, 70, 97), Threshblack);
	////////////////////////////////////////////////////////////////////////////////////
	morphologyEx(Threshorange, Threshorange, CV_MOP_OPEN, element);
	morphologyEx(Threshred, Threshred, CV_MOP_OPEN, element);
	morphologyEx(Threshgreen, Threshgreen, CV_MOP_OPEN, element);
	morphologyEx(Threshblue, Threshblue, CV_MOP_OPEN, element);
	morphologyEx(Threshwhite, Threshwhite, CV_MOP_OPEN, element);
	morphologyEx(Threshblack, Threshblack, CV_MOP_OPEN, element);

	morphologyEx(Threshorange, Threshorange, CV_MOP_OPEN, element);
	morphologyEx(Threshred, Threshred, CV_MOP_OPEN, element);
	morphologyEx(Threshgreen, Threshgreen, CV_MOP_OPEN, element);
	morphologyEx(Threshblue, Threshblue, CV_MOP_OPEN, element);
	morphologyEx(Threshwhite, Threshwhite, CV_MOP_OPEN, element);
	morphologyEx(Threshblack, Threshblack, CV_MOP_OPEN, element);
	////////////////////////////////////////////////////////////////////////////////////
	add(Threshorange, Threshred, Thresh);
	add(Threshgreen, Thresh, Thresh);
	add(Threshblue, Thresh, Thresh);
	add(Threshwhite, Thresh, Thresh);
	add(Threshblack, Thresh, Thresh);

	multiply(Thresh, BackgroundOneChannel, Thresh);
	////////////////////////////////////////////////////////////////////////////////////
	TrackObject(Thresh);
	////////////////////////////////////////////////////////////////////////////////////
	//imshow("Thresh", Thresh);
	//imshow("Contours", Contour);
	//imshow("Original", Frame);
}
//////////////////////////////////////
void TrackObject(Mat Thresh){
	Contour = Thresh.clone();

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(Contour, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	vector<Moments> mu(contours.size());
	int detectedObj = 0, detectedObj1 = 0, detectedObj2;
	detectedObj2 = detectedObj1;

	for (int i = 0; i < contours.size(); i++){
		mu[i] = moments(contours[i], false);
		if (contourArea(contours[i], false) < 2500 && contourArea(contours[i], false) > 400){
			detectedObj = detectedObj + 1;
		}
		if (detectedObj2 < detectedObj){
			detectedObj1 = detectedObj;
		}
		TrafficVolume = round(detectedObj1 / 2);
	}
	TrafficVolumeCalc(TrafficVolume);
	DelayCalc(TrafficVolume);
}
//////////////////////////////////////////////
void TrafficVolumeCalc(int TraffVol){ // COMMUNICATION WITH THE ARDUINO BOARD IS DONE THROUGH THIS
	if (TraffVol != ' '){
		Comm.startDevice("COM3", 9600); // FOR COMS GREATER THAN COM10, THE COM SHOULD BE DEFIND AS FOLLOWS "\\\\.\\COM21"
		Comm.send_data(TrafficVolumeSerial); 
		Comm.stopDevice();
	}
}
/////////////////////////////////////
int DelayCalc(int TraffVol){ // CALCULATION OF DELAY TIME DEPENDING ON THE TRAFFIC CONDITION IN THE RESPECTIVE LANES
	if (TrafficVolume < 1)
		Delay = 3500;
	else if (TrafficVolume >= 1 && TrafficVolume < 3)
		Delay = 8000;
	else if (TrafficVolume >= 3 && TrafficVolume < 5)
		Delay = 13000;
	else if (TrafficVolume >= 5)
		Delay = 18000;

	return Delay;
}