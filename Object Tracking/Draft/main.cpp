
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>


using namespace cv;
using namespace std;

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;

class drawGraph1
{
public:
	float distanceFromObj(int x, int y)
	{
		float displacement = sqrt(pow(x, 2) + pow(y, 2));
		return displacement;
	}
};

void drawObject(int x, int y, Mat &frame){
	
	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(255, 0, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(255, 0, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(255, 0, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(255, 0, 0), 2);

	putText(frame, to_string(x) + "," + to_string(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	double refArea = 0;
	bool objectFound = false;

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();

		if (numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;
			}
			if (objectFound == true){
				putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				drawObject(x, y, cameraFeed);
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}

int main(int argc, char** argv)
{
	//VideoCapture cap("15105112.avi");
	VideoCapture cap(0);
	if (!cap.isOpened())  
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

		bool trackObjects = true;
		double relativeDistance = 0;
		double relativeDisplacement = 0; 

		namedWindow("Control", CV_WINDOW_AUTOSIZE); 

		int iLowH = 0;
		int iHighH = 179;

		int iLowS = 0;
		int iHighS = 255;

		int iLowV = 0;
		int iHighV = 255;

		int x = 0, y = 0;
		int a = 0, b = 0, c = 0, d = 0;

		
		cvCreateTrackbar("LowH", "Control", &iLowH, 179); 
		cvCreateTrackbar("HighH", "Control", &iHighH, 179);

		cvCreateTrackbar("LowS", "Control", &iLowS, 255); 
		cvCreateTrackbar("HighS", "Control", &iHighS, 255);

		cvCreateTrackbar("LowV", "Control", &iLowV, 255); 
		cvCreateTrackbar("HighV", "Control", &iHighV, 255);

		ofstream file1("TimeStamp.txt");
		ofstream file2("Displacement.txt");

		//waitKey(10000);
	
		while (true)
		{
			Mat imgOriginal;
			drawGraph1 drawGraph;

			if (drawGraph.distanceFromObj(x, y) == 0 && relativeDistance == 0)
			relativeDistance = (drawGraph.distanceFromObj(x, y))*0.026458333;
			relativeDisplacement = relativeDistance - (drawGraph.distanceFromObj(x, y))*0.026458333;
			double video_timestamp = cap.get(CAP_PROP_POS_MSEC);
			file1 << (video_timestamp/1000) << endl;
			file2 << (relativeDisplacement) << endl;
			
			bool bSuccess = cap.read(imgOriginal);

			if (!bSuccess) 
			{
				cout << "Cannot read a frame from video stream" << endl;
				break;
			}
			 
			Mat imgHSV;
			cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 
			Mat imgThresholded;

			inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

			if (trackObjects)
				trackFilteredObject(x, y, imgThresholded, imgOriginal);

			putText(imgOriginal, to_string(0 - relativeDisplacement), Point(200, 200), 1, 1, Scalar(0, 255, 0), 2);
			imshow("Threshold", imgThresholded); 
			imshow("Original", imgOriginal);

			if (waitKey(300) == 27) 
			{
				cout << "esc key is pressed by user" << endl;
				break;
			}
			
		}
		file1.close();
		file2.close();

	return 0;
}