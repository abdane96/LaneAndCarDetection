// main.cpp

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<conio.h>           // it may be necessary to change or remove this line if not using Windows
#include <string>
#include <memory>
#include <stdexcept>
#include "Blob.h"
#include <windows.h> 
#include <cstdlib>
#include "stdlib.h"
#include <array>
#include <time.h>
//#include "mosquittopp.h"
//#include "Mosquitto\mqtt.h"

#define SHOW_STEPS            // un-comment or comment this line to show steps or not
using namespace std;
using namespace cv;
//using namespace mosqpp;

// global variables ///////////////////////////////////////////////////////////////////////////////
const Scalar SCALAR_BLACK = Scalar(0.0, 0.0, 0.0);
const Scalar SCALAR_WHITE = Scalar(255.0, 255.0, 255.0);
const Scalar SCALAR_YELLOW = Scalar(0.0, 255.0, 255.0);
const Scalar SCALAR_GREEN = Scalar(0.0, 200.0, 0.0);
const Scalar SCALAR_RED = Scalar(0.0, 0.0, 255.0);
time_t startTime = time(NULL);
time_t waitStartTime;
boolean SNRedTrue;
boolean WERedTrue;
boolean cycle;
boolean alreadyPrinted;
boolean leftGreenEast, leftGreenSouth;
int greenTimeSN, greenTimeWE;
string RPiAddress;

// function prototypes ////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(vector<Blob> &existingBlobs, vector<Blob> &currentFrameBlobs);
void addBlobToExistingBlobs(Blob &currentFrameBlob, vector<Blob> &existingBlobs, int &intIndex);
void addNewBlob(Blob &currentFrameBlob, vector<Blob> &existingBlobs);
double distanceBetweenPoints(Point point1, Point point2);
void drawAndShowContours(Size imageSize, vector<vector<Point> > contours, string strImageName);
void drawAndShowContours(Size imageSize, vector<Blob> blobs, string strImageName);
bool checkIfBlobsCrossedTheLine(vector<Blob> &blobs, vector<vector<double>> &intersectionOfInterest, int &intHorizontalLinePosition, int &intHorizontalLinePosition2, int &carCount, int &midCount, int &midCount2, int &midCount3, int &leftCount, int &rightCount, int Cars[]);
void drawBlobInfoOnImage(vector<Blob> &blobs, Mat &imgFrame2Copy);
void drawCarCountOnImage(int &numberOfLanes, int &carCount, int &midCount, int &midCount2, int &midCount3, int &leftCount, int &rightCount, Mat &imgFrame2Copy);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
Mat whiteFilter(const Mat& src);
void TrafficLogic(int &carCountSouth, int &carCountEast, int carsSouth[], int carsEast[], int &max, int &maxTime, boolean &leftGreenSouth, boolean &leftGreenEast);
int countMaxTime(int cars[],int &max,int &maxTime, boolean &leftGreen);
//string exec(const char* cmd);

///////////////////////////////////////////////////////////////////////////////////////////////////

int main(void){//int argc, char *argv[]) {
	
	/*class mqtt_client *iot_client;
	int rc;

	char client_id[] = CLIENT_ID;
	char host[] = BROKER_ADDRESS;
	int port = MQTT_PORT;

	mosqpp::lib_init();
	if (argc > 1)
		strcpy_s(host, argv[1]);

	iot_client = new mqtt_client(client_id, host, port);

	while (1)
	{
		rc = iot_client->loop();
		if (rc)
		{
			iot_client->reconnect();
		}
		else
			iot_client->subscribe(NULL, MQTT_TOPIC);
	}

	mosqpp::lib_cleanup();*/
	
	VideoCapture capVideo, capVideo2;


	Mat imgFrame1South, imgFrame2South, imgFrame1East, imgFrame2East;

	vector<Blob> blobsSouth, blobsEast;

	Point crossingLineSouth[2], crossingLineEast[2],crossingLineSouth2[2], crossingLineEast2[2];

	int carCountSouth = 0;
	int midCountSouth = 0;
	int midCount2South = 0;
	int midCount3South = 0;
	int leftCountSouth = 0;
	int rightCountSouth = 0;
	int carCountEast = 0;
	int midCountEast = 0;
	int midCount2East = 0;
	int midCount3East = 0;
	int leftCountEast = 0;
	int rightCountEast = 0;
	int carsSouth[5] = { 0,0,0,0,0 };
	int carsEast[5] = { 0,0,0,0,0 };
	int max, maxTime = 0;
	
	greenTimeSN = 0;
	greenTimeWE = 0;
	SNRedTrue = false;
	WERedTrue = true;
	cycle = false;
	alreadyPrinted = false;
	leftGreenSouth, leftGreenEast = false;
	capVideo.open("Cars.mp4");
	//capVideo.open("rtsp://admin:Rem99h21!!@169.254.36.158/doc/page/previw.asp");
	//capVideo.open("rtsp://cam1:Rem99h21!!@192.168.0.21:88/videoMain");
	//capVideo.open("http://192.168.0.18:8080/video");
	//capVideo.open("https://pysource.com/wp-content/uploads/2018/03/road_car_view.mp4");

	capVideo2.open("CarsDrivingUnderBridge.mp4");
	//capVideo.open("CarsDrivingUnderBridge.mp4");

	//capVideo.open("Li165C-DN.mp4");

	//capVideo.open("http://166.241.180.137/mjpg/video.mjpg");

	//capVideo.open("http://172.17.142.194:8080/videofeed");

	//capVideo.open("https://firebasestorage.googleapis.com/v0/b/smarttrafficmonitoring.appspot.com/o/yo.mp4?alt=media&token=d7ec7990-267c-49ae-9404-1ddfab95723b");

	if (!capVideo.isOpened() || !capVideo2.isOpened()) {                                                 // if unable to open video file
		cout << "error reading video file" << endl << endl;      // show error message
		//_getch();                   // it may be necessary to change or remove this line if not using Windows
		return(0);                                                              // and exit program
	}

	/*if (capVideo.get(CV_CAP_PROP_FRAME_COUNT) < 2) {
		cout << "error: video file must have at least two frames";
		_getch();                   // it may be necessary to change or remove this line if not using Windows
		return(0);
	}*/
	Mat imgFrame1SouthResized,imgFrame2SouthResized, imgFrame2EastResized, imgFrame1EastResized;
	
	capVideo.read(imgFrame1South);
	capVideo.read(imgFrame2South);
	capVideo2.read(imgFrame1East);
	capVideo2.read(imgFrame2East);

	resize(imgFrame2South, imgFrame2SouthResized, Size(720, 480), 0, 0, INTER_CUBIC);// Size(imgFrame2South.cols / 2, imgFrame2South.rows / 2));//, 60, 60, INTER_NEAREST);
	resize(imgFrame1South, imgFrame1SouthResized, Size(720, 480), 0, 0, INTER_CUBIC);// Size(imgFrame2South.cols / 2, imgFrame2South.rows / 2));//, 60, 60, INTER_NEAREST);
	imgFrame2South = imgFrame2SouthResized;
	imgFrame1South = imgFrame1SouthResized;

	resize(imgFrame2East, imgFrame2EastResized, Size(720, 480), 0, 0, INTER_CUBIC);// Size(imgFrame2South.cols / 2, imgFrame2South.rows / 2));//, 60, 60, INTER_NEAREST);
	resize(imgFrame1East, imgFrame1EastResized, Size(720, 480), 0, 0, INTER_CUBIC);// Size(imgFrame2South.cols / 2, imgFrame2South.rows / 2));//, 60, 60, INTER_NEAREST);
	imgFrame2East = imgFrame2EastResized;
	imgFrame1East = imgFrame1EastResized;

	/*while (capVideo2.isOpened() && capVideo.isOpened()) {
		Mat frame, frame2;
		// Capture frame-by-frame
		capVideo >> frame;
		capVideo2 >> frame2;

		// If the frame is empty, break immediately
		if (frame.empty() || frame2.empty())
			break;

		// Display the resulting frame
		imshow("Frame", frame);
		imshow("Frame2", frame2);

		// Press  ESC on keyboard to exit
		char c = (char)waitKey(25);
		if (c == 27)
			break;
	}
	waitKey(0);*/
	/*while (capVideo.isOpened()) {
		Mat frame;
		// Capture frame-by-frame
		capVideo >> frame;

		// If the frame is empty, break immediately
		if (frame.empty())
			break;

		// Display the resulting frame
		imshow("Frame", frame);

		// Press  ESC on keyboard to exit
		char c = (char)waitKey(25);
		if (c == 27)
			break;
	}*/
	int intHorizontalLinePositionSouth = (int)round((double)imgFrame2South.rows * 0.2);
	int intHorizontalLinePositionEast = (int)round((double)imgFrame2East.rows * 0.2);
	int intHorizontalLinePositionSouth2 = (int)round((double)imgFrame2South.rows*0.8);
	int intHorizontalLinePositionEast2 = (int)round((double)imgFrame2East.rows * 0.8);

	crossingLineSouth[0].x = 0;
	crossingLineSouth[0].y = intHorizontalLinePositionSouth;

	crossingLineSouth[1].x = imgFrame2South.cols - 1;
	crossingLineSouth[1].y = intHorizontalLinePositionSouth;

	crossingLineEast[0].x = 0;
	crossingLineEast[0].y = intHorizontalLinePositionEast;

	crossingLineEast[1].x = imgFrame2East.cols - 1;
	crossingLineEast[1].y = intHorizontalLinePositionEast;


	crossingLineSouth2[0].x = 0;
	crossingLineSouth2[0].y = intHorizontalLinePositionSouth2;

	crossingLineSouth2[1].x = imgFrame2South.cols - 1;
	crossingLineSouth2[1].y = intHorizontalLinePositionSouth2;

	crossingLineEast2[0].x = 0;
	crossingLineEast2[0].y = intHorizontalLinePositionEast2;

	crossingLineEast2[1].x = imgFrame2East.cols - 1;
	crossingLineEast2[1].y = intHorizontalLinePositionEast2;

	char chCheckForEscKey = 0;

	bool blnFirstFrame = true;
	
	Mat blurredSouth, blurredEast, cannySouth, cannyEast, cvtCSouth, cvtCEast;
	Mat copySouth = imgFrame2South.clone();
	Mat copyEast = imgFrame2East.clone();
	GaussianBlur(whiteFilter(copySouth), blurredSouth, { 5, 5 }, 0);
	GaussianBlur(whiteFilter(copyEast), blurredEast, { 5, 5 }, 0);
	Canny(blurredSouth, cannySouth, 50, 200, 3);
	Canny(blurredEast, cannyEast, 50, 200, 3);
	cvtColor(cannySouth, cvtCSouth, COLOR_GRAY2BGR);
	cvtColor(cannyEast, cvtCEast, COLOR_GRAY2BGR);
	
	
	
	//waitKey(0);
	vector<Vec4i> linesSouth, linesEast;
	vector<Vec4i> linesOfInterestSouth, linesOfInterestEast;
	HoughLinesP(cannySouth, linesSouth, 0.7, CV_PI / 180, 50, 10, 200);
	HoughLinesP(cannyEast, linesEast, 0.7, CV_PI / 180, 50, 10, 200);

	//sort(linesSouth.begin(), linesSouth.end());
	
	/*namedWindow("My Window", 1);
	setMouseCallback("My Window", CallBackFunc, NULL);
	imshow("My Window", cvtCSouth);*/
	
	
	for (size_t i = 0; i < linesSouth.size(); i++)
	{
		//Filtering out unwanted lines

		Vec4i l = linesSouth[i];
		float angle = (atan2(abs(l[1] - l[3]), abs(l[0] - l[2]))) * 180 / CV_PI;

		if (angle > 40 && angle < 90) {
			line(cvtCSouth, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1);
			linesOfInterestSouth.insert(linesOfInterestSouth.begin(), linesSouth[i]);
			//cout << angle << endl;
		}
	}
	//cout << count << endl;
	//cout << linesSouth.size();
	//imshow("TEST", cvtCSouth);
	//waitKey(0);
	for (size_t i = 0; i < linesEast.size(); i++)
	{
		//Filtering out unwanted lines

		Vec4i l = linesEast[i];
		float angle = (atan2(abs(l[1] - l[3]), abs(l[0] - l[2]))) * 180 / CV_PI;

		if (angle > 40 && angle < 90) {
			//line(cvtCEast, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1);
			linesOfInterestEast.insert(linesOfInterestEast.begin(), linesEast[i]);
			//cout << angle << endl;
		}
	}

	/*cout << linesOfInterestSouth.size();
	imshow("TEST", cvtCSouth);
	waitKey(0);*/

	vector<vector<double>> lineInfoSouth, lineInfoEast;	
	//vector<double> intersections;
	for (size_t i = 0; i < linesOfInterestSouth.size(); i++) {
		//Getting points of intersection with counter line

		Vec4i l = linesOfInterestSouth[i];
		double slope = (double)(l[3] - l[1]) / (l[2] - l[0]);
		double intercept = (double)l[1] - slope * l[0];
		double x = (double)(intHorizontalLinePositionSouth - intercept) / slope;
		Point intersection = Point(x, intHorizontalLinePositionSouth);
		if (l[1] > l[3]) {
			if (intersection.y<l[1] && intersection.y > l[3]) {
				lineInfoSouth.push_back({ (double)intersection.x,slope,intercept });
			}
		}
		else {
			if (intersection.y<l[3] && intersection.y > l[1]) {
				lineInfoSouth.push_back({ (double)intersection.x,slope,intercept });
			}
		}
	}

	for (size_t i = 0; i < linesOfInterestEast.size(); i++) {
		//Getting points of intersection with counter line

		Vec4i l = linesOfInterestEast[i];
		double slope = (double)(l[3] - l[1]) / (l[2] - l[0]);
		double intercept = (double)l[1] - slope * l[0];
		double x = (double)(intHorizontalLinePositionEast - intercept) / slope;
		Point intersection = Point(x, intHorizontalLinePositionEast);
		if (l[1] > l[3]) {
			if (intersection.y<l[1] && intersection.y > l[3]) {
				lineInfoEast.push_back({ (double)intersection.x,slope,intercept });
			}
		}
		else {
			if (intersection.y<l[3] && intersection.y > l[1]) {
				lineInfoEast.push_back({ (double)intersection.x,slope,intercept });
			}
		}
	}

	sort(lineInfoSouth.begin(), lineInfoSouth.end());
	sort(lineInfoEast.begin(), lineInfoEast.end());
	vector<vector<double>> intersectionOfInterestSouth, intersectionOfInterestEast;
	int numberOfLanesSouth = 0;
	int numberOfLanesEast = 0;
	vector<double> savedLastIntersectionSouth, savedLastIntersectionEast;
	
	for (size_t i = 0; i < lineInfoSouth.size() - 1; i++) {
		//Getting points of intersection that we need, which are the range of each lane
		//cout << "First one is: " << lineInfoSouth[i][0] << ", Second one is: " << lineInfoSouth[i + 1][0] << endl;
		
		//if (abs(lineInfoSouth[i][0] - lineInfoSouth[i + 1][0]) > 200 && abs(lineInfoSouth[i][0] - lineInfoSouth[i + 1][0]) < 400) {
		if (abs(lineInfoSouth[i][0] - lineInfoSouth[i + 1][0]) > 50 && abs(lineInfoSouth[i][0] - lineInfoSouth[i + 1][0]) < 180) {
			numberOfLanesSouth++;
			//cout << "GAP SOUTH" << endl;
			circle(cvtCSouth, Point(lineInfoSouth[i][0], intHorizontalLinePositionSouth), 10, Scalar(255, 0, 0));
			savedLastIntersectionSouth = lineInfoSouth[i + 1];
			intersectionOfInterestSouth.push_back(lineInfoSouth[i]);
		}
	}

	for (size_t i = 0; i < lineInfoEast.size() - 1; i++) {
		//Getting points of intersection that we need, which are the range of each lane
		//cout << "First one is: " << lineInfoEast[i][0] << ", Second one is: " << lineInfoEast[i + 1][0] << endl;
		

		if (abs(lineInfoEast[i][0] - lineInfoEast[i + 1][0]) > 20 && abs(lineInfoEast[i][0] - lineInfoEast[i + 1][0]) < 180) {
			numberOfLanesEast++;
			//cout << "GAP EAST" << endl;
			//circle(cvtCEast, Point(lineInfoEast[i][0], intHorizontalLinePositionEast), 10, Scalar(255, 0, 0));
			savedLastIntersectionEast = lineInfoEast[i + 1];
			intersectionOfInterestEast.push_back(lineInfoEast[i]);
		}
	}

	
	intersectionOfInterestSouth.push_back(savedLastIntersectionSouth);
	intersectionOfInterestEast.push_back(savedLastIntersectionEast);

	//circle(cvtCSouth, Point(savedLastIntersectionSouth[0], intHorizontalLinePositionSouth), 10, Scalar(255, 0, 0));
	//circle(cvtCEast, Point(savedLastIntersectionEast[0], intHorizontalLinePositionEast), 10, Scalar(255, 0, 0));
	//numberOfLanes--;
	//cout << numberOfLanesSouth << endl;
	//cout << numberOfLanesEast << endl;

	vector<vector<Point>> laneLinesSouth, laneLinesEast;

	for (size_t i = 0; i < intersectionOfInterestSouth.size(); i++) {
		//getting end points of each lane line to draw in the while loop
		double x1 = -intersectionOfInterestSouth[i][2] / intersectionOfInterestSouth[i][1];
		double x2 = (copySouth.rows - intersectionOfInterestSouth[i][2]) / intersectionOfInterestSouth[i][1];
		laneLinesSouth.push_back({ Point(x1, 0), Point(x2,copySouth.rows) });
	}

	for (size_t i = 0; i < intersectionOfInterestEast.size(); i++) {
		//getting end points of each lane line to draw in the while loop
		double x1 = -intersectionOfInterestEast[i][2] / intersectionOfInterestEast[i][1];
		double x2 = (copyEast.rows - intersectionOfInterestEast[i][2]) / intersectionOfInterestEast[i][1];
		laneLinesEast.push_back({ Point(x1, 0), Point(x2,copyEast.rows) });
	}
	
	/*line(cvtCSouth, Point(0, intHorizontalLinePositionSouth), Point(imgFrame1South.cols-1, intHorizontalLinePositionSouth), Scalar(0, 0, 255), 1);
	namedWindow("My Window", 1);
	setMouseCallback("My Window", CallBackFunc, NULL);
	imshow("My Window", cvtCEast);
	imshow("test", copyEast);*/
	/*for (size_t i = 0; i < laneLinesEast.size(); i++)
	{
		line(cvtCEast, laneLinesEast[i][0], laneLinesEast[i][1], SCALAR_RED, 2);;
	}
	imshow("TEST", cvtCEast);
	waitKey(0);*/
	
	
	
	//VideoWriter video("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(imgFrame2South.cols, imgFrame2South.rows));
	//waitKey(0);
	while ((capVideo2.isOpened() || capVideo.isOpened()) && chCheckForEscKey != 27) {
		vector<Blob> currentFrameBlobsSouth, currentFrameBlobsEast;

		Mat imgFrame1CopySouth = imgFrame1South.clone();
		Mat imgFrame2CopySouth = imgFrame2South.clone();

		Mat imgFrame1CopyEast = imgFrame1East.clone();
		Mat imgFrame2CopyEast = imgFrame2East.clone();

		Mat imgDifferenceSouth, imgDifferenceEast;
		Mat imgThreshSouth, imgThreshEast;

		cvtColor(imgFrame1CopySouth, imgFrame1CopySouth, CV_BGR2GRAY);
		cvtColor(imgFrame2CopySouth, imgFrame2CopySouth, CV_BGR2GRAY);
		cvtColor(imgFrame1CopyEast, imgFrame1CopyEast, CV_BGR2GRAY);
		cvtColor(imgFrame2CopyEast, imgFrame2CopyEast, CV_BGR2GRAY);

		GaussianBlur(imgFrame1CopySouth, imgFrame1CopySouth, Size(5, 5), 0);
		GaussianBlur(imgFrame2CopySouth, imgFrame2CopySouth, Size(5, 5), 0);
		GaussianBlur(imgFrame1CopyEast, imgFrame1CopyEast, Size(5, 5), 0);
		GaussianBlur(imgFrame2CopyEast, imgFrame2CopyEast, Size(5, 5), 0);

		absdiff(imgFrame1CopySouth, imgFrame2CopySouth, imgDifferenceSouth);
		absdiff(imgFrame1CopyEast, imgFrame2CopyEast, imgDifferenceEast);

		threshold(imgDifferenceSouth, imgThreshSouth, 30, 255.0, CV_THRESH_BINARY);
		threshold(imgDifferenceEast, imgThreshEast, 30, 255.0, CV_THRESH_BINARY);

		//imshow("imgThreshSouth", imgThreshSouth);
		//imshow("imgThreshEast", imgThreshEast);

		Mat structuringElement3x3 = getStructuringElement(MORPH_RECT, Size(3, 3));
		Mat structuringElement5x5 = getStructuringElement(MORPH_RECT, Size(5, 5));
		Mat structuringElement7x7 = getStructuringElement(MORPH_RECT, Size(7, 7));
		Mat structuringElement15x15 = getStructuringElement(MORPH_RECT, Size(15, 15));

		for (unsigned int i = 0; i < 2; i++) {
			dilate(imgThreshSouth, imgThreshSouth, structuringElement5x5);
			dilate(imgThreshSouth, imgThreshSouth, structuringElement5x5);
			erode(imgThreshSouth, imgThreshSouth, structuringElement5x5);
		}

		for (unsigned int i = 0; i < 2; i++) {
			dilate(imgThreshEast, imgThreshEast, structuringElement5x5);
			dilate(imgThreshEast, imgThreshEast, structuringElement5x5);
			erode(imgThreshEast, imgThreshEast, structuringElement5x5);
		}

		Mat imgThreshCopySouth = imgThreshSouth.clone();
		Mat imgThreshCopyEast = imgThreshEast.clone();

		vector<vector<Point>> contoursSouth, contoursEast;

		findContours(imgThreshCopySouth, contoursSouth, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		findContours(imgThreshCopyEast, contoursEast, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		//drawAndShowContours(imgThreshSouth.size(), contoursSouth, "imgContoursSouth");
		//drawAndShowContours(imgThreshEast.size(), contoursEast, "imgContoursEast");

		vector<vector<Point>> convexHullsSouth(contoursSouth.size());
		vector<vector<Point>> convexHullsEast(contoursEast.size());

		for (unsigned int i = 0; i < contoursSouth.size(); i++) {
			convexHull(contoursSouth[i], convexHullsSouth[i]);
		}

		for (unsigned int i = 0; i < contoursEast.size(); i++) {
			convexHull(contoursEast[i], convexHullsEast[i]);
		}

		//drawAndShowContours(imgThreshSouth.size(), convexHullsSouth, "imgConvexHullsSouth");
		//drawAndShowContours(imgThreshEast.size(), convexHullsEast, "imgConvexHullsEast");

		for (auto &convexHull : convexHullsSouth) {
			Blob possibleBlob(convexHull);

			if (possibleBlob.currentBoundingRect.area() > 400 &&
				possibleBlob.dblCurrentAspectRatio > 0.2 &&
				possibleBlob.dblCurrentAspectRatio < 4.0 &&
				possibleBlob.currentBoundingRect.width > 30 &&
				possibleBlob.currentBoundingRect.height > 30 &&
				possibleBlob.dblCurrentDiagonalSize > 60.0 &&
				(contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.50) {
				currentFrameBlobsSouth.push_back(possibleBlob);
			}
		}

		for (auto &convexHull : convexHullsEast) {
			Blob possibleBlob(convexHull);

			if (possibleBlob.currentBoundingRect.area() > 400 &&
				possibleBlob.dblCurrentAspectRatio > 0.2 &&
				possibleBlob.dblCurrentAspectRatio < 4.0 &&
				possibleBlob.currentBoundingRect.width > 30 &&
				possibleBlob.currentBoundingRect.height > 30 &&
				possibleBlob.dblCurrentDiagonalSize > 60.0 &&
				(contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.50) {
				currentFrameBlobsEast.push_back(possibleBlob);
			}
		}

		//drawAndShowContours(imgThreshSouth.size(), currentFrameBlobsSouth, "imgCurrentFrameBlobsSouth");
		//drawAndShowContours(imgThreshEast.size(), currentFrameBlobsEast, "imgCurrentFrameBlobsEast");

		if (blnFirstFrame == true) {
			for (auto &currentFrameBlob : currentFrameBlobsSouth) {
				blobsSouth.push_back(currentFrameBlob);
			}

			for (auto &currentFrameBlob : currentFrameBlobsEast) {
				blobsEast.push_back(currentFrameBlob);
			}
		}
		else {
			matchCurrentFrameBlobsToExistingBlobs(blobsSouth, currentFrameBlobsSouth);
			matchCurrentFrameBlobsToExistingBlobs(blobsEast, currentFrameBlobsEast);
		}

		//drawAndShowContours(imgThreshSouth.size(), blobsSouth, "imgBlobsSouth");
		//drawAndShowContours(imgThreshEast.size(), blobsEast, "imgBlobsEast");

		imgFrame2CopySouth = imgFrame2South.clone();          // get another copy of frame 2 since we changed the previous frame 2 copy in the processing above
		imgFrame2CopyEast = imgFrame2East.clone();


		//drawBlobInfoOnImage(blobsSouth, imgFrame2CopySouth);
		//drawBlobInfoOnImage(blobsEast, imgFrame2CopyEast);

		bool blnAtLeastOneBlobCrossedTheLineSouth = checkIfBlobsCrossedTheLine(blobsSouth, intersectionOfInterestSouth, intHorizontalLinePositionSouth, intHorizontalLinePositionSouth2, carCountSouth, midCountSouth, midCount2South, midCount3South, leftCountSouth, rightCountSouth, carsSouth);
		bool blnAtLeastOneBlobCrossedTheLineEast = checkIfBlobsCrossedTheLine(blobsEast, intersectionOfInterestEast, intHorizontalLinePositionEast, intHorizontalLinePositionEast2, carCountEast, midCountEast, midCount2East, midCount3East, leftCountEast, rightCountEast, carsEast);

		if (blnAtLeastOneBlobCrossedTheLineSouth == true) {
			line(imgFrame2CopySouth, crossingLineSouth[0], crossingLineSouth[1], SCALAR_GREEN, 2);
		}
		else {
			line(imgFrame2CopySouth, crossingLineSouth[0], crossingLineSouth[1], SCALAR_RED, 2);
		}

		if (blnAtLeastOneBlobCrossedTheLineEast == true) {
			line(imgFrame2CopyEast, crossingLineEast[0], crossingLineEast[1], SCALAR_GREEN, 2);
		}
		else {
			line(imgFrame2CopyEast, crossingLineEast[0], crossingLineEast[1], SCALAR_RED, 2);
		}

		if (blnAtLeastOneBlobCrossedTheLineSouth == true) {
			line(imgFrame2CopySouth, crossingLineSouth2[0], crossingLineSouth2[1], SCALAR_GREEN, 2);
		}
		else {
			line(imgFrame2CopySouth, crossingLineSouth2[0], crossingLineSouth2[1], SCALAR_RED, 2);
		}

		if (blnAtLeastOneBlobCrossedTheLineEast == true) {
			line(imgFrame2CopyEast, crossingLineEast2[0], crossingLineEast2[1], SCALAR_GREEN, 2);
		}
		else {
			line(imgFrame2CopyEast, crossingLineEast2[0], crossingLineEast2[1], SCALAR_RED, 2);
		}
		

		drawCarCountOnImage(numberOfLanesSouth, carCountSouth, midCountSouth, midCount2South, midCount3South, leftCountSouth, rightCountSouth, imgFrame2CopySouth);
		drawCarCountOnImage(numberOfLanesEast, carCountEast, midCountEast, midCount2East, midCount3East, leftCountEast, rightCountEast, imgFrame2CopyEast);

		for (size_t i = 0; i < laneLinesSouth.size(); i++)
		{
			line(imgFrame2CopySouth, laneLinesSouth[i][0], laneLinesSouth[i][1], SCALAR_RED, 2);;
		}

		for (size_t i = 0; i < laneLinesEast.size(); i++)
		{
			line(imgFrame2CopyEast, laneLinesEast[i][0], laneLinesEast[i][1], SCALAR_RED, 2);;
		}

		imshow("imgFrame2CopySouth", imgFrame2CopySouth);
		imshow("imgFrame2CopyEast", imgFrame2CopyEast);
		//video.write(imgFrame2CopySouth);

		//waitKey(0);                 // uncomment this line to go frame by frame for debugging

		// now we prepare for the next iteration

		currentFrameBlobsSouth.clear();
		currentFrameBlobsEast.clear();

		imgFrame1South = imgFrame2South.clone();           // move frame 1 up to where frame 2 is
		capVideo.read(imgFrame2South);
		resize(imgFrame2South, imgFrame2SouthResized, Size(720,480),0,0,INTER_CUBIC);// Size(imgFrame2South.cols / 2, imgFrame2South.rows / 2));
		imgFrame2South = imgFrame2SouthResized;

		imgFrame1East = imgFrame2East.clone();
		capVideo2.read(imgFrame2East);
		resize(imgFrame2East, imgFrame2EastResized, Size(720, 480), 0, 0, INTER_CUBIC);// Size(imgFrame2South.cols / 2, imgFrame2South.rows / 2));
		imgFrame2East = imgFrame2EastResized;

		/*if ((capVideo.get(CV_CAP_PROP_POS_FRAMES) + 1) < capVideo.get(CV_CAP_PROP_FRAME_COUNT)) {
			capVideo.read(imgFrame2);
		} else {
			cout << "end of video\n";
			break;
		}*/

		blnFirstFrame = false;
		
		TrafficLogic(carCountSouth,carCountEast, carsSouth, carsEast, max, maxTime, leftGreenSouth, leftGreenEast);
		chCheckForEscKey = waitKey(1);
	}

	if (chCheckForEscKey != 27) {               // if the user did not press esc (i.e. we reached the end of the video)
		waitKey(0);                         // hold the windows open to allow the "end of video" message to show
	}
	// note that if the user did press esc, we don't need to hold the windows open, we can simply let the program end which will close the windows
	//video.release();
	return(0);
}

void TrafficLogic(int &carCountSouth, int &carCountEast, int carsSouth[], int carsEast[], int &max, int &maxTime, boolean &leftGreenSouth, boolean &leftGreenEast) {

	time_t seconds = time(NULL)-startTime;
	//long seconds = System.nanoTime() / 1000000000 - startTime;
	long yellowTime = 3;

	if (seconds > yellowTime) { // After 3 seconds, the first cycle starts

		if (!cycle) {
			greenTimeSN = countMaxTime(carsSouth, max, maxTime, leftGreenSouth);

			
			greenTimeWE = countMaxTime(carsEast, max, maxTime, leftGreenEast);

			if (greenTimeSN > 0 || greenTimeWE > 0) {
			//if (greenTimeSN > 0 && greenTimeWE > 0) {
				cycle = true;
				//yellowTime = 3;
			}
			else {
				cout << "No cars detected" << endl;
			}
		}
		else {
			// SN first then WE // FIX THE CASE WHEN greenTimeSN IS ZERO
			if (!SNRedTrue) {
				//greenTimeWE = 2 * (carCountEast);// +eastCount);

				//Keep track of the max time for East				
				greenTimeWE = countMaxTime(carsEast, max, maxTime, leftGreenEast);

				// while SNRedTrue is false (SN is green), keep into account of the WE count
				//SNGreen(); // Set SN to green
				if (!alreadyPrinted) {
					if (leftGreenSouth) {
						//system("mosquitto_pub -h raspberrypi -t test -m \"4\"");
						cout << "South and North Left are now Green for " << greenTimeSN << " seconds" << endl;
						alreadyPrinted = true;
					}
					else {
						//system("mosquitto_pub -h raspberrypi -t test -m \"0\"");
						cout << "South and North are now Green for " << greenTimeSN << " seconds" << endl;
						alreadyPrinted = true;
					}					
				}				
				if (seconds - yellowTime >= greenTimeSN) { // Set SN Red after the time for green (2*(southCount+northCount))
					if (leftGreenSouth) {
						greenTimeSN = countMaxTime(carsSouth, max, maxTime, leftGreenSouth);
						if (leftGreenSouth) {
							leftGreenSouth = false;
						}
						startTime = time(NULL);
					}else {
						if (greenTimeWE != 0) {
							//do this only if there are cars detected on other side
							SNRedTrue = true;
							WERedTrue = false;
							waitStartTime = time(NULL);
							//SNRed();
							cout << "South and North are now Yellow for " << yellowTime << " seconds" << endl;
							//system("mosquitto_pub -h raspberrypi -t test -m \"2\"");
							alreadyPrinted = false;
						}
						else {
							greenTimeSN = countMaxTime(carsSouth, max, maxTime, leftGreenSouth);
							startTime = time(NULL);
						}
					}
				}
				//System.out.println("SN is " + greenTimeSN);
				//System.out.println("SN is " + (seconds - yellowTime));
			}
			else if (!WERedTrue) {
				// greenTimeSN = 2*(southCount+northCount);
				long waitEndTime = time(NULL);
				if (waitEndTime - waitStartTime >= yellowTime) {
					//int greenTimeSN2 = 2 * (carCountSouth);// +northCount);
					int greenTimeSN2;
					
					//Keep track of the max time for South
					
					greenTimeSN2 = countMaxTime(carsSouth, max, maxTime, leftGreenSouth);
					
					//greenTimeSN is taken into account below, if it keeps getting updated, we will never reach >= greenTimeWE
					//WEGreen();
					if (!alreadyPrinted) {
						if (leftGreenSouth) {
							//system("mosquitto_pub -h raspberrypi -t test -m \"5\"");
							cout << "West and East Left are now Green for " << greenTimeWE << " seconds" << endl;
							alreadyPrinted = true;
						}
						else {
							cout << "West and East are now green for " << greenTimeWE << " seconds" << endl;
							//system("mosquitto_pub -h raspberrypi -t test -m \"1\"");
							alreadyPrinted = true;
						}
					}
					
					if (seconds - yellowTime - greenTimeSN - yellowTime >= greenTimeWE) {

						//WERed();
						cout << "West and East are now yellow for " << yellowTime << " seconds" << endl;
						//system("mosquitto_pub -h raspberrypi -t test -m \"3\"");
						if (seconds - yellowTime - greenTimeSN - yellowTime - greenTimeWE >= 0) {
							WERedTrue = true;
							SNRedTrue = false;
							cycle = false;
							startTime = time(NULL);
							greenTimeSN = greenTimeSN2;
							alreadyPrinted = false;
						}
						else {
							//Maybe dont need because time resets anyways?
							
							greenTimeWE = countMaxTime(carsEast, max, maxTime, leftGreenEast);
						}
					}
				}
				//System.out.println("WE is " + greenTimeWE);
				//System.out.println("WE is " + (seconds - yellowTime - greenTimeSN - 3));

			}
		}

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(vector<Blob> &existingBlobs, vector<Blob> &currentFrameBlobs) {

	for (auto &existingBlob : existingBlobs) {

		existingBlob.blnCurrentMatchFoundOrNewBlob = false;

		existingBlob.predictNextPosition();
	}

	for (auto &currentFrameBlob : currentFrameBlobs) {

		int intIndexOfLeastDistance = 0;
		double dblLeastDistance = 100000.0;

		for (unsigned int i = 0; i < existingBlobs.size(); i++) {

			if (existingBlobs[i].blnStillBeingTracked == true) {

				double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);

				if (dblDistance < dblLeastDistance) {
					dblLeastDistance = dblDistance;
					intIndexOfLeastDistance = i;
				}
			}
		}

		if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 0.5) {
			addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
		}
		else {
			addNewBlob(currentFrameBlob, existingBlobs);
		}

	}

	for (auto &existingBlob : existingBlobs) {

		if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
			existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
		}

		if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
			existingBlob.blnStillBeingTracked = false;
		}

	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addBlobToExistingBlobs(Blob &currentFrameBlob, vector<Blob> &existingBlobs, int &intIndex) {

	existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
	existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;

	existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());

	existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
	existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;

	existingBlobs[intIndex].blnStillBeingTracked = true;
	existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addNewBlob(Blob &currentFrameBlob, vector<Blob> &existingBlobs) {

	currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

	existingBlobs.push_back(currentFrameBlob);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double distanceBetweenPoints(Point point1, Point point2) {

	int intX = abs(point1.x - point2.x);
	int intY = abs(point1.y - point2.y);

	return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawAndShowContours(Size imageSize, vector<vector<Point> > contours, string strImageName) {
	Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

	drawContours(image, contours, -1, SCALAR_WHITE, -1);

	imshow(strImageName, image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawAndShowContours(Size imageSize, vector<Blob> blobs, string strImageName) {

	Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

	vector<vector<Point> > contours;

	for (auto &blob : blobs) {
		if (blob.blnStillBeingTracked == true) {
			contours.push_back(blob.currentContour);
		}
	}

	drawContours(image, contours, -1, SCALAR_WHITE, -1);

	imshow(strImageName, image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool checkIfBlobsCrossedTheLine(vector<Blob> &blobs, vector<vector<double>> &points, int &intHorizontalLinePosition, int &intHorizontalLinePosition2, int &carCount, int &midCount, int &midCount2, int &midCount3, int &leftCount, int &rightCount, int cars[]) {
	bool blnAtLeastOneBlobCrossedTheLine = false;
	int numberOfLanes = points.size() - 1;
	double point1, point2, point3, point4, point5, point6;

	if (numberOfLanes == 1) {
		point1 = points[0][0];
		point2 = points[1][0];
	}
	else if (numberOfLanes == 2) {
		point1 = points[0][0];
		point2 = points[1][0];
		point3 = points[2][0];
	}
	else if (numberOfLanes == 3) {
		point1 = points[0][0];
		point2 = points[1][0];
		point3 = points[2][0];
		point4 = points[3][0];
	}
	else if (numberOfLanes == 4) {
		point1 = points[0][0];
		point2 = points[1][0];
		point3 = points[2][0];
		point4 = points[3][0];
		point5 = points[4][0];
	}
	else {
		point1 = points[0][0];
		point2 = points[1][0];
		point3 = points[2][0];
		point4 = points[3][0];
		point5 = points[4][0];
		point6 = points[5][0];
	}


	for (auto blob : blobs) {

		if (blob.blnStillBeingTracked == true && blob.centerPositions.size() >= 2) {
			int prevFrameIndex = (int)blob.centerPositions.size() - 2;
			int currFrameIndex = (int)blob.centerPositions.size() - 1;

			if (blob.centerPositions[prevFrameIndex].y <= intHorizontalLinePosition && blob.centerPositions[currFrameIndex].y > intHorizontalLinePosition) {
				carCount++;
				blnAtLeastOneBlobCrossedTheLine = true;
				if (numberOfLanes == 1) {
					if (blob.centerPositions[currFrameIndex].x < point1 && blob.centerPositions[currFrameIndex].x >= point2) {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else if (numberOfLanes == 2) {
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						leftCount++;
						cars[0]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						rightCount++;
						cars[4]++;
					}
					else {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else if (numberOfLanes == 3) {
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						leftCount++;
						cars[0]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						midCount++;
						cars[1]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point3 && blob.centerPositions[currFrameIndex].x < point4) {
						rightCount++;
						cars[5]++;
					}
					else {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else if (numberOfLanes == 4) {
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						leftCount++;
						cars[0]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						midCount++;
						cars[1]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point3 && blob.centerPositions[currFrameIndex].x < point4) {
						midCount2++;
						cars[2]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point4 && blob.centerPositions[currFrameIndex].x < point5) {
						rightCount++;
						cars[4]++;
					}
					else {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else {
					//5 lanes
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						//cout << "LEFT" << endl;
						leftCount++;
						cars[0]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						//cout << "MID1" << endl;
						midCount++;
						cars[1]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point3 && blob.centerPositions[currFrameIndex].x < point4) {
						//cout << "MID2" << endl;
						midCount2++;
						cars[2]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point4 && blob.centerPositions[currFrameIndex].x < point5) {
						//cout << "MID3" << endl;
						midCount3++;
						cars[3]++;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point5 && blob.centerPositions[currFrameIndex].x < point6) {
						//cout << "RIGHT" << endl;
						rightCount++;
						cars[4]++;
					}
					else {
						//cout << "ERROR, CAR OUT OF BOUNDS";
					}
					
				}
			}

			if (blob.centerPositions[prevFrameIndex].y <= intHorizontalLinePosition2 && blob.centerPositions[currFrameIndex].y > intHorizontalLinePosition2) {
				carCount--;
				blnAtLeastOneBlobCrossedTheLine = true;
				if (numberOfLanes == 1) {
					if (blob.centerPositions[currFrameIndex].x < point1 && blob.centerPositions[currFrameIndex].x >= point2) {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else if (numberOfLanes == 2) {
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						leftCount--;
						cars[0]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						rightCount--;
						cars[4]--;
					}
					else {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else if (numberOfLanes == 3) {
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						leftCount--;
						cars[0]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						midCount--;
						cars[1]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point3 && blob.centerPositions[currFrameIndex].x < point4) {
						rightCount--;
						cars[4]--;
					}
					else {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else if (numberOfLanes == 4) {
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						leftCount--;
						cars[0]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						midCount--;
						cars[1]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point3 && blob.centerPositions[currFrameIndex].x < point4) {
						midCount2--;
						cars[2]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point4 && blob.centerPositions[currFrameIndex].x < point5) {
						rightCount--;
						cars[4]--;
					}
					else {
						cout << "ERROR, CAR OUT OF BOUNDS";
					}
				}
				else {
					//5 lanes
					if (blob.centerPositions[currFrameIndex].x >= point1 && blob.centerPositions[currFrameIndex].x < point2) {
						//cout << "LEFT" << endl;
						leftCount--;
						cars[0]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point2 && blob.centerPositions[currFrameIndex].x < point3) {
						//cout << "MID1" << endl;
						midCount--;
						cars[1]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point3 && blob.centerPositions[currFrameIndex].x < point4) {
						//cout << "MID2" << endl;
						midCount2--;
						cars[2]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point4 && blob.centerPositions[currFrameIndex].x < point5) {
						//cout << "MID3" << endl;
						midCount3--;
						cars[3]--;
					}
					else if (blob.centerPositions[currFrameIndex].x >= point5 && blob.centerPositions[currFrameIndex].x < point6) {
						//cout << "RIGHT" << endl;
						rightCount--;
						cars[4]--;
					}
					else {
						//cout << "ERROR, CAR OUT OF BOUNDS";
					}

				}
			}
		}

	}

	return blnAtLeastOneBlobCrossedTheLine;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void drawBlobInfoOnImage(vector<Blob> &blobs, Mat &imgFrame2Copy) {

	for (unsigned int i = 0; i < blobs.size(); i++) {

		if (blobs[i].blnStillBeingTracked == true) {
			rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_RED, 2);

			int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
			double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
			int intFontThickness = (int)round(dblFontScale * 1.0);

			//putText(imgFrame2Copy, to_string(i), blobs[i].centerPositions.back(), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawCarCountOnImage(int &numberOfLanes, int &carCount, int&midCount, int&midCount2, int&midCount3, int&leftCount, int&rightCount, Mat &imgFrame2Copy) {

	int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
	double dblFontScale = (720*480)/300000.0;// (imgFrame2Copy.rows * imgFrame2Copy.cols) / 300000.0;
	int intFontThickness = (int)round(dblFontScale);

	Size textSize = getTextSize(to_string(carCount), intFontFace, dblFontScale, intFontThickness, 0);

	Point ptTextBottomLeftPosition, midCountCounter, midCountCounter2, midCountCounter3, leftCountCounter, rightCountCounter;

	ptTextBottomLeftPosition.x = imgFrame2Copy.cols - 1 - (int)((double)textSize.width);
	ptTextBottomLeftPosition.y = (int)((double)textSize.height);

	if (numberOfLanes == 1) {
		midCountCounter.x = ptTextBottomLeftPosition.x - (double)textSize.width;
		rightCountCounter.y = ptTextBottomLeftPosition.y;

		putText(imgFrame2Copy, to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
	}
	else if (numberOfLanes == 2) {
		rightCountCounter.x = ptTextBottomLeftPosition.x - (double)textSize.width;
		rightCountCounter.y = ptTextBottomLeftPosition.y;

		leftCountCounter.x = rightCountCounter.x - (double)textSize.width;
		leftCountCounter.y = rightCountCounter.y;

		putText(imgFrame2Copy, to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(rightCount), rightCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(leftCount), leftCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
	}
	else if (numberOfLanes == 3) {

		rightCountCounter.x = ptTextBottomLeftPosition.x - (double)textSize.width;
		rightCountCounter.y = ptTextBottomLeftPosition.y;

		midCountCounter.x = rightCountCounter.x - (double)textSize.width;
		midCountCounter.y = rightCountCounter.y;

		leftCountCounter.x = midCountCounter.x - (double)textSize.width;
		leftCountCounter.y = midCountCounter.y;

		putText(imgFrame2Copy, to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(midCount), midCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(leftCount), leftCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(rightCount), rightCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
	}
	else if (numberOfLanes == 4) {

		rightCountCounter.x = ptTextBottomLeftPosition.x - (double)textSize.width;
		rightCountCounter.y = ptTextBottomLeftPosition.y;

		midCountCounter2.x = rightCountCounter.x - (double)textSize.width;
		midCountCounter2.y = rightCountCounter.y;

		midCountCounter.x = midCountCounter2.x - (double)textSize.width;
		midCountCounter.y = midCountCounter2.y;

		leftCountCounter.x = midCountCounter.x - (double)textSize.width;
		leftCountCounter.y = midCountCounter.y;

		putText(imgFrame2Copy, to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(midCount), midCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(leftCount), leftCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(rightCount), rightCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(midCount2), midCountCounter2, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);

	}
	else {
		rightCountCounter.x = ptTextBottomLeftPosition.x - (double)textSize.width;
		rightCountCounter.y = ptTextBottomLeftPosition.y;

		midCountCounter3.x = rightCountCounter.x - (double)textSize.width;
		midCountCounter3.y = rightCountCounter.y;

		midCountCounter2.x = midCountCounter3.x - (double)textSize.width;
		midCountCounter2.y = midCountCounter3.y;

		midCountCounter.x = midCountCounter2.x - (double)textSize.width;
		midCountCounter.y = midCountCounter2.y;

		leftCountCounter.x = midCountCounter.x - (double)textSize.width;
		leftCountCounter.y = midCountCounter.y;

		putText(imgFrame2Copy, to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(midCount), midCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(leftCount), leftCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(rightCount), rightCountCounter, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(midCount2), midCountCounter2, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
		putText(imgFrame2Copy, to_string(midCount3), midCountCounter3, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);

		//cout << leftCount << ", " << midCount << ", " << midCount2 << ", " << midCount3 << ", " << rightCount << ", " << carCount << endl;
		
		/*cout << "This intersection is now Green." << endl;
		cout << "The left lane has " << leftCount << " cars." << endl;
		cout << "The left-most midlane has " << midCount << " cars." << endl;
		cout << "The midlane has " << midCount2 << " cars." << endl;
		cout << "The right-most midlane has " << midCount3 << " cars." << endl;
		cout << "The right lane has " << rightCount << " cars." << endl;
		cout << "The total count of cars is " << carCount << " cars." << endl;
		cout << "The timer is set to " << 2*carCount << " seconds." << endl << endl;*/


	}
}

Mat whiteFilter(const Mat& src)
{
	assert(src.type() == CV_8UC3);

	Mat whiteOnly;
	inRange(src, Scalar(170, 170, 170), Scalar(255, 255, 255), whiteOnly);

	return whiteOnly;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if (event == EVENT_RBUTTONDOWN)
	{
		cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if (event == EVENT_MOUSEMOVE)
	{
		cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

	}
}

int countMaxTime(int cars[], int &max, int &maxTime, boolean &leftGreen) {
	maxTime = 0;
	for (int i = 0; i < (sizeof(cars) / sizeof(*cars)); i++) {
		if (i == 0) {
			max = cars[i];
			leftGreen = true;
		}
		else {
			if (cars[i] > max) {
				max = cars[i];
				leftGreen = false;
			}
		}
	}
	maxTime = 2 * max + max;
	/*for (int i = 1; i <= max; i++) {
		maxTime += 2 * i;
	}*/
	return maxTime;
		/*
			for (int j = 1; j <= cars[i]; j++) {
				max += 2 * j;
				maxTime = max;
			}
		}
		else {
			for (int j = 1; j <= cars[i]; j++) {
				maxTime += 2 * j;
			}
			if (maxTime > max) {
				max = maxTime;
			}
		}
	}
	int greenTime = max;
	return greenTime;*/
}
