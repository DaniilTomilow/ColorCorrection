﻿#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "Windows.h"
#include "NuiApi.h"
#include <algorithm>    // std::sort

using namespace cv;
using namespace std;

// OpenCV and VS 2015: http://dogfeatherdesign.com/opencv-3-0-microsoft-visual-studio-2015-cmake-and-c/

// Example http://answers.opencv.org/question/54785/face-detect-kinect-v2-opencv/
// https://msdn.microsoft.com/en-us/library/dn791996.aspx

// Kinect with CS: https://msdn.microsoft.com/en-us/library/hh855360.aspx
// OpenCV: http://docs.opencv.org/2.4/doc/tutorials/introduction/windows_visual_studio_Opencv/windows_visual_studio_Opencv.html
// Kinect C++ Reference https://msdn.microsoft.com/en-us/library/hh855364.aspx


class Kicc {

private:
	// Kinect resolution
	NUI_IMAGE_RESOLUTION resolution = NUI_IMAGE_RESOLUTION_640x480;
	unsigned int width;
	unsigned int height;

	// Screen Size
	int screenWidth;
	int screenHeight;

	Mat input;

	// https://gist.github.com/DaniilTomilow/1088bca80f5a1f449f15
	void getDesktopResolution(int& width, int& height, int& posX, int& posY)
	{
		int index = 0;
		DISPLAY_DEVICE dd;
		dd.cb = sizeof(DISPLAY_DEVICE);

		while (EnumDisplayDevices(NULL, index++, &dd, 0))
		{
			if (dd.StateFlags & DISPLAY_DEVICE_ATTACHED_TO_DESKTOP) {
				DEVMODE mode;
				mode.dmSize = sizeof(mode);
				EnumDisplaySettings(dd.DeviceName, ENUM_CURRENT_SETTINGS, &mode);

				width = mode.dmPelsWidth;
				height = mode.dmPelsHeight;
				posX = mode.dmPosition.x;
				posY = mode.dmPosition.y;

				if (dd.StateFlags & DISPLAY_DEVICE_PRIMARY_DEVICE) printf("* ");

				cout << "Device Name: " << dd.DeviceName << "; Device String: " << dd.DeviceString << endl;
				cout << "5: Width: " << width << "; height: " << height << '\n';

				// We found a second monitor!
				if (!(dd.StateFlags & DISPLAY_DEVICE_PRIMARY_DEVICE)) break;
			}
		}
	}

	// Mouse callback for manual surface
	static void callBackFunc(int event, int x, int y, int flags, void* userdata)
	{
		vector<Point>* points = (vector<Point>*) userdata;

		if (event == EVENT_LBUTTONDOWN && points->size() < 4)
		{
			points->push_back(Point(x, y));
			cout << "Added Point" << endl;
		}
		else if (event == EVENT_RBUTTONDOWN && points->size() > 0)
		{
			points->pop_back();
			cout << "Removed Point" << endl;
		}
	}

	// Adjust surface perspective and order the points
	void adjustPerspective(Mat& src, Mat& surface, vector<Point>& points) {

		vector<Point> contours_poly = points;
		// Reorder Points
		sort(contours_poly.begin(), contours_poly.end(), [](Point pt1, Point pt2) { return pt1.y > pt2.y; });
		Point tl, tr, br, bl;
		if (contours_poly[0].x < contours_poly[1].x) { tl = contours_poly[0]; tr = contours_poly[1]; }
		else { tl = contours_poly[1]; tr = contours_poly[0]; }
		if (contours_poly[2].x < contours_poly[3].x) { bl = contours_poly[2]; br = contours_poly[3]; }
		else { bl = contours_poly[3]; br = contours_poly[2]; }


		/** Order WTF
		2st-------4nd
		|         |
		|         |
		|         |
		1rd-------3th
		*/
		Size s = src.size();
		vector<Point2f> quad_pts{ Point2f(bl.x, bl.y), Point2f(tl.x, tl.y),  Point2f(br.x, br.y), Point2f(tr.x, tr.y) };
		vector<Point2f> squre_pts{ Point2f(0, 0),       Point2f(0, s.height), Point2f(s.width, 0), Point2f(s.width, s.height) };

		// Warp perspective
		Mat transmtx = getPerspectiveTransform(quad_pts, squre_pts);
		warpPerspective(src, surface, transmtx, s);

		// Resize surface and orig to screen size
		resize(surface, surface, Size(screenWidth, screenHeight));
	}

	// Find conrours on surface
	bool findSurfaceContours(Mat& src, Mat& redSurface, vector<Point>& contours_poly) {

		// Convert to HSV
		Mat hsv_image;
		cvtColor(redSurface, hsv_image, cv::COLOR_BGR2HSV);

		Mat lower_red_hue_range, upper_red_hue_range;
		inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
		inRange(hsv_image, cv::Scalar(120, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

		// Combine the above two images
		addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, redSurface);

		//Mat blur;
		//GaussianBlur(mask, blur, cv::Size(7, 7), 1.5, 1.5);
		Mat tre;
		threshold(redSurface, tre, 170, 255, CV_THRESH_BINARY);

		Mat canny;
		Canny(tre, canny, 0, 50, 5); // imshow("Canny",canny);

		vector<vector<Point>> contours;
		findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		if (contours.size() == 0) {
			// Try to set manual Surface
			if (!manualSurface(src, contours_poly)) return false;
		}
		else {
			int largest_contour_index = 0;
			int largest_area = 0;
			for (unsigned int i = 0; i < contours.size(); i++)
			{
				double a = contourArea(contours[i], false);  //  Find the area of contour
				if (a > largest_area) {
					largest_area = a;
					largest_contour_index = i;                //Store the index of largest contour
				}
			}

			// Largest contour
			vector<Point> contour = contours[largest_contour_index];
			approxPolyDP(Mat(contour), contours_poly, arcLength(Mat(contour), true) * 0.02, true);

			// Number of vertices of polygonal curve
			int vtc = contours_poly.size();

			// Skip small or non-convex objects 
			if (vtc != 4 || fabs(contourArea(contour)) < 100 || !isContourConvex(contours_poly)) {
				std::cout << "No surface found" << std::endl;
				// Empty points
				contours_poly = {};
				// Try to set manual points
				if (!manualSurface(src, contours_poly))
					return false;
			}

			return true;
		}

		return false;
	}

	// Get next kinect frame as Mat
	Mat nextKinectFrame(INuiSensor* pNuiSensor, HANDLE hColorStreamHandle) {
		NUI_IMAGE_FRAME nuiImage;
		HRESULT hr;
		hr = pNuiSensor->NuiImageStreamGetNextFrame(hColorStreamHandle, 1000, &nuiImage);
		if (FAILED(hr)) {
			std::cerr << "Error : NuiImageStreamGetNextFrame" << endl;
		}

		INuiFrameTexture* texture = nuiImage.pFrameTexture;
		NUI_LOCKED_RECT rect;
		texture->LockRect(0, &rect, nullptr, 0);
		cv::Mat frame(height, width, CV_8UC4, reinterpret_cast<unsigned char*>(rect.pBits));
		flip(frame, frame, 1);

		texture->UnlockRect(0);
		pNuiSensor->NuiImageStreamReleaseFrame(hColorStreamHandle, &nuiImage);

		return frame;
	}

	// Init Kinect Sensor
	HRESULT initKinect(INuiSensor** sensor, HANDLE* hColorStreamHandle) {
		// If we want the width
		unsigned long widthTmp, heightTmp;
		NuiImageResolutionToSize(resolution, widthTmp, heightTmp);
		width = static_cast<unsigned int>(widthTmp);
		height = static_cast<unsigned int>(heightTmp);

		NUI_IMAGE_TYPE imageType = NUI_IMAGE_TYPE_COLOR;

		HANDLE hNextColorFrameEvent;
		HRESULT hr;

		int iSensorCount = 0;
		hr = NuiGetSensorCount(&iSensorCount);
		if (FAILED(hr))
		{
			cerr << "No NuiSensor not found" << endl;
			return hr;
		}

		// Look at each Kinect sensor
		for (int i = 0; i < iSensorCount; ++i)
		{
			// Create the sensor so we can check status, if we can't create it, move on to the next
			hr = NuiCreateSensorByIndex(i, sensor);
			if (FAILED(hr))
			{
				continue;
			}

			// Get the status of the sensor, and if connected, then we can initialize it
			hr = (*sensor)->NuiStatus();
			if (S_OK == hr)
			{
				break;
			}

			// This sensor wasn't OK, so release it since we're not using it
			(*sensor)->Release();
		}

		if (nullptr == (*sensor))
		{
			cerr << "NuiSensor not created" << endl;
			return hr;
		}

		// Initialize the Kinect and specify that we'll be using color
		hr = (*sensor)->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR);
		if (FAILED(hr))
		{
			return hr;
		}

		// Create an event that will be signaled when color data is available
		hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

		// Open a color image stream to receive color frames
		hr = (*sensor)->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
			resolution,
			0 /* dwImageFrameFlags */,
			2 /* dwFrameLimit */,
			hNextColorFrameEvent,
			hColorStreamHandle);

		if (FAILED(hr)) {
			std::cerr << "Error : NuiImageStreamOpen\n";
			return hr;
		}

		return hr;
	}

public:

	void setInputFile(char* path) {
		string inputImage = string(path);
		// TODO: this is just test
		input = imread(inputImage, CV_LOAD_IMAGE_COLOR);
	}

	//
	// First Approach. Fix orig with add mask
	// 
	void addingMask(const Mat& o, const Mat& s, Mat& d) {
		// Clone surface
		Mat mask = s.clone();

		// Invert surface = MASK
		bitwise_not(mask, mask);

		// Add to Projection
		add(input, mask, d);
	}

	//
	// Second Approach: calculate the right color by divide
	//
	void division(const Mat& o, const Mat& s, const Mat& d)
	{
		for (int y = 0; y < o.rows; ++y)
		{
			for (int x = 0; x < o.cols; ++x)
			{
				for (int c = 0; c < 3; ++c)
				{
					unsigned char oPx = o.data[y * o.step + x * 3 + c];
					unsigned char sPx = s.data[y * s.step + x * 3 + c];

					// Non Zero division
					float fsPx = (sPx == 0) ? 0.00001 : (float)sPx;
					float s = ((float)oPx) / fsPx;

					unsigned char r = (s > 1.0 ? 1.0 : s) * 255;

					d.data[y * d.step + 3 * x + c] = r;
				}
			}
		}
	}

	// Main entry point for color correction
	bool correction(Mat& src, Mat& redSurface) {
		if (!src.data)                              // Check for invalid input
		{
			cout << "Could not open or find the src image" << std::endl;
			return false;
		}

		if (!redSurface.data)                              // Check for invalid input
		{
			cout << "Could not open or find the red surface" << std::endl;
			return false;
		}

		if (!input.data)                              // Check for invalid input
		{
			cout << "Could not open or find the orig image" << std::endl;
			return false;
		}

		int k = waitKey(30);

		// Detect the surface
		Mat surface;
		while (!detectSurface(src, redSurface, surface)) {
			return false;
		}

		resize(input, input, Size(screenWidth, screenHeight));

		// Approach 1.
		Mat fix1(screenHeight, screenWidth, CV_8UC3, Scalar(255, 255, 255));
		addingMask(input, surface, fix1);

		// Approach 2. Division.
		Mat fix2(screenHeight, screenWidth, CV_8UC3, Scalar(255, 255, 255));
		division(input, surface, fix2);

		imshow("Fullscreen", fix1);

		double alpha[2] = { 1.0, 1.0 };
		int beta[2] = { 0, 0 };

		bool changed = false;
		int lastImg = 1;

		Mat changedSurface = surface.clone();

		// ECS = Exit App
		while (k != 27) {
			k = waitKey(30);

			if (changed && lastImg != 0) {
				cout << "Approach:" << lastImg << " - alpha =" << alpha[lastImg-1] << "; beta = " << beta[lastImg-1] << endl;
				changed = false;

				surface.convertTo(changedSurface, -1, alpha[lastImg-1], beta[lastImg-1]);
				if (lastImg == 1)
					addingMask(input, changedSurface, fix1);
				if (lastImg == 2)
					division(input, changedSurface, fix2);
			}

			// Key = 1
			if (lastImg == 1 || k == 49) {
				imshow("Fullscreen", fix1);
				lastImg = 1;
			}

			// Key = 2
			if (lastImg == 2 || k == 50) {
				imshow("Fullscreen", fix2);
				lastImg = 2;
			}

			// Key = 0 Original Image
			if (lastImg == 0 || k == 48) {
				imshow("Fullscreen", input);
				lastImg = 0;
			}

			// Adjust Contrast and Brightness
			switch (k) {
			case 'u': if (lastImg != 0) { alpha[lastImg-1] -= 0.1; changed = true; } break;
			case 'j': if (lastImg != 0) { alpha[lastImg-1] += 0.1; changed = true; } break;

			case 'i': if (lastImg != 0) { beta[lastImg-1] -= 5;  changed = true; } break;
			case 'k': if (lastImg != 0) { beta[lastImg-1] += 5;  changed = true; } break;
			}

			// Save File
			if (k == 13) {
				stringstream fileName;
				fileName << "Images/surface_" << lastImg << ".jpg";
				imwrite(fileName.str(), changedSurface);
			}
		}

		return true;
	}

	// Manual surface
	bool manualSurface(Mat& src, vector<Point>& points) {
		//Create a window
		namedWindow("Draw", 1);

		Mat draw;
		src.copyTo(draw);
		//set the callback function for any mouse event
		setMouseCallback("Draw", callBackFunc, &points);

		unsigned int lastSize = 0;
		int k = waitKey(30);
		while (k != 27) {
			k = waitKey(30);
			// Points size changed?
			if (lastSize != points.size()) {
				src.copyTo(draw);
				lastSize = points.size();
				for (unsigned int i = 0; i < points.size(); i++) {
					// Draw contour line
					line(draw, points[i], points[i == points.size() - 1 ? 0 : i + 1], Scalar(0, 0, 255), 1, CV_AA, 0);
				}
			}

			imshow("Draw", draw);

			if (k == 13 && points.size() == 4) { // Enter
				destroyWindow("Draw");
				return true;
			}
		}

		return false;
	}


	// 
	// Detect projector surface.
	// Keystone korrection.
	// https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
	// https://github.com/bsdnoobz/opencv-code/blob/master/quad-segmentation.cpp
	// 
	bool detectSurface(Mat& src, Mat& redSurface, Mat& surface) {

		vector<Point> contours_poly;

		if (!findSurfaceContours(src, redSurface, contours_poly)) {
			// User exits
			return false;
		}

		adjustPerspective(src, surface, contours_poly);

		// Project surface back
		imshow("Fullscreen", surface);

		int k = waitKey(30);
		while (k != 27) { // ECS - exit
			k = waitKey(30);
			if (k == 13) return true; // Enter - Accepts Surface
			if (k == 'r') { // R - change surface manual
				if (manualSurface(src, contours_poly)) {
					adjustPerspective(src, surface, contours_poly);
					imshow("Fullscreen", surface);
				}
			}
		}

		return false;
	}


	// Open White Fullscreen
	void openWhiteFullscreen() {
		namedWindow("Fullscreen", CV_WINDOW_NORMAL);

		int posX = 0;
		int posY = 0;
		getDesktopResolution(screenWidth, screenHeight, posX, posY);

		Mat white(screenWidth, screenHeight, CV_8UC3, Scalar(255, 255, 255));

		// Move window to another monitor
		moveWindow("Fullscreen", posX, posY);
		setWindowProperty("Fullscreen", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		imshow("Fullscreen", white);
	}

	// Get From Video Stream
	int fromVideoStream(bool kinect) {
		// Kinect
		HANDLE hColorStreamHandle = NULL;
		INuiSensor* pNuiSensor = nullptr;

		// Video camera
		VideoCapture cap;

		if (kinect) {
			initKinect(&pNuiSensor, &hColorStreamHandle);
			if (pNuiSensor == nullptr || hColorStreamHandle == NULL) {
				cerr << "Error: Failed to open Kinect camera" << endl;
				return E_FAIL;
			}
		}
		else {
			cap.open(0);
			if (!(cap.isOpened())) {
				cerr << "Error : VideoCapture open camera" << endl;
				return E_FAIL;
			}
		}

		openWhiteFullscreen();

		// Preview
		namedWindow("Camera", cv::WINDOW_AUTOSIZE);

		bool showRedScreen = false;
		bool captureRedAndWhite = false;
		bool captureFinish = false;

		Mat red(screenWidth, screenHeight, CV_8UC3, Scalar(0, 0, 0));
		cv::copyMakeBorder(red, red, 60, 60, 20, 20, BORDER_CONSTANT, Scalar(0, 0, 255));
		Mat white(screenWidth, screenHeight, CV_8UC3, Scalar(255, 255, 255));

		Mat frame, src, redSurface;

		int k = waitKey(30);
		while (k != 27) {

			if (kinect) // get a new frame from Kinect
				frame = nextKinectFrame(pNuiSensor, hColorStreamHandle);
			else // get a new frame from camera
				cap >> frame;

			// open camera view
			imshow("Camera", frame);

			k = waitKey(30);
			if (k == 13 && !showRedScreen) { // Enter
				showRedScreen = true;
			}
			else if (!captureRedAndWhite && k == 13) {
				showRedScreen = false;
				captureRedAndWhite = true;
			}

			if (showRedScreen) {
				imshow("Fullscreen", red);
			}
			else if (captureRedAndWhite) {
				// Store the red surface
				//cap >> frame;
				frame.copyTo(redSurface);

				imshow("Fullscreen", white);
				k = waitKey(300);

				// Get Next Frame
				if (kinect) {// get a new frame from Kinect
					frame = nextKinectFrame(pNuiSensor, hColorStreamHandle);
					frame = nextKinectFrame(pNuiSensor, hColorStreamHandle);
					// Kinect has alpha channel
					cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
				}
				else // get a new frame from camera
					cap >> frame;

				frame.copyTo(src);

				destroyWindow("Camera");
				if (correction(src, redSurface)) {
					break;
				}
				else {
					showRedScreen = false;
					captureRedAndWhite = false;
				}
			}
		}

		return 0;
	}

	// Get From File (Test)
	void fromFile() {
		Mat redSurface = imread("Images/02_tr.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
		Mat src = imread("Images/02_tt.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file

		openWhiteFullscreen();
		correction(src, redSurface);
	}

};

// Grabed from here http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
	char ** itr = std::find(begin, end, option);
	if (itr != end && ++itr != end)
	{
		return *itr;
	}
	return 0;
}

// Grabed from here http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
	return std::find(begin, end, option) != end;
}

void help() {
	cout << "Kicc - Kinect Color Correction" << endl;
	cout << "usage: ./kicc -i=string [options]" << endl;
	cout << "options:" << endl;
	cout << "-i input path" << endl;
	cout << "-c Video Camera" << endl;
	cout << "-f Test Files" << endl;
	cout << "-f this message" << endl;
}

int main(int argc, char** argv) {
	// c = Camera
	// f = file
	// k = kinect

	Kicc kicc;

	// Debug code
	char code = 'k';

	if (cmdOptionExists(argv, argv + argc, "-h"))
	{
		help();
	}

	char* input = getCmdOption(argv, argv + argc, "-i");
	if (input)
	{
		kicc.setInputFile(input);
	}
	else
	{
		// Load per default the Jap Girl
		kicc.setInputFile("Images/jap_o.jpg");
	}

	if (cmdOptionExists(argv, argv + argc, "-f") || code == 'f')
	{
		kicc.fromFile();
	}
	else if (cmdOptionExists(argv, argv + argc, "-c") || code == 'c')
	{
		kicc.fromVideoStream(false /* No Kinect */);
	}
	else
	{
		// Default, Try open stream from Kinect
		kicc.fromVideoStream(true /* Kinect */);
	}

	while (waitKey() != 27) continue;

	return 0;
}
