#include <iostream>
#include <opencv2/opencv.hpp>
#include "Windows.h"
#include "NuiApi.h"


using namespace cv;
using namespace std;

// OpenCV and VS 2015: http://dogfeatherdesign.com/opencv-3-0-microsoft-visual-studio-2015-cmake-and-c/

// Example http://answers.opencv.org/question/54785/face-detect-kinect-v2-opencv/
// https://msdn.microsoft.com/en-us/library/dn791996.aspx

// Kinect with CS: https://msdn.microsoft.com/en-us/library/hh855360.aspx
// OpenCV: http://docs.opencv.org/2.4/doc/tutorials/introduction/windows_visual_studio_Opencv/windows_visual_studio_Opencv.html
// Kinect C++ Reference https://msdn.microsoft.com/en-us/library/hh855364.aspx

int fromKinect();
void fromCamera();
void fromFile();

bool detectSurface();
void invert(Mat mat);


int width = 640; // Kinect v1 height = 640
int height = 480; // Kinect v1 width =  480

int screenWidth = 1600;
int screenHeight = 1200;

unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

Mat src;
Mat redSurface;
Mat orig;

Mat mask;

int main(int argc, char** argv) {
	// c = Camera
	// f = file
	// k = kinect

	char code = 'k'; 
	bool d = false;

	switch (code) {
	case 'k': fromKinect(); break;
	case 'c': fromCamera(); break;
	case 'f': fromFile(); break;
	}

	while (waitKey() != 27) continue;

	return 0;
}


/**
* Helper function to display text in the center of a contour
*/
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}


// https://gist.github.com/DaniilTomilow/1088bca80f5a1f449f15
void GetDesktopResolution(int& width, int& height)
{
	DEVMODE mode;
	mode.dmSize = sizeof(mode);
	EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &mode);

	width = mode.dmPelsWidth;
	height = mode.dmPelsHeight;

	cout << "5: Width: " << width << "; height: " << height << '\n';
}

struct MonitorInfo {
	unsigned monitor = 0;
	unsigned primary = 0;
	unsigned index = 0;
	unsigned width;
	unsigned height;
	string displayName;
};

static BOOL CALLBACK 
MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
	MonitorInfo& info = *(MonitorInfo*) dwData;
	MONITORINFOEX mi;
	memset(&mi, 0, sizeof(MONITORINFOEX));
	mi.cbSize = sizeof(MONITORINFOEX);

	GetMonitorInfo(hMonitor, &mi);
	string displayName = (const char*)mi.szDevice;
	if (displayName.find(R"(\\.\DISPLAYV)") == 0) return TRUE;  //ignore pseudo-monitors
	if (mi.dwFlags & MONITORINFOF_PRIMARY) info.primary = info.index;
	if (info.monitor == info.index) {
		info.width = lprcMonitor->right - lprcMonitor->left;
		info.height = lprcMonitor->bottom - lprcMonitor->top;
		info.displayName = displayName;
	}

	info.index++;
	return TRUE;
}

void GetDesktopResolution6(unsigned monitor) {
	MonitorInfo info;
	info.monitor = monitor;

	EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, (LPARAM)&info);

	cout << "Name:" << info.displayName << "; Width: " << info.width << "; height " << info.height << endl;
}

/**
* Detect projector surface.
* Keystone korrection.
* https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
* https://github.com/bsdnoobz/opencv-code/blob/master/quad-segmentation.cpp
*/
bool detectSurface() {
	// medianBlur(bgr_image, bgr_image, 3);

	// Convert to HSV
	Mat hsv_image;
	cvtColor(redSurface, hsv_image, cv::COLOR_BGR2HSV);

	Mat lower_red_hue_range, upper_red_hue_range;
	inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

	// Combine the above two images
	addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, redSurface);

	//Mat blur;
	//GaussianBlur(mask, blur, cv::Size(7, 7), 1.5, 1.5);
	Mat tre;
	threshold(redSurface, tre, 180, 255, CV_THRESH_BINARY);

	Mat canny;
	Canny(tre, canny, 0, 50, 5);

	vector<vector<Point>> contours;
	findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	if (contours.size() == 0)
		return false;

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

	// Approximate contour with accuracy proportional
	// to the contour perimeter
	vector<Point> contours_poly;
	approxPolyDP(Mat(contour), contours_poly, arcLength(Mat(contour), true) * 0.02, true);

	// Number of vertices of polygonal curve
	int vtc = contours_poly.size();

	// Skip small or non-convex objects 
	if (vtc != 4 || fabs(contourArea(contour)) < 100 || !isContourConvex(contours_poly)) {
		std::cerr << "No projectile surface found" << std::endl;
		return false;
	}

	vector<Point2f> quad_pts;
	vector<Point2f> squre_pts;
	quad_pts.push_back(Point2f(contours_poly[0].x, contours_poly[0].y));
	quad_pts.push_back(Point2f(contours_poly[1].x, contours_poly[1].y));
	quad_pts.push_back(Point2f(contours_poly[3].x, contours_poly[3].y));
	quad_pts.push_back(Point2f(contours_poly[2].x, contours_poly[2].y));

	Size s = src.size();
	squre_pts.push_back(Point2f(0, 0));
	squre_pts.push_back(Point2f(0, s.height));
	squre_pts.push_back(Point2f(s.width, 0));
	squre_pts.push_back(Point2f(s.width, s.height));

	Mat transmtx = getPerspectiveTransform(quad_pts, squre_pts);
	warpPerspective(src, mask, transmtx, s);

	Point P1 = contours_poly[0];
	Point P2 = contours_poly[1];
	Point P3 = contours_poly[2];
	Point P4 = contours_poly[3];

	line(src, P1, P2, Scalar(0, 0, 255), 1, CV_AA, 0);
	line(src, P2, P3, Scalar(0, 0, 255), 1, CV_AA, 0);
	line(src, P3, P4, Scalar(0, 0, 255), 1, CV_AA, 0);
	line(src, P4, P1, Scalar(0, 0, 255), 1, CV_AA, 0);
	Rect boundRect = boundingRect(contour);
	rectangle(src, boundRect, Scalar(0, 255, 0), 1, 8, 0);

	resize(mask, mask, Size(screenWidth, screenHeight));

	invert(mask);

	int k = waitKey(30);

	//imshow("Result", result);
	imshow("Fullscreen", mask);
	imshow("White Surface", src);

	while (k != 27) {
		k = waitKey(30);
	}

	return true;
}

void invert(Mat mat) {
	// 1. Invert Mask
	subtract(Scalar::all(255), mat, mat);
}

// Open White Fullscreen
void OpenWhiteFullscreen() {
	namedWindow("Fullscreen", CV_WINDOW_NORMAL);
	setWindowProperty("Fullscreen", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	GetDesktopResolution(screenWidth, screenHeight);

	Mat white(screenWidth, screenHeight, CV_8UC3, Scalar(255, 255, 255));
	imshow("Fullscreen", white);
}

void CaptureScreen() {

	waitKey(30);
}

int fromKinect() {
	cv::setUseOptimized(true);

	INuiSensor* pNuiSensor = nullptr;
	NUI_IMAGE_RESOLUTION resolution = NUI_IMAGE_RESOLUTION_640x480;

	HANDLE hNextColorFrameEvent;
	HANDLE hColorStreamHandle = NULL;
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
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (nullptr == pNuiSensor)
	{
		cerr << "NuiSensor not created" << endl;
		return E_FAIL;
	}

	// Initialize the Kinect and specify that we'll be using color
	hr = pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR);
	if (FAILED(hr))
	{
		return E_FAIL;
	}
		// Create an event that will be signaled when color data is available
	hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	// Open a color image stream to receive color frames
	hr = pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		resolution,
		0,
		2,
		hNextColorFrameEvent,
		&hColorStreamHandle);

	if (FAILED(hr)) {
		std::cerr << "Error : NuiImageStreamOpen\n";
	}

	unsigned long widthTmp, heightTmp;
	NuiImageResolutionToSize(resolution, widthTmp, heightTmp);
	UINT width = static_cast<UINT>(widthTmp);
	UINT height = static_cast<UINT>(heightTmp);


	while (1) {
		NUI_IMAGE_FRAME nuiImage;
		hr = pNuiSensor->NuiImageStreamGetNextFrame(hColorStreamHandle, 1000, &nuiImage);
		if (FAILED(hr)) {
			std::cerr << "Error : NuiImageStreamGetNextFrame\n";
		}

		INuiFrameTexture* texture = nuiImage.pFrameTexture;
		NUI_LOCKED_RECT rect;
		texture->LockRect(0, &rect, nullptr, 0);
		cv::Mat cvImage(height, width, CV_8UC4, reinterpret_cast<unsigned char*>(rect.pBits));

		cv::imshow("", cvImage);
		cv::waitKey(30);

		texture->UnlockRect(0);
		pNuiSensor->NuiImageStreamReleaseFrame(hColorStreamHandle, &nuiImage);
	}
}

void fromCamera() {

	//OpenNI::initialize();
	// http://docs.opencv.org/master/d8/dfe/classcv_1_1VideoCapture.html
	VideoCapture cap(CV_CAP_OPENNI); // Open Kinect Camera
	if (!(cap.isOpened())) {
		//cap.open(0); // open the default camera
		//if (!cap.isOpened())  // check if we succeeded
	//	{
			std::cerr << "Error : VideoCapture open camera" << std::endl;
			return;
	//	}
	}
	
	namedWindow("Camera", cv::WINDOW_AUTOSIZE);

	bool showRedScreen = false;
	bool captureRedAndWhite = false;
	bool captureFinish = false;

	int k = waitKey(30);

	Mat red(screenWidth, screenHeight, CV_8UC3, Scalar(255, 255, 255));
	cv::copyMakeBorder(red, red, 50, 40, 40, 40, BORDER_CONSTANT, Scalar(0,0,255));
	Mat white(screenWidth, screenHeight, CV_8UC3, Scalar(255, 255, 255));

	while(k != 27) // ECS
	{
		Mat frame;
		cap >> frame; // get a new frame from camera

		cv::imshow("Camera", frame);
	
		k = waitKey(30);

		if (k == 13) { // Enter
			showRedScreen = true;
		}
		else if (!captureRedAndWhite && k == 'c') {
			showRedScreen = false;
			captureRedAndWhite = true;
		}

		if (showRedScreen) {
			cv::imshow("Fullscreen", red);

		} else if (captureRedAndWhite) {
			// Store the red surface
			cap >> frame; 
			frame.copyTo(redSurface);

			imshow("Fullscreen", white);
			k = waitKey(300);

			// Get Next Frame
			cap >> frame;
			cap >> frame;

			frame.copyTo(src);

			break;
		}
	}

	detectSurface();
}

void fromFile() {
	redSurface = imread("Images/test1_r.png", CV_LOAD_IMAGE_COLOR);   // Read the file
	src = imread("Images/test1.png", CV_LOAD_IMAGE_COLOR);   // Read the file
	orig = imread("Images/jap_o.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file

	if (!src.data)                              // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return;
	}

	if (!orig.data)                              // Check for invalid input
	{
		cout << "Could not open or find the orig image" << std::endl;
		return;
	}

	detectSurface();

}
