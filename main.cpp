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

int FromVideoStream(bool kinect);
void fromFile();

bool DetectSurface(Mat& src, Mat& redSurface, Mat& surface);

int width = 640; // Kinect v1 height = 640
int height = 480; // Kinect v1 width =  480

int screenWidth = 640; //1600;
int screenHeight = 480; //1200;

unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

Mat orig;

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


//
// Fix orig with add invert mask
// 
void AddingMask(const Mat& o, const Mat& s, const Mat& d) {
	// Invert the mask
	Mat mask = s.clone();
	// invert mask
	bitwise_not(mask, mask);

	add(orig, mask, d);
}

//
// Second Approach: calculate the right color by divide
//
void Division(const Mat& o, const Mat& s, const Mat& d)
{
	for (int y = 0; y < o.size().height; ++y)
	{
		for (int x = 0; x < o.size().width; ++x)
		{	
			for (int c = 0; c < o.channels(); ++c)
			{
				unsigned char oPx = o.data[y * o.step + x * o.channels() + c];
				unsigned char sPx = s.data[y * s.step + x * s.channels() + c];

				// No Zero division
				if (sPx == 0) sPx = 1;
				float s = ((float)oPx) / ((float)sPx);
				// unsigned short s = (oPx * 255) / sPx;

				unsigned char r = (s > 1.0 ? 1.0 : s) * 255;

				d.data[y * d.step + d.channels() * x + c] = r;
			}
		}
	}
}

bool ColorCorrection(Mat& src, Mat& redSurface) {

	Mat surface;
	if (!DetectSurface(src, redSurface, surface)) {
		// No surface found
		return false;
	}

	// TODO: this is just test
	orig = imread("Images/jap_o.jpg", CV_LOAD_IMAGE_COLOR);

	// Resize surface and orig to Window Size
	resize(surface, surface, Size(screenWidth, screenHeight));
	resize(orig, orig, Size(screenWidth, screenHeight));

	// Approach 1
	Mat fix1(screenHeight, screenWidth, CV_8UC3, Scalar(255, 255, 255));
	AddingMask(orig, surface, fix1);

	// 2 approach. Division.
	Mat fix2(screenHeight, screenWidth, CV_8UC3, Scalar(255, 255, 255));
	Division(orig, surface, fix2);

	imshow("Fullscreen", fix1);
	moveWindow("Fullscreen", 0, 0);

	int k = waitKey(30);
	while (k != 27) {
		k = waitKey(30);
		if (k == 's') {
			imshow("Fullscreen", fix1);
		}

		if (k == 'd') {
			imshow("Fullscreen", fix2);
		}
	}

	return true;
}

// 
// Detect projector surface.
// Keystone korrection.
// https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
// https://github.com/bsdnoobz/opencv-code/blob/master/quad-segmentation.cpp
// 
bool DetectSurface(Mat& src, Mat& redSurface, Mat& surface) {
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


	// Reorder Points
	Point t1, t2, b1, b2;
	for (int i = 0; i < 4; i++) {
		if (contours_poly[i].y >= t1.y && contours_poly[i] != t2)
			t1 = contours_poly[i];
		if (contours_poly[i].y >= t2.y && contours_poly[i] != t1)
			t2 = contours_poly[i];
	}

	for (int i = 0; i < 4; i++) {
		if (contours_poly[i] != t1 && contours_poly[i] != t2 && b1 != b2)
			b1 = contours_poly[i];
		if (contours_poly[i] != t1 && contours_poly[i] != t2 && contours_poly[i] != b1)
			b2 = contours_poly[i];
	}

	Point tl, tr, br, bl;
	if (t1.x < t2.x) { tl = t1; tr = t2; }
	else { tl = t2; tr = t1; }
	if (b1.x < b2.x) { bl = b1; br = b2; }
	else { bl = b2; br = b1; }

	/** Order WTF
	2st-------4nd
	|         |
	|         |
	|         |
	1rd-------3th
	*/
	Size s = src.size();
	vector<Point2f> quad_pts  { Point2f(bl.x, bl.y), Point2f(tl.x, tl.y),  Point2f(br.x, br.y), Point2f(tr.x, tr.y) };
	vector<Point2f> squre_pts { Point2f(0, 0),       Point2f(0, s.height), Point2f(s.width, 0), Point2f(s.width, s.height) };

	Mat transmtx = getPerspectiveTransform(quad_pts, squre_pts);
	warpPerspective(src, surface, transmtx, s);

	// Draw contour line
	line(src, contours_poly[0], contours_poly[1], Scalar(0, 0, 255), 1, CV_AA, 0);
	line(src, contours_poly[1], contours_poly[2], Scalar(0, 0, 255), 1, CV_AA, 0);
	line(src, contours_poly[2], contours_poly[3], Scalar(0, 0, 255), 1, CV_AA, 0);
	line(src, contours_poly[3], contours_poly[0], Scalar(0, 0, 255), 1, CV_AA, 0);

	// Draw rectangle
	rectangle(src, boundingRect(contour), Scalar(0, 255, 0), 1, 8, 0);

	return true;
}


// Open White Fullscreen
void OpenWhiteFullscreen() {
	namedWindow("Fullscreen", CV_WINDOW_NORMAL);
	setWindowProperty("Fullscreen", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	GetDesktopResolution(screenWidth, screenHeight);

	Mat white(screenWidth, screenHeight, CV_8UC3, Scalar(255, 255, 255));
	imshow("Fullscreen", white);
}

Mat NextKinectFrame(INuiSensor* pNuiSensor, HANDLE hColorStreamHandle) {
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

HRESULT
InitKinect(INuiSensor** sensor, HANDLE* hColorStreamHandle) {

	NUI_IMAGE_RESOLUTION resolution = NUI_IMAGE_RESOLUTION_640x480;

	// If we want the width
	unsigned long widthTmp, heightTmp;
	NuiImageResolutionToSize(resolution, widthTmp, heightTmp);
	UINT width = static_cast<UINT>(widthTmp);
	UINT height = static_cast<UINT>(heightTmp);

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


int FromVideoStream(bool kinect) {
	// Kinect
	HANDLE hColorStreamHandle = NULL;
	INuiSensor* pNuiSensor = nullptr;

	// Video camera
	VideoCapture cap;

	if (kinect) {
		InitKinect(&pNuiSensor, &hColorStreamHandle);
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
	
	OpenWhiteFullscreen();

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
		
		if(kinect) // get a new frame from Kinect
		  frame = NextKinectFrame(pNuiSensor, hColorStreamHandle);
		else // get a new frame from camera
		  cap >> frame;
	
		// open camera view
		imshow("Camera", frame);

		k = waitKey(30);
		if (k == 13) { // Enter
			showRedScreen = true;
		}
		else if (!captureRedAndWhite && k == 'c') {
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
				frame = NextKinectFrame(pNuiSensor, hColorStreamHandle);
				// Kinect has alpha channel
				cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
			}
			else // get a new frame from camera
				cap >> frame;

			frame.copyTo(src);

			if (ColorCorrection(src, redSurface)) {
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

void FromFile() {
	Mat redSurface = imread("Images/01_tr.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
	Mat src = imread("Images/01_tt.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file

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

	ColorCorrection(src, redSurface);
}


int main(int argc, char** argv) {
	// c = Camera
	// f = file
	// k = kinect

	char code = 'f';
	bool d = false;

	// Open From
	switch (code) {
	case 'k': FromVideoStream(true /* Kinect */); break;
	case 'c': FromVideoStream(false); break;
	case 'f': FromFile(); break;
	}

	
	while (waitKey() != 27) continue;

	return 0;
}
