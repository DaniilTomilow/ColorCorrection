#include <iostream>

#include <Kinect.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// OpenCV and VS 2015: http://dogfeatherdesign.com/opencv-3-0-microsoft-visual-studio-2015-cmake-and-c/

// Example http://answers.opencv.org/question/54785/face-detect-kinect-v2-opencv/
// https://msdn.microsoft.com/en-us/library/dn791996.aspx

// Kinect with CS: https://msdn.microsoft.com/en-us/library/hh855360.aspx
// OpenCV: http://docs.opencv.org/2.4/doc/tutorials/introduction/windows_visual_studio_Opencv/windows_visual_studio_Opencv.html
// Kinect C++ Reference https://msdn.microsoft.com/en-us/library/hh855364.aspx

bool fromCamera();
bool fromFile();
bool fromKinect();

bool detectSurface();
void correct();
bool exec(char code);

int width = 640; // Kinect v1 height = 640
int height = 480; // Kinect v1 width =  480
unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

Mat src;

Mat mask;

int main(int argc, char** argv) {
	// c = Camera
	// f = file
	// k = kinect

	char code = 'f'; 
	bool d = false;

	while (!d) {
		if (exec(code)) {
			d = detectSurface();
		}
	}
	
	return 0;
}


bool exec(char code) {
	bool cmd = false;

	switch (code) {
	case 'c': cmd = fromCamera(); break;
	case 'k': cmd = fromKinect(); break;
	case 'f': cmd = fromFile(); break;
	}

	if (!cmd) {
		cerr << "Could not open or find the image" << std::endl;
		return false;
	}

	return true;
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
	cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);

	Mat lower_red_hue_range, upper_red_hue_range;
	inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

	// Combine the above two images
	addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, mask);

	//cvtColor(red_hue_image, mask, cv::COLOR_HSV);
	//Mat blur;
	//GaussianBlur(mask, blur, cv::Size(7, 7), 1.5, 1.5);
	Mat tre;
	threshold(mask, tre, 180, 255, CV_THRESH_BINARY);

	Mat canny;
	Canny(tre, canny, 0, 50, 5);

	vector<vector<Point>> contours;
	findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	if (contours.size() == 0)
		return false;

	Mat dst;
	src.copyTo(dst);

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

	Rect boundRect = boundingRect(contour);
	vector<Point2f> quad_pts;
	vector<Point2f> squre_pts;
	quad_pts.push_back(Point2f(contours_poly[0].x, contours_poly[0].y));
	quad_pts.push_back(Point2f(contours_poly[1].x, contours_poly[1].y));
	quad_pts.push_back(Point2f(contours_poly[3].x, contours_poly[3].y));
	quad_pts.push_back(Point2f(contours_poly[2].x, contours_poly[2].y));

	squre_pts.push_back(Point2f(0, 0));
	squre_pts.push_back(Point2f(0, height));
	squre_pts.push_back(Point2f(width, 0));
	squre_pts.push_back(Point2f(width, height));

	Mat transmtx = getPerspectiveTransform(quad_pts, squre_pts);
	warpPerspective(src, mask, transmtx, src.size());

	Point P1 = contours_poly[0];
	Point P2 = contours_poly[1];
	Point P3 = contours_poly[2];
	Point P4 = contours_poly[3];

	line(dst, P1, P2, Scalar(0, 0, 255), 1, CV_AA, 0);
	line(dst, P2, P3, Scalar(0, 0, 255), 1, CV_AA, 0);
	line(dst, P3, P4, Scalar(0, 0, 255), 1, CV_AA, 0);
	line(dst, P4, P1, Scalar(0, 0, 255), 1, CV_AA, 0);

	rectangle(dst, boundRect, Scalar(0, 255, 0), 1, 8, 0);

	int width = 0;
	int height = 0;
	GetDesktopResolution(width, height);

	namedWindow("Mask", CV_WINDOW_NORMAL);
	setWindowProperty("Mask", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	resize(mask, mask, cv::Size(width, height));

	imshow("Mask", mask);
	imshow("dst", dst);

	correct();

	waitKey(0);

	return true;
}

void correct() {
	// 1. Invert Mask
	subtract(Scalar::all(255), mask, mask);
}


bool fromCamera() {
	// http://docs.opencv.org/master/d8/dfe/classcv_1_1VideoCapture.html
	cv::VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
	{
		std::cerr << "Error : VideoCapture open default camera" << std::endl;
		return false;
	}

	cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
	int t = 7;
	while(true)
	{
		Mat frame;
		Mat g;
		cap >> frame; // get a new frame from camera

		imshow("Camera", frame);
	
		int k = cv::waitKey(30);

		if (k == 27) break; // ECS
		else if (k == 'c') {
			resize(frame, src, cv::Size(width, height));
			imwrite("Images/test2.png", src);
			destroyWindow("Camera");
			return true;
		}
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return false;
}

bool fromFile() {
	src = imread("Images/test1.png", CV_LOAD_IMAGE_COLOR);   // Read the file

	if (!src.data)                              // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return false;
	}

	return true;
}

// Get a working kinect sensor
bool fromKinect() {
	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	//while (1)
	//{
		// Color Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(src.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
	//}
}