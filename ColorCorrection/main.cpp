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

void detectSurface();

int width = 640; // Kinect v1 height = 640
int height = 480; // Kinect v1 width =  480
unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

Mat orig;
Mat mask;


int main(int argc, char** argv) {
	// c = Camera
	// f = file
	// k = kinect

	char code = 'f'; // k, f
	bool cmd = false;

	switch (code) {
	case 'c': cmd = fromCamera(); break;
	case 'k': cmd = fromKinect(); break;
	case 'f': cmd = fromFile(); break;
	}

	if (!cmd) {
		cerr << "Could not open or find the image" << std::endl;
		return 0;
	}

	detectSurface();

	return 0;
}

/**
* Helper function to find a cosine of angle between vectors
* from pt0->pt1 and pt0->pt2
*/
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
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

/**
* Detect projector surface.
* Keystone korrection.
* https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
*/
void detectSurface() {
	// Convert to grayscale
	cvtColor(orig, mask, cv::COLOR_BGR2GRAY);
	//Mat blur;
	//GaussianBlur(mask, blur, cv::Size(7, 7), 1.5, 1.5);
	Mat canny;
	Canny(mask, canny, 0, 50, 5);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/// Find contours
	findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<Point> approx;
	
	Mat dst(orig);
	Mat rotated;

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
			continue;

		// Number of vertices of polygonal curve
		int vtc = approx.size();

		if (vtc == 4)
		{
			// Get the cosines of all corners
			vector<double> cos;
			for (int j = 2; j < vtc + 1; j++) {
				cos.push_back(angle(approx[j%vtc], approx[j - 2], approx[j - 1]));
			}

			// Sort ascending the cosine values
			sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();
			
			//iterating through each point
			/*
			1st-------2nd
            |         |
            |         |
            |         |
            3rd-------4th
			*/
			vector<Point> shape;
			for (int i = 0; i < 4; i++) {
				shape.push_back(approx.at(i));
			}

			//drawing lines around the quadrilateral
			line(dst, shape.at(0), shape.at(1), Scalar(255, 0, 0), 4);
			line(dst, shape.at(1), shape.at(2), Scalar(0, 255, 0), 4);
			line(dst, shape.at(2), shape.at(3), Scalar(255, 255, 255), 4);
			line(dst, shape.at(3), shape.at(0), Scalar(0, 0, 255), 4);

			setLabel(dst, "RECT", contours[i]);

			// Assemble a rotated rectangle out of that info
			RotatedRect box = minAreaRect(Mat(shape));
			std::cout << "Rotated box set to (" << box.boundingRect().x << "," << box.boundingRect().y << ") " << box.size.width << "x" << box.size.height << std::endl;

			Point2f pts[4];
			box.points(pts);

			// topLeft, topRight, bottomRight, bottomLeft?
			Point2f src_vertices[3];
			src_vertices[0] = pts[2];
			src_vertices[1] = pts[3];

			src_vertices[2] = pts[1];
			src_vertices[3] = pts[0];

			Point2f dst_vertices[3];
			dst_vertices[0] = Point(0, 0);
			dst_vertices[1] = Point(box.boundingRect().width - 1, 0);
			dst_vertices[2] = Point(0, box.boundingRect().height - 1);
			dst_vertices[3] = Point(box.boundingRect().width - 1, box.boundingRect().height - 1);

			Mat warpAffineMatrix = getAffineTransform(src_vertices, dst_vertices);

			Size size(box.boundingRect().width, box.boundingRect().height);
			warpAffine(orig, rotated, warpAffineMatrix, size, INTER_LINEAR, BORDER_CONSTANT);
			
			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if(mincos >= -0.1 && maxcos <= 0.3) {
				setLabel(dst, "NO SURFACE", contours[i]);
			}
		}
		else
		{
			setLabel(dst, "NO SURFACE", contours[i]);
		}
	}

	imshow("dst", dst);
	imshow("rotated", rotated);

	waitKey(0);
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
	while(true)
	{
		cv::Mat frame;
		cap >> frame; // get a new frame from camera
		cv::imshow("Camera", frame);
		int k = cv::waitKey(30);

		if (k == 27) break; // ECS
		else if (k == 'c') {
			cv::resize(frame, orig, cv::Size(width, height));
			cv::destroyWindow("Camera");
			return true;
		}
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return false;
}

bool fromFile() {
	orig = imread("Images/test1.png", CV_LOAD_IMAGE_COLOR);   // Read the file

	if (!orig.data)                              // Check for invalid input
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
			pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(orig.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
	//}
}