#include <iostream>

#include <Kinect.h>
#include <opencv2/opencv.hpp>

// OpenCV and VS 2015: http://dogfeatherdesign.com/opencv-3-0-microsoft-visual-studio-2015-cmake-and-c/

// Example http://answers.opencv.org/question/54785/face-detect-kinect-v2-opencv/
// https://msdn.microsoft.com/en-us/library/dn791996.aspx

// Kinect with CS: https://msdn.microsoft.com/en-us/library/hh855360.aspx
// OpenCV: http://docs.opencv.org/2.4/doc/tutorials/introduction/windows_visual_studio_Opencv/windows_visual_studio_Opencv.html
// Kinect C++ Reference https://msdn.microsoft.com/en-us/library/hh855364.aspx

bool captureCamera();

int main(int argc, char** argv) {
	return captureCamera();
}


bool captureCamera() {
	cv::VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
	{
		std::cerr << "Error : VideoCapture open default camera" << std::endl;
		return -1;
	}

	cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
	while(true)
	{
		cv::Mat frame;
		cap >> frame; // get a new frame from camera
		//cvtColor(frame, edges, COLOR_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		//Canny(edges, edges, 0, 30, 3);
		cv::imshow("Camera", frame);
		if (cv::waitKey(30) >= 0) break;
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

// Get a working kinect sensor
bool captureKinect() {
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

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width(&width); // Kinect v1 height = 640
	pDescription->get_Height(&height); // Kinect v1 width =  480
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

	cv::Mat bufferMat(height, width, CV_8UC4);

	//while (1)
	//{
		// Color Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
	//}
}