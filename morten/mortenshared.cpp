#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include "smootour.h"
#include "../MainReactor/SharedMat.h"
#include "../MainReactor/rsfront_camera_include.h"

using namespace cv;
using namespace std;

Mat src; Mat src_gray; Mat canny_output; Mat drawing;
int thresh = 35;
int max_thresh = 255;
RNG rng(12345);

bool to_bool(string str) {
	transform(str.begin(), str.end(), str.begin(), ::tolower);
	istringstream is(str);
	bool b;
	is >> boolalpha >> b;
	return b;
}

void erode_and_dilate(cv::Mat mat, bool erode = true, bool dilate = true) {
	//apply both operations to the mat

	//http://docs.opencv.org/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
	
	if (erode) {
		int erosion_type = cv::MORPH_RECT;
		int erosion_size = 3;

		cv::Mat erode_element = cv::getStructuringElement( erosion_type,
										cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
										cv::Point( erosion_size, erosion_size ) );
		cv::erode( mat, mat, erode_element );
	}

	if (dilate) {
		int dilation_type = cv::MORPH_RECT;
		int dilation_size = 5;

		cv::Mat dilate_element = cv::getStructuringElement( dilation_type,
										cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
										cv::Point( dilation_size, dilation_size ) );
		cv::dilate( mat, mat, dilate_element );
	}
}

int main(int argc, char** argv)
{
	string port = "5000";
	string ip = "127.0.0.1";
	bool localView = false;
	if (argc == 3) {
		ip = argv[1];
		port = argv[2];
	} else if (argc == 4) {
		ip = argv[1];
		port = argv[2];
		localView = to_bool(string(argv[3]));
	} else {
		cerr << "Error!  Call with mortenintel IP PORT (view = false)" << endl;
		cerr << "Using defaults - localhost:5000 view false" << endl;
	}

	int iWidth = 960;//cap.get(CAP_PROP_FRAME_WIDTH);
	int iHeight = 540;//cap.get(CAP_PROP_FRAME_HEIGHT);
	int codec = VideoWriter::fourcc('X','2','6','4');
    const cv::String output_pipeline = "appsrc ! video/x-raw, format=(string)BGR, width=(int)"+ to_string(iWidth) +", height=(int)"+ to_string(iHeight) +", framerate=(fraction)60/1 ! videoconvert ! omxh264enc quality-level=0 control-rate=2 bitrate=400000 ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! udpsink host="+ip+" port="+port;
    VideoWriter out(output_pipeline, codec, 60, Size(iWidth,iHeight));


	if (out.isOpened() == false)
	{
		cout << "Cannot open VideoWriter" << endl;
		return -1;
	}


	cout << "Resolution is: " << iWidth << " x " << iHeight << endl;
	string rv_name = "Raw Video";
	string vv_name = "Vector	 View";
	string is_name = "Implicit Surface";
	if (localView) {
		namedWindow(rv_name);
		createTrackbar(" Canny thresh:", "Raw Video", &thresh, max_thresh);
		namedWindow(vv_name);
		namedWindow(is_name);
	}

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Smootour frame_smootour(iHeight, iWidth);
	Mat implicit_image;

	SharedMat color_mat(RSFRONT_COLOR_MEMORY_NAME, RSFRONT_COLOR_HEADER_NAME);

	while (true)
	{
		int frameNum = color_mat.waitForFrame();
		src = color_mat.mat;

		cvtColor(src, src_gray, COLOR_BGR2GRAY);
		blur(src_gray, src_gray, Size(3, 3));

		/// Detect edges using canny
		Canny(src_gray, canny_output, thresh, thresh * 2, 3);
		/// Find contours
		//findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		frame_smootour.update(canny_output);
        contours = frame_smootour.get_contours();
		implicit_image = 255*frame_smootour.get_implicit_image();

		/// Draw contours
		drawing = Mat(canny_output.size(), CV_8UC3, Scalar(255,255,255));//Mat::zeros(canny_output.size(), CV_8UC3);
		drawContours(drawing, contours, -1, Scalar(0,0,0), 1);

		// Draw Lines
		line(drawing, Point(iWidth/2, 0), Point(iWidth/2, iHeight), Scalar(255,0,0), 2);
		line(drawing, Point(iWidth/4, iHeight/3), Point(iWidth/4, 2*iHeight/3), Scalar(0,255,0), 2);
		line(drawing, Point(3*iWidth/4, iHeight/3), Point(3*iWidth/4, 2*iHeight/3), Scalar(0,255,0), 2);

		// for (int i = 0; i < contours.size(); i++)
		// {
		// 	Scalar color = Scalar(0,0,0);//Scalar(255, 255, 255);//Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		// 	drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		// }

		//show the frame in the created window
		if (localView) {
			imshow(rv_name, src);
			imshow(vv_name, drawing);
			imshow(is_name, implicit_image);
		}

		out.write(drawing);



		//wait for for 10 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		//If the any other key is pressed, continue the loop 
		//If any key is not pressed withing 10 ms, continue the loop 
		if (waitKey(1) == 27)
		{
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
	}

	return 0;
}
