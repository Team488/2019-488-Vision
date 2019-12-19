#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include "smootour.h"

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
    std::string arg = argv[1];
    int cameraNum = 0;
	string port = "5000";
	string ip = "127.0.0.1";
	bool localView = false;
	if (argc == 4) {
		cameraNum = stoi(arg);
		ip = argv[2];
		port = argv[3];
	} else if (argc == 5) {
		cameraNum = stoi(arg);
		ip = argv[2];
		port = argv[3];
		localView = to_bool(string(argv[4]));
	} else {
		cerr << "Error!  Call with morten CAMERANUM IP PORT (view = false)" << endl;
		cerr << "Using defaults - camera 0, localhost:5000 view false" << endl;
	}

	//Open default camera
	VideoCapture cap(cameraNum);

	if (cap.isOpened() == false)
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}
    cap.set(CAP_PROP_FRAME_WIDTH, 960);
    cap.set(CAP_PROP_FRAME_HEIGHT, 540);
	int iWidth = cap.get(CAP_PROP_FRAME_WIDTH);
	int iHeight = cap.get(CAP_PROP_FRAME_HEIGHT);
	int codec = VideoWriter::fourcc('X','2','6','4');
    const cv::String output_pipeline = "appsrc ! video/x-raw, format=(string)BGR width=(int)"+ to_string(iWidth) +", height=(int)"+ to_string(iHeight) +", framerate=(fraction)30/1 ! videoconvert ! omxh265enc quality-level=0 control-rate=2 bitrate=400000 ! video/x-h265, stream-format=(string)byte-stream ! h265parse ! rtph265pay ! udpsink host="+ip+" port="+port;
    VideoWriter out(output_pipeline, codec, 30, Size(iWidth,iHeight));


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

	while (true)
	{
		bool bSuccess = cap.read(src); // read a new frame from video 

		//Breaking the while loop if the frames cannot be captured
		if (bSuccess == false)
		{
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			break;
		}

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
