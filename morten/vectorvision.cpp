#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include "smootour.h"

using namespace cv;
using namespace std;

Mat src; Mat src_gray; Mat canny_output; Mat drawing;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

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
    try {
        std::size_t pos;
        cameraNum = std::stoi(arg, &pos);
        if (pos < arg.size()) {
            std::cerr << "Trailing characters after number: " << arg << '\n';
        }
    } catch (std::invalid_argument const &ex) {
        std::cerr << "Invalid number: " << arg << '\n';
    } catch (std::out_of_range const &ex) {
        std::cerr << "Number out of range: " << arg << '\n';
    }
	//Open default camera
	VideoCapture cap(cameraNum);

	if (cap.isOpened() == false)
	{
		cout << "Cannot open camera" << endl;
		cin.get();
		return -1;
	}

    // cap.set(CAP_PROP_FRAME_WIDTH, 800);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 600);

	double dWidth = cap.get(CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT);

	cout << "Resolution is: " << dWidth << " x " << dHeight << endl;
	string rv_name = "Raw Video";
	string vv_name = "Vector View";
	string is_name = "Implicit Surface";
	namedWindow(rv_name);
	createTrackbar(" Canny thresh:", "Raw Video", &thresh, max_thresh);
	namedWindow(vv_name);
	namedWindow(is_name);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Smootour frame_smootour(dHeight, dWidth);
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
		imshow(rv_name, src);
		imshow(vv_name, drawing);
		imshow(is_name, implicit_image);



		//wait for for 10 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		//If the any other key is pressed, continue the loop 
		//If any key is not pressed withing 10 ms, continue the loop 
		if (waitKey(10) == 27)
		{
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
	}

	return 0;
}
