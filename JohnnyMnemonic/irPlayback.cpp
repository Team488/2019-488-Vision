#include <iostream>
#include <time.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <fstream>

using namespace std;
using namespace cv;

int main() 
{
    int width = 848;//color_mat.mat.width();
    int height = 480;//depth_stream.height();

    Mat ir(height, width, CV_8UC1);
    ifstream irFile ("irData.bin", ios::in | ios::binary);
    int dataSize = width*height;
    char irData[dataSize];
    VideoWriter video("rsir.avi", VideoWriter::fourcc('M','J','P','G'), 60, Size(width, height), true);

    while(!irFile.eof()){
        irFile.read(irData, dataSize);
        ir = Mat(height, width, CV_8UC1, irData);
        video.write(ir);
        
        cv::imshow("Out", ir);
        if (waitKey(16) == 27) { //Set to ~60 FPS
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}