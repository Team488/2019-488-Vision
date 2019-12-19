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
    int width = 960;//color_mat.mat.width();
    int height = 540;//depth_stream.height();

    Mat color(height, width, CV_8UC3);
    ifstream colorFile ("colorData.bin", ios::in | ios::binary);
    int dataSize = width*height*3;
    char colorData[dataSize];
    VideoWriter video("rscolor.avi", VideoWriter::fourcc('M','J','P','G'), 60, Size(width, height), true);

    while(!colorFile.eof()){
        colorFile.read(colorData, dataSize);
        //depth.data = (uchar*)depthData;
        color = Mat(540, 960, CV_8UC3, colorData);
        video.write(color);
        
        cv::imshow("Out", color);
        if (waitKey(16) == 27) { //Set to ~60 FPS
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}