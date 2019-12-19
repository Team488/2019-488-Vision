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

    Mat depth(480, 848, CV_16UC1);
    ifstream depthFile ("data.bin", ios::in | ios::binary);
    int dataSize = 848*480*2;
    char depthData[dataSize];
    while(!depthFile.eof()){
        depthFile.read(depthData, dataSize);
        //depth.data = (uchar*)depthData;
        depth = Mat(480, 848, CV_16UC1, depthData);
        double min;
        double max;
        cv::minMaxIdx(depth, &min, &max);
        cv::Mat adjMap;
        // Histogram Equalization
        float scale = 255.0f / (max-min);
        depth.convertTo(adjMap, CV_8UC1, scale, -min*scale); 
        cv::Mat falseColorsMap;
        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

        cv::imshow("Out", falseColorsMap);
        imshow("Depth", depth);
        if (waitKey(16) == 27) { //Set to ~60 FPS
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}