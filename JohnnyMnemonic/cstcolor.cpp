#include <iostream>
#include <time.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <chrono>
#include <thread>
#include "../MainReactor/SharedMat.h"
#include "../MainReactor/rsfront_camera_include.h"
#include <fstream>

using namespace std;
using namespace cv;

const string FILE_NAME = "/media/nvidia/datalog/constellationData.bin";

int main() 
{
    SharedMat color_mat(RSFRONT_COLOR_MEMORY_NAME, RSFRONT_COLOR_HEADER_NAME);

    int width = 1080;//color_mat.mat.width();
    int height = 920;//depth_stream.height();

    //VideoWriter video("rsdepth.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(width, height), true);

    Mat color;
    ofstream colorFile (FILE_NAME, ios::out | ios::binary);
    for (;;){
        int frameNum = color_mat.waitForFrame();
        //video.write(depth_mat.mat);
        //imshow("Depth", depth_mat.mat);
        color = color_mat.mat;
        int dataSize = (color.size().area())*3;
        colorFile.write((char*)color.data, dataSize);
        if (waitKey(1) == 27) {
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}