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

const string FILE_NAME = "/media/nvidia/datalog/depthData.bin";

int main() 
{
    SharedMat depth_mat(RSFRONT_DEPTH_MEMORY_NAME, RSFRONT_DEPTH_HEADER_NAME);

    int width = 848;//color_mat.mat.width();
    int height = 480;//depth_stream.height();

    //VideoWriter video("rsdepth.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(width, height), true);

    Mat depth;
    ofstream depthFile (FILE_NAME, ios::out | ios::binary);
    for (;;){
        int frameNum = depth_mat.waitForFrame();
        //video.write(depth_mat.mat);
        //imshow("Depth", depth_mat.mat);
        depth = depth_mat.mat;
        int dataSize = (depth.size().area())*2;
        depthFile.write((char*)depth.data, dataSize);
        if (waitKey(1) == 27) {
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}