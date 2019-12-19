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

const string FILE_NAME = "/media/nvidia/datalog/irData.bin";

int main() 
{
    SharedMat ir_mat(RSFRONT_IRONE_MEMORY_NAME, RSFRONT_IRONE_HEADER_NAME);

    int width = 848;//color_mat.mat.width();
    int height = 480;//depth_stream.height();

    //VideoWriter video("rsdepth.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(width, height), true);

    Mat ir;
    ofstream irFile (FILE_NAME, ios::out | ios::binary);
    for (;;){
        int frameNum = ir_mat.waitForFrame();
        //video.write(depth_mat.mat);
        //imshow("Depth", depth_mat.mat);
        ir = ir_mat.mat;
        int dataSize = (ir.size().area());
        irFile.write((char*)ir.data, dataSize);
        if (waitKey(1) == 27) {
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}