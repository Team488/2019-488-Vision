#include <iostream>
#include <time.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <chrono>
#include <thread>
#include "../MainReactor/SharedMat.h"
#include "../MainReactor/rsfront_camera_include.h"

using namespace std;
using namespace cv;

int main() 
{
    SharedMat color_mat(RSFRONT_COLOR_MEMORY_NAME, RSFRONT_COLOR_HEADER_NAME);

    int width = 960;//color_mat.mat.width();
    int height = 540;//depth_stream.height();

    VideoWriter video("/media/nvidia/datalog/rscolor.avi", VideoWriter::fourcc('M','J','P','G'), 60, Size(width, height), true);

    for (;;){
        int frameNum = color_mat.waitForFrame();
        video.write(color_mat.mat);
        
        if (waitKey(1) == 27) {
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}
