#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <iostream>
#include "SharedMat.h"

using namespace std;
using namespace cv;

int64 showFrameStats(int nFrames, int64 processingTime, int64 t0)
{
    const int N = 100;
    int64 t1 = getTickCount();
    cout << "Frames captured: " << format("%5lld", (long long int)nFrames)
         << "    Average FPS: " << format("%9.1f", (double)cv::getTickFrequency() * N / (t1 - t0))
         << "    Average time per frame: " << format("%9.2f ms", (double)(t1 - t0) * 1000.0f / (N * cv::getTickFrequency()))
         << "    Average processing time: " << format("%9.2f ms", (double)(processingTime)*1000.0f / (N * cv::getTickFrequency()))
         << std::endl;
    return t1;
}

const char *keys =
    "{@filter || filter to apply to image}"
    "{help h  ||}"
    "{server  || whether to start as a server}";

int main(int argc, const char **argv) {
   //Open managed shared memory
    cout << "Starting client" << endl;
    CommandLineParser parser(argc, argv, keys);

    string filter = parser.get<String>("@filter");

    SharedMat shared(DEMO_MEMORY_NAME, DEMO_HEADER_NAME);

    cout << "Made Shared Mat" << endl;

    if (filter == "")
    {
        cout << "Unknown filter, using none!" << endl;
    }

    size_t nFrames = 0;
    int64 t0 = getTickCount();
    int64 processingTime = 0;
    for (;;)
    {
        int frameNum = shared.waitForFrame();
        int64 tp0 = getTickCount();

        if (filter == "canny")
        {
            Mat processed;
            Canny(shared.mat, processed, 400, 1000, 5);
            imshow("Frame", processed);
        }
        else if (filter == "blur")
        {
            Mat processed;
            boxFilter(shared.mat, processed, -1, cv::Size(100, 100));
            imshow("Frame", processed);
        }
        else
        {
            imshow("Frame", shared.mat);
        }
        processingTime += getTickCount() - tp0;
        nFrames++;
        if (nFrames % 100 == 0)
        {
            t0 = showFrameStats(nFrames, processingTime, t0);
            processingTime = 0;
        }
        int key = waitKey(1);
        if (key == 27 /*ESC*/)
            break;
    }

    std::this_thread::sleep_for(chrono::milliseconds(5));
}