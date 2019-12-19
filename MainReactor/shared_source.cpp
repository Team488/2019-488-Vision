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

int main(int argc, const char **argv)
{
    CommandLineParser parser(argc, argv, keys);

    Mat frame;
    cout << "Opening camera..." << endl;
    VideoCapture capture(0); // open the first camera
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);
    if (!capture.isOpened())
    {
        cerr << "ERROR: Can't initialize camera capture" << endl;
        return 1;
    }

    cout << "Frame width: " << capture.get(CAP_PROP_FRAME_WIDTH) << endl;
    cout << "     height: " << capture.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "Capturing FPS: " << capture.get(CAP_PROP_FPS) << endl;

    cout << endl
         << "Press 'ESC' to quit, 'space' to toggle frame processing" << endl;
    cout << endl
         << "Start grabbing..." << endl;

    capture >> frame;

    SharedMatWriter shared(DEMO_MEMORY_NAME, DEMO_HEADER_NAME, frame);
    cout << "Got writer" << endl;

    bool enableProcessing = false;
    int nFrames = 0;
    int64 t0 = getTickCount();
    int64 processingTime = 0;
    for (;;)
    {
        shared.updateFrame(capture);

        if (shared.mat.empty())
        {
            cerr << "ERROR: Can't grab camera frame." << endl;
            break;
        }
        nFrames++;
        if (nFrames % 100 == 0)
        {
            t0 = showFrameStats(nFrames, processingTime, t0);
            processingTime = 0;
        }

        int key = waitKey(1);
        if (key == 27 /*ESC*/)
            break;
        if (key == 32 /*SPACE*/)
        {
            enableProcessing = !enableProcessing;
            cout << "Enable frame processing ('space' key): " << enableProcessing << endl;
        }
    }

    return 0;
}
