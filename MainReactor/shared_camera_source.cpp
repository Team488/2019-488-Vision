#include "camera.h"
#include "test_camera_include.h"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

class SharedCamera : public Camera {

    Mat applyFilters(Mat &inMat) {
        return inMat;
    }
};

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
    SharedCamera shareCam;
    shareCam.setMemoryHandles(TEST_HEADER_NAME, TEST_MEMORY_NAME);
    
    cout << "Opening camera..." << endl;
    VideoCapture capture(0); // open the first camera
    capture.set(CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);
    if (!capture.isOpened())
    {
        cerr << "ERROR: Can't initialize camera capture" << endl;
        return 1;
    }

    capture >> frame;

    shareCam.setup(capture, frame);

    bool enableProcessing = false;
    int nFrames = 0;
    int64 t0 = getTickCount();
    int64 processingTime = 0;
    
    for (;;)
    {
        imshow("Image", frame);
        capture >> frame;
        if (shareCam.updateFrame(frame) == false) return 0;
        nFrames++;
        if (nFrames % 100 == 0)
        {
            t0 = showFrameStats(nFrames, processingTime, t0);
            processingTime = 0;
        }
        if(waitKey(1) == 27) break;
    }

    return 0;
}
