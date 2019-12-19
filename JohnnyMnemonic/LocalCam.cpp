#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include <stdio.h>

using namespace std;
using namespace cv;

int main() 
{
    time_t start = time(0);
    
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout << "Error Opening Camera" << endl;
        return -1;
    }

    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);

    cout << "FRAME WIDTH: " << frame_width << endl;
    cout << "FRAME HEIGHT: " << frame_height << endl;

    VideoWriter video("LocalVideo.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(frame_width, frame_height), true);

    for (;;) {
        Mat frame;

        cap >> frame;
        video.write(frame);

        imshow("Camera Frame", frame);

        int timeFromStart = difftime(time(0), start);
        cout << "Time from start: " << timeFromStart << endl;

        if (waitKey(1) == 27) {
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }
    return 0;
}