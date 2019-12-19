#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace cv;
using namespace std;

string ip;
string port;

int main(int argc, char** argv) {
    //Set defaults
    port = "5000";
    ip = "127.0.0.1";
    if (argc < 3 || argc > 4) {
        cerr << "Error! call with videosend IP PORT" << '\n';
        cerr << "Using Defaults - localhost:5000" << '\n';
    } else {
        ip = argv[1];
        port = argv[2];
    }
    // VideoCapture: Getting frames using 'v4l2src' plugin, format is 'BGR' because
    // the VideoWriter class expects a 3 channel image since we are sending colored images.
    // Both 'YUY2' and 'I420' are single channel images. 
    VideoCapture cap(0);//cap("v4l2src ! video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! appsink",CAP_GSTREAMER);

    // VideoWriter: 'videoconvert' converts the 'BGR' images into 'YUY2' raw frames to be fed to
    // 'jpegenc' encoder since 'jpegenc' does not accept 'BGR' images. The 'videoconvert' is not
    // in the original pipeline, because in there we are reading frames in 'YUY2' format from 'v4l2src'
    int codec = VideoWriter::fourcc('X','2','6','4');
    const cv::String output_pipeline = "appsrc ! video/x-raw, format=(string)BGR width=(int){}, height=(int){}, framerate=(fraction)30/1 ! videoconvert ! omxh264enc bitrate=600000 ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! udpsink host="+ip+" port="+port;
    VideoWriter out(output_pipeline, codec, 30, Size(640,480));
    //VideoWriter out("appsrc ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! ffenc_mpeg4 ! rtpmp4vpay config-interval=3 ! udpsink host=127.0.0.1 port=5000",CAP_GSTREAMER,0,30,Size(640,480),true);
    //VideoWriter out("appsrc ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! x264enc ! rtph264pay ! udpsink host=127.0.0.1 port=5000", CAP_GSTREAMER,0,30,Size(640,480),true);

    if(!cap.isOpened() || !out.isOpened())
    {
        cout<<"VideoCapture or VideoWriter not opened"<<endl;
        exit(-1);
    }

    Mat frame;

    while(true) {

        cap.read(frame);

        if(frame.empty())
            break;

        out.write(frame);

        imshow("Sender", frame);
        if(waitKey(1) == 's')
            break;
    }
    destroyWindow("Sender");

}
