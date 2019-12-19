#include <iostream>
#include <time.h>
#include <stdio.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() 
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);

    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    int width = depth_stream.width();
    int height = depth_stream.height();

    VideoWriter video("RealSenseVideo.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(width, height), true);

    rs2::frameset frames;

    for (;;){
        frames = pipe.wait_for_frames();

        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        video.write(color);

        // Display in a GUI
        namedWindow("Display Video", WINDOW_AUTOSIZE );
        imshow("Display Video", color);

        if (waitKey(1) == 27) {
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
    }

    return 0;
}