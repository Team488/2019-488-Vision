// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    string filepath = "./images/";
    string filename = "ir_image";
    int fileending = 0;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 60);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);
	rs2::device selected_device = selection.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();
	depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.f);
	depth_sensor.set_option(RS2_OPTION_EXPOSURE, 7000);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    while(1) {
        frames = pipe.wait_for_frames();
        //Get each frame
        rs2::frame ir_frame = frames.get_infrared_frame();

        // Creating OpenCV Matrix from a color image
        Mat ir(Size(848, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);

        // Display in a GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", ir);

        int key = waitKey(15);
        if(key == 32) {
            //Capture
            imwrite(filepath+filename+to_string(fileending)+".png", ir);
            fileending++;
        } else if (key == 27) {
            //Quit
            return 1;
        }
    }

    return 0;
}