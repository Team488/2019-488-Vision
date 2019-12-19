//A camera focused implementation of @heathkit's original Shared Source - particularly for building individual camera ingest systems.
//To use it, create a new .cpp that extends Camera with the filters you want and a shared .h file with your ingestors that provide
//the shared header/memory handle.  Be sure to call "setMemoryHandles" before setting up.

#ifndef CAMERA_H
#define CAMERA_H 1

#include <opencv2/opencv.hpp>
// #include "opencv2/core/utility.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <streambuf>
#include "SharedMat.h"
#include "picojson.h"

using namespace picojson;
using namespace cv;
using namespace std;

enum CameraType { RGB, IR, DEPTH, NOCAMSET };

struct CameraSettings {
    int exposure;
    int contrast;
    int brightness;
    int gain;
    int saturation;
    CameraType camType;
};

//TODO: Add a means for Cameras to tell their recipients what type of data they should expect
class Camera {
    private:
        Mat cameraMatrix;
        Mat cameraDistortion;
        Mat cameraExtrinsics; //Camera Frame from Robot Frame
        double camSensorH, camSensorV;
        int resolutionH, resolutionV;
        double lensFocalLength;
        CameraType type;
        CameraSettings settings;
        virtual Mat applyFilters(Mat &inMat) = 0;
        char * memoryHandle;
        char * headerHandle;
        SharedMatWriter shared;//undistored and filtered output

    public:
        //Setters
        void setSensorDimensions (double H, double V) {camSensorH = H; camSensorV = V;}
        void setCameraResolution (int H, int V) {resolutionH = H; resolutionV = V;}
        void setLensFocalLength (double lensFL) {lensFocalLength = lensFL;}
        void setMemoryHandles(char* headerH, char* memoryH) { memoryHandle = memoryH; headerHandle = headerH;}

        //Getters
        double getCameraRatio() {return resolutionH/resolutionV;}
        double getHorizontalView() {return 2*atan2(camSensorH, (2*lensFocalLength));}
        double getVerticalView() {return 2*atan2(camSensorV, (2*lensFocalLength));}
        
        //Loader
        bool loadCameraSettings (VideoCapture &capture, const string& path);
        //Saver
        bool saveCameraSettings (const string& path, CameraSettings settings);
        bool saveCameraSettings (const string& path, const VideoCapture& cap, const CameraType& camType);
        CameraSettings getCurrentSettings(const VideoCapture& cap, const CameraType& type);
        //Operations
        bool setup(VideoCapture &capture, Mat &cameraFrame, string path = "../config.json");
        bool updateFrame(Mat &inMat);
};



#endif //ifndef CAMERA_H