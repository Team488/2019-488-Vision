#ifndef TARGETINGCOMPUTER_H
#define TARGETINGCOMPUTER_H

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <math.h>
#include <algorithm>
#include <functional>
#include <czmq.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <chrono>
#include <thread>
#include "../MainReactor/SharedMat.h"
#include "../MainReactor/rsfront_camera_include.h"
#include "json_spirit.h"


using namespace cv;
using namespace std;
using namespace json_spirit;

bool to_bool(string str) {
	transform(str.begin(), str.end(), str.begin(), ::tolower);
	istringstream is(str);
	bool b;
	is >> boolalpha >> b;
	return b;
}

//custom Contour Sort - returns a sorted list of contours from largest to smallest
struct {
	bool operator()(vector<Point> a, vector<Point> b) const
	{
		if (contourArea(a) > contourArea(b)) return true;
		else return false;
	}
} contourBigToSmall;

struct ContourData {
    Point center;
    double rotation;
    vector<Point> contour;
    RotatedRect rect;
};

struct {
    bool operator()(ContourData a, ContourData b) const
    {
        if (a.center.x < b.center.x) return true;
        else return false;
    }
} contourDataLeftToRight;

struct TargetData{
    Point center;
    double yaw;
    RotatedRect rectOne;
    RotatedRect rectTwo;
};

struct {
    bool operator()(TargetData a, TargetData b) const
    {
        if (fabs(a.yaw) < fabs(b.yaw)) return true;
        else return false;
    }
} targetDataSmallestYaw;

struct CameraSettings {
    int exposure;
    int contrast;
    int brightness;
    int gain;
    int saturation;
};

struct CornersParameters {
    int maxCorners;
    double qualityLevel;
    double minDistance;
    int blockSize, gradientSize; 
    bool useHarrisDetector; 
    double k;
    CornersParameters(int maxCornersIn, double qualityLevelIn, double minDistanceIn, int blockSizeIn, int gradientSizeIn, bool useHarrisDetectorIn, double kIn) {
        maxCorners = maxCornersIn;
        qualityLevel = qualityLevelIn;
        minDistance = minDistanceIn;
        blockSize = blockSizeIn;
        gradientSize = gradientSizeIn;
        useHarrisDetector = useHarrisDetectorIn;
        k = kIn;
    }
};

enum Status { active, retro_target_found, optical_flow_tracking, inactive };

class DataToJSON
{
    private:
        bool goodFix = false;
        Status tcMode = inactive;
        double range = -1.0;
        double yaw = -488.0;
        double rotation = -488.0;
        Object output;

    public:
        void updateJSON (bool fixStatus, Status currentStatus, double currentYaw, double currentRange, double currentRotation) {
            goodFix = fixStatus;
            tcMode = currentStatus;
            range = currentRange;
            yaw = currentYaw;
            rotation = currentRotation;
            output.clear();
            output.push_back( Pair("hasTarget", goodFix));
            output.push_back(Pair("mode", tcMode)); 
            output.push_back(Pair("yaw", yaw)); 
            output.push_back(Pair("range", range)); 
            output.push_back(Pair("rotation", rotation));
        }

        string getJSONString() {
            return write(output, none, 17);
        }

        string updateJSONToString(bool fixStatus, Status currentStatus, double currentYaw, double currentRange, double currentRotation) {
            updateJSON (fixStatus, currentStatus, currentYaw, currentRange, currentRotation);
            return getJSONString();
        }

};

//Return -1 for negative, 1 for positive and 0 for zero
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class TargetingComputer
{
    private:
        //CAMERA PARAMETERS
        double camSensorH = 3.896; //mm
        double camSensorV = 2.453; //mm
        int resolutionH = 848; //px
        int resolutionV = 480; //px
        int centerX = resolutionH/2;
        int centerY = resolutionV/2;
        double ratio = resolutionH/resolutionV;
        double lensFocalLength = 1.93;//mm
        double horizontalView = 91.2;//2*atan2(camSensorH, (2*lensFocalLength));
        double verticalView = 65.6;//2*atan2(camSensorV, (2*lensFocalLength));
        

        //SYSTEM PARAMETERS
        Scalar lower_color = Scalar(0, 0, 200);
        Scalar upper_color = Scalar(255, 255, 255);
        int lower_bin = 225;
        int upper_bin = 255;
        int minContourSize = 50;
        int minHullSize = 50; 

        //Corner Parameters
        CornersParameters cornerParams = CornersParameters(8, 0.01, 10, 3, 3, false, 0.04);
        
        //Pts in cm
        //Center of target offset is 14.208125 in either direction.
        //RAW
        // vector<Point3d> leftTapePoints = {Point3d(0, 0, 0), Point3d(-4.9, 1.5, 0), Point3d(-1.1, 14.7, 0), Point3d(4, 13.5, 0)};
        // vector<Point3d> rightTapePoints = {Point3d(0, 0, 0), Point3d(-4, 13.5, 0), Point3d(1.1, 14.7, 0), Point3d(4.9, 1.5, 0) };
        //Fixed to midline (and set for BL, TL , TR, BR order)
        vector<Point3d> leftTapePoints = {Point3d(-4.9-14.208125, 1.5, 0), Point3d(-1.1-14.208125, 14.7, 0), Point3d(4-14.208125, 13.5, 0), Point3d(0-14.208125, 0, 0)};
        vector<Point3d> rightTapePoints = {Point3d(0+14.208125, 0, 0), Point3d(-4+14.208125, 13.5, 0), Point3d(1.1+14.208125, 14.7, 0), Point3d(4.9+14.208125, 1.5, 0) };
        //combined
        //vector<Point3d> tapePoints = {Point3d(-1.1-14.208125, 14.7, 0), Point3d(4-14.208125, 13.5, 0), Point3d(-4+14.208125, 13.5, 0), Point3d(1.1+14.208125, 14.7, 0), Point3d(4.9+14.208125, 1.5, 0), Point3d(0+14.208125, 0, 0), Point3d(0-14.208125, 0, 0), Point3d(-4.9-14.208125, 1.5, 0)};
        //left and then right - order matters, but not global order.
        vector<Point3d> tapePoints = {Point3d(-4.9-14.208125, 1.5, 0), Point3d(-1.1-14.208125, 14.7, 0), Point3d(4-14.208125, 13.5, 0), Point3d(0-14.208125, 0, 0), Point3d(0+14.208125, 0, 0), Point3d(-4+14.208125, 13.5, 0), Point3d(1.1+14.208125, 14.7, 0), Point3d(4.9+14.208125, 1.5, 0)};

        float yawOut = -488.0;

    public:
        //TargetingComputer();
        void threshold_RGBvideo(Mat& frame, Mat& mask);
        void threshold_IRvideo(Mat& frame, Mat& mask);
        void thresholdIRWithoutBlur(Mat& frame, Mat& mask);
        Mat getContours(Mat& frame, Mat& mask, vector<Point2f>& corners, bool& fixOut);
        bool findTargets(vector<vector<Point>> contours, Mat& image, vector<Point2f>& corners, int centerX, int centerY);
        double translateRotation(double rotation, int width, int height);
        double calculateDistance(double heightOfCamera, double heightOfTarget, double pitch);
        double calculateYaw(int pixelX);
        double calculatePitch(int pixelY);
        double getEllipseRotation(Mat& image, vector<Point> contour);
        float getTargetYaw() {return yawOut;}
        int getHResolution(){return resolutionH;}
        int getVResolution(){return resolutionV;}
        bool solveforPnP(Mat& srcIn, const CornersParameters& params, Mat& rvec, Mat& tvec);
        bool solveforPnP(Mat& srcIn, const vector<Point2f>& corners, const CornersParameters& params, Mat& rvec, Mat& tvec);
        void retrieveCorners(Mat& src_grayIn, vector<Point2f>& corners, const CornersParameters& params);
        CornersParameters getCornerParams() {return cornerParams;}
        vector<Point3d> getTapePoints(){return tapePoints;}
        double getRotatedRect(Mat& image, vector<Point>& contour, RotatedRect& outRect);
};


#endif
