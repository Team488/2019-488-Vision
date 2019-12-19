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

using namespace cv;
using namespace std;

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
};

//Return -1 for negative, 1 for positive and 0 for zero
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class TargetingComputer
{
    private:
        //CAMERA PARAMETERS
        double camSensorH = 6.18; //mm
        double camSensorV = 5.85; //mm
        int resolutionH = 800; //px
        int resolutionV = 600; //px
        double ratio = resolutionH/resolutionV;
        double lensFocalLength = 2.1;//mm
        double horizontalView = 2*atan2(camSensorH, (2*lensFocalLength));
        double verticalView = 2*atan2(camSensorV, (2*lensFocalLength));
        

        //SYSTEM PARAMETERS
        Scalar lower_color = Scalar(0, 0, 200);
        Scalar upper_color = Scalar(255, 255, 255);
        int minContourSize = 10;
        int minHullSize = 10; 

        float yawOut = -488.0;

    public:
        //TargetingComputer();
        void threshold_video(Mat& frame, Mat& mask);
        Mat getContours(Mat& frame, Mat& mask);
        void findTargets(vector<vector<Point>> contours, Mat& image, int centerX, int centerY);
        double translateRotation(double rotation, int width, int height);
        double calculateDistance(double heightOfCamera, double heightOfTarget, double pitch);
        double calculateYaw(int pixelX, int centerX, double hFocalLength);
        double calculatePitch(int pixelY, int centerY, double vFocalLength);
        double getEllipseRotation(Mat& image, vector<Point> contour);
        float getTargetYaw() {return yawOut;}
        void solveforPnP(Mat& srcIn, const CornersParameters& params, Mat& rvec, Mat& tvec);
        void retrieveCorners(Mat& src_grayIn, vector<Point3f>& corners, const CornersParameters& params);
};


#endif
