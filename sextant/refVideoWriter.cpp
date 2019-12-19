#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

int videoWriter(string& nameIn) {
	Mat newCamMatForUndistort;
	Mat src; Mat src_unwarped;
	Mat map1, map2;

	//Open default camera
	VideoCapture cap(0);

	if (cap.isOpened() == false)
	{
		cout << "Cannot open camera" << endl;
		cin.get();
		return -1;
	}

	//manually sets camera dimensions
	cap.set(CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

	//finds & prints camera dimensions
	double dWidth = cap.get(CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT);
	cout << "Resolution is: " << dWidth << " x " << dHeight << endl;

	//hardcoded calibration data
	Mat K = (Mat_<double>(3, 3) << 540.6884489226692, 0.0, 951.3635524878698, 0.0, 540.4187901470385, 546.9124878500451, 0.0, 0.0, 1.0);
	Mat D = (Mat_<double>(1, 4) << -0.04517325603821452, 0.001435732351585509, -0.004105241869408653, 0.0009228132505096691);
	cout << "K = " << K << endl;
	cout << "D = " << D << endl;

	//generates undistortion maps from first frame
	Size image_size = Size(1920, 1080);
	cout << "size" << src.size() << endl;
	fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, image_size, Matx33d::eye(), newCamMatForUndistort, 1, image_size);
	fisheye::initUndistortRectifyMap(K, D, Matx33d::eye(), newCamMatForUndistort, image_size, CV_16SC2, map1, map2);


	cap.read(src);

	cout << "Video name (.avi is added automatically): ";
	//string nameIn;
	cin >> nameIn;
	stringstream nameS;
	nameS << "./setup_videos/" << nameIn << ".avi";
	string name = nameS.str();
	cout << name << endl;


	VideoWriter video(name, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(dWidth, dHeight));

	//no endl here for user input
	cout << "how many seconds: ";
	int seconds;
	cin >> seconds;
	cout << endl << "writing a " << seconds << " second video" << endl;
	for (int i = 0; i < seconds * 10; i++) {
		Mat frame;
		cap >> frame;
		if (frame.empty()) {
			break;
		}
		remap(frame, frame, map1, map2, cv::INTER_LINEAR);

		video.write(frame);
	}
	video.release();
	cout << "wrote a " << seconds << " second video called: " << name << endl;

	return 0;

}

int bestFrame() {
	vector<String> videoNames;
	vector<vector<Mat>> videoMats;
	vector<Mat> best_frames;

	glob("./setup_videos/*.avi", videoNames, false);

	for (int k = 0; k < videoNames.size(); k++) {
		cout << videoNames[k] << endl;
	}

	for (int i = 0; i < videoNames.size(); i++) {
		cout << "video names size: " << videoNames.size() << endl;

		VideoCapture cap(videoNames[i]);
		Ptr<Feature2D> orb = ORB::create();
		vector<KeyPoint> mostKeypoints;
		Mat vid_best_frame;
		if (!cap.isOpened()) {
			cout << "could not open referance video ---" << i << "---" << endl;
			return -1;
		}

		for (int j = 0; cap.get(CAP_PROP_FRAME_COUNT); j++) {
			Mat frame;
			cap >> frame;

			if (frame.empty()) {
				break;
			}

			//these are just for ranking becasue I don't know how to write these values to a JSON
			vector<KeyPoint> keypoints;
			Mat descriptors;

			orb->detectAndCompute(frame, Mat(), keypoints, descriptors);

			if (keypoints.size() > mostKeypoints.size()) {
				mostKeypoints = keypoints;
				vid_best_frame = frame;
			}

			if (vid_best_frame.empty()) {
				cout << "best frame is empty!" << endl;
			}
		}
		best_frames.push_back(vid_best_frame);
		cout << "i: " << i << endl;
	}

	for (int i = 0; i < best_frames.size(); i++) {
		int a = i + 1;
		stringstream nameS;
		nameS << "./setup_images/" << a << ".png";
		string name = nameS.str();

		imwrite(name, best_frames[i]);
		cout << "best_frame size: " << best_frames.size() << endl;
		cout << "Idx:  " << i << endl;
	}

	return 0;
}

int main(int argc, char** argv) {
	string name;
	int numVids;
	cout << "how many videos to write?" << endl;
	cin >> numVids;
	for (int i = 0; i < numVids; i++) {
		videoWriter(name);
	}
	bestFrame();
}