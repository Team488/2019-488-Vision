#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

int main() {
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