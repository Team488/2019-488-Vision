#include "runtime.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void videoKeypointMatches(double GOOD_MATCH_PERCENT, Mat src_unwarped, vector<Mat> best_frames, int& frameIdx, Mat& win_frame, vector<Point2f>& videoPointsRef, vector<Point2f>& videoPointsLive) {
	vector<KeyPoint> keypointsRef;
	Mat descriptorsRef;
	Mat img_keypointsRef;

	vector<KeyPoint> keypointsLive;
	Mat descriptorsLive;
	Mat img_keypointsLive;

	vector<DMatch> matches;
	Mat THELINES_V;

	//create feature detector
	Ptr<Feature2D> orb = ORB::create();

	//create Matcher
	Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING, true);

	// detect orb features and compute descriptors for live frame.
	orb->detectAndCompute(src_unwarped, Mat(), keypointsLive, descriptorsLive);

	//draws and shows keypoints of live frame
	drawKeypoints(src_unwarped, keypointsLive, img_keypointsLive, Scalar(0, 0, 255));
	imshow("live keypoints", img_keypointsLive);

	//cout << "best_frame size: " << best_frames.size() << endl;;

	for (int i = 0; i < best_frames.size(); i++) {
		vector<KeyPoint> temp_keypointsRef;
		Mat temp_desciptorsRef;
		vector<DMatch> temp_matches;

		orb->detectAndCompute(best_frames[i], Mat(), temp_keypointsRef, temp_desciptorsRef);

		matcher->match(temp_desciptorsRef, descriptorsLive, temp_matches);
		//cout << "temp matches of " << i << " : " << temp_matches.size() << endl;

		if (temp_matches.size() > matches.size()) {
			matches = temp_matches;
			keypointsRef = temp_keypointsRef;
			descriptorsRef = temp_desciptorsRef;
			frameIdx = i;
			win_frame = best_frames[i];
			//cout << "idx of win frame: " << i << endl << "win frame matches: " << matches.size() << endl;
		}


	}

	//sort by confidence level
	sort(matches.begin(), matches.end());

	const int numGoodMatches = (int)(matches.size() * GOOD_MATCH_PERCENT);
	//cout << "number of matches: " << matches.size() << " number of good matches: " << numGoodMatches << endl;
	matches.erase(matches.begin() + numGoodMatches, matches.end());

	for (size_t i = 0; i < matches.size(); i++) {
		videoPointsRef.push_back(keypointsRef[matches[i].queryIdx].pt);
		videoPointsLive.push_back(keypointsLive[matches[i].trainIdx].pt);
	}

	if (win_frame.empty()) {
		//cout << "win_frame is empty!" << endl;
	}

	drawMatches(win_frame, keypointsRef, src_unwarped, keypointsLive, matches, THELINES_V);
	resize(THELINES_V, THELINES_V, Size(), .5, .5);
	imshow("THELINES_V", THELINES_V);
}

void rvecAndTvec(vector<Point2f> pointsRef, vector<Point2f> pointsLive, Mat K, Mat D, Mat& rvec, Mat& tvec) {
	vector<Point3f> pointsRef3D;

	for (int i = 0; i < pointsRef.size(); i++) {
		Point3d point;
		point.x = (float)pointsRef[i].x;
		point.y = (float)pointsRef[i].y;
		point.z = 0.0f;
		pointsRef3D.push_back(point);
	}

	solvePnPRansac(pointsRef3D, pointsLive, K, D, rvec, tvec);
	//cout << "rotation vector length: " << sum(rvec) << endl;
	//cout << "translation: " << tvec << endl;
}

void homographyPerspectiveWarp (vector<Point2f> pointsRef, vector<Point2f> pointsLive,  Mat best_frame, Mat src_unwarped, Mat& homography, Mat &img_warpedToPerspective) {

	homography = findHomography(pointsRef, pointsLive, RANSAC);
	
	Mat pose;
	//it break here because Mat assignment is broken right now. see method for details.
	//two people on stack overflow thread say this answer is trash
	//cameraPoseFromHomography(homography, pose);
	warpPerspective(src_unwarped, img_warpedToPerspective, homography, best_frame.size());
	resize(img_warpedToPerspective, img_warpedToPerspective, Size(), .5, .5);
}

void cameraOffsetOrigin(Mat rvec, Mat tvec, Mat& T) {
	//cout << "position before T: " << tvec.at<double>(0, 0) << ", " << tvec.at<double>(1, 0) << endl;
	Mat R;
	Rodrigues(rvec, R);
	R = R.t();
	tvec = -R * tvec;
	T = Mat::eye(4, 4, R.type());
	T(Range(0, 3), Range(0, 3)) = R * 1;
	T(Range(0, 3), Range(3, 4)) = tvec * 1;
	//cout << "Offset from ref image" << endl << T << endl;
	ofstream T_file;
	T_file.open("T_data.txt", ios_base::app);
	T_file << T.at<double>(0, 3) << " , " << T.at<double>(1, 3) << endl;
	T_file.close();
}

int offset(int frameIdx, Mat T, Point2d& position) {
	if (frameIdx = 1) {
		//I assume cm
		position.x = 0 + T.at<double>(0, 3);
		position.y = 0 + T.at<double>(1, 3);
		return 0;

	} 
	else if (frameIdx = 2) {
		position.x = 0 + T.at<double>(0, 4);
		position.y = 332 + T.at<double>(1, 4);
		return 0;
	} 
	else if (frameIdx = 3) {
		position.x = 0 + T.at<double>(0, 4);
		position.y = 0  + T.at<double>(1, 4);
		return 0;
	}
	else {
	cout << "frame Idx was not between 1 and 2" << endl;
		return -1;
	}
	return 0;
}

int main(int argc, char** argv) {
	Mat newCamMatForUndistort;
	Mat map1, map2;
	Mat src; Mat src_unwarped;
	Point2d position;
	 
	const double GOOD_MATCH_PERCENT = .1; //increase me!

	vector<String> best_frames_names;
	vector<Mat> best_frames;
	int frameIdx;
	Mat win_frame;

	//check glob and the for loop for bugs
	glob("./setup_images/*.png", best_frames_names, false);

	for (int i = 0; i < best_frames_names.size(); i++) {
		Mat best_frame;
		stringstream pathS;
		pathS << "./setup_images/" << i+1 << ".png";
		string path = pathS.str();
		cout << "path: " << path << endl;
		best_frames.push_back(imread(path));
	}

	//Open default camera
	//or read video
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
	//cout << "Resolution is: " << dWidth << " x " << dHeight << endl;

	//hardcoded calibration data
	Mat K = (Mat_<double>(3, 3) << 540.6884489226692, 0.0, 951.3635524878698, 0.0, 540.4187901470385, 546.9124878500451, 0.0, 0.0, 1.0);
	Mat D = (Mat_<double>(1, 4) << -0.04517325603821452, 0.001435732351585509, -0.004105241869408653, 0.0009228132505096691);
	cout << "K = " << K << endl;
	cout << "D = " << D << endl;

	// read a new frame from video, breaking the while loop if the frames cannot be captured
	bool bSuccess = cap.read(src);

	//spam these to get a later frame. there doesn't seem to be a better way
	cap.read(src);

	bSuccess = cap.read(src);
	if (bSuccess == false)
	{
		cout << "Video camera is disconnected" << endl;
		cin.get(); //Wait for any key press
		return 0;
	}
	cout << "Camera open!" << endl;

	Size image_size = src.size(); 
	//generates undistortions maps from first frame
	fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, image_size, Matx33d::eye(), newCamMatForUndistort, 1, image_size);
	fisheye::initUndistortRectifyMap(K, D, Matx33d::eye(), newCamMatForUndistort, image_size, CV_16SC2, map1, map2);
	
	remap(src, src_unwarped, map1, map2, cv::INTER_LINEAR);

	while (true)
	{
		Mat img_warpedToPerspective, homography, img_matches, rvec, tvec, rvec_out, tvec_out, T;
		vector<Mat> rotations, translations, normals;
		vector<Point2f> pointsRef, pointsLive;
		// read a new frame from video breaking the while loop if the frames cannot be captured
		bSuccess = cap.read(src);
		if (bSuccess == false)
		{
			cout << "Video camera is disconnected" << endl;
			//Wait for any key press
			cin.get();
			break;
		}

		//imshow("before", src);

		//remaps the Mat accoring to unwarping data from "fisheye::initUndistortRectifyMap"
		remap(src, src_unwarped, map1, map2, cv::INTER_LINEAR);

		//keypointMatches(GOOD_MATCH_PERCENT, keypointsRef, descriptorsRef, src_unwarped, ref, img_matches, pointsRef, pointsLive);
		//imshow("THE LINES", img_matches);

		vector<Point2f> videoPointsRef;
		vector<Point2f> videoPointsLive;
		videoKeypointMatches(GOOD_MATCH_PERCENT, src_unwarped, best_frames, frameIdx, win_frame, videoPointsRef, videoPointsLive);

		//cout << "videoPointsRef.size()" << videoPointsRef.size() << endl;

		//get rvec and tvec from ref image
		rvecAndTvec(videoPointsRef, videoPointsLive, K, D, rvec, tvec);
		//cout << "rotation vector lenght: " << sum(rvec) << endl;
		//cout << "translation vector lenght: " << sum(tvec) << endl;

		cameraOffsetOrigin(rvec, tvec, T);

		//generates Point2d with x y position
		offset(frameIdx, T, position);
		cout << "position: " << position << endl;

		imshow("best frame: ", win_frame);
		homographyPerspectiveWarp(videoPointsRef, videoPointsLive, win_frame, src_unwarped, homography, img_warpedToPerspective);
		imshow("img_warpedToPerspective", img_warpedToPerspective);


		//wait for 1 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		//If the any other key is pressed, continue the loop 
		//If any key is not pressed withing 10 ms, continue the loop 
		int keyValue = waitKey(1);
		if (keyValue == 27)
		{
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
	}
	return 0;
}