#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void warpWithHomography(float GOOD_MATCH_PERCENT, Mat img1, Mat img2, Mat &img1Reg) {
	// Variables to store keypoints and descriptors
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;

	Ptr<GFTTDetector> detector = GFTTDetector::create();
	detector->detect(img1, keypoints1);
	detector->detect(img2, keypoints2);

	Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
	extractor->compute(img1, keypoints1, descriptors1);
	extractor->compute(img2, keypoints2, descriptors2);

	// Match features.
	vector<DMatch> matches;
	Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING, true);
	matcher->match(descriptors1, descriptors2, matches);

	//sort matches
	sort(matches.begin(), matches.end());

	//kills bad matches
	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
	cout << matches.size() << endl;
	cout << numGoodMatches << endl;
	matches.erase(matches.begin() + numGoodMatches, matches.end());

	//draws matches
	Mat imMatches;
	drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
	imshow("Display window", imMatches);

	//extract location of good matches
	vector<Point2f> points1, points2;

	for (size_t i = 0; i < matches.size(); i++) {
		points1.push_back(keypoints1[matches[i].queryIdx].pt);
		points2.push_back(keypoints2[matches[i].trainIdx].pt);
	}


	Mat homo;
	//find homo
	homo = findHomography(points1, points2, RANSAC);
	cout << "homo is: " << endl << homo << endl;
	
	warpPerspective(img1, img1Reg, homo, img2.size());
}


int main(int argc, char** argv) {
	Mat img1Reg;
	const float GOOD_MATCH_PERCENT = 0.10f;
	Mat img1 = imread("IMG_1365.JPG");
	Mat img2 = imread("image.png");

	warpWithHomography(GOOD_MATCH_PERCENT, img1, img2, img1Reg);

	imshow("yeet window", img1Reg);
	imwrite("C:\\Users\\Gabriel Young\\Desktop\\X-Bot\\Vision\\sextant\\build\\yeet.jpg", img1Reg);

	waitKey(0);
}