#ifndef RUNTIME_H
#define RUNTIME_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>

using namespace cv;
using namespace std;


//custom vector Sort - returns a sorted list of vectors from largest to smallest
struct {
	bool operator()(vector<DMatch> a, vector<DMatch> b) const
	{
		if (a.size() > b.size()) {
			return true;
		} else {
			return false;
		}
	}
} vectorBigToSmall;


class runtime {};


#endif