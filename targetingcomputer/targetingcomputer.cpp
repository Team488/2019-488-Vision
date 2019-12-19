#include "targetingcomputer.h"

Mat src; Mat src_gray; Mat canny_output; Mat drawing;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

const double PI = 3.14159265358;

void TargetingComputer::threshold_video(Mat& frame, Mat& mask) 
{
	medianBlur(frame, mask, 5);
	cvtColor(mask, mask, COLOR_BGR2HSV);
	inRange(mask, lower_color, upper_color, mask);
}

Mat TargetingComputer::getContours(Mat& frame, Mat& mask)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_KCOS, Point(0, 0));
	int centerX = floor(resolutionH / 2); //-0.5?
	int centerY = floor(resolutionV / 2); //-0.5?
	Mat image = frame.clone();

	//TODO add a largest calculator
	if (contours.size() != 0) {
		findTargets(contours, image, centerX, centerY);
	}
	return image;
}

void TargetingComputer::findTargets(vector<vector<Point>> contours, Mat& image, int centerX, int centerY) 
{
	vector<TargetData> targets;
	vector<ContourData> biggestContours;
	
	if (contours.size() > 2) 
	{
		//Sort contours by size
		sort(contours.begin(), contours.end(), contourBigToSmall);
	}

	//cout << "Contours Sorted" << endl;
	//TODO: Track the biggest contours?
	vector<Moments> mnts(contours.size());
	vector<Point> hull;

	double hullarea;
	double area;
	double rotation, pitch, yaw;
	Point conCenter;

	for (int i = 0; i < contours.size(); i++) 
	{
		mnts[i] = moments(contours[i], false);
		convexHull(contours[i],hull, false);
		area = contourArea(contours[i], false);
		hullarea = contourArea(hull, false);
		
		if (area > minContourSize && hullarea > minHullSize)
		{
			//Get the centroids of this contour.
			if (mnts.at(i).m00 != 0)
			{
				conCenter.x = (int)(mnts.at(i).m10 / mnts.at(i).m00);
				conCenter.y = (int)(mnts.at(i).m01 / mnts.at(i).m00);
			} else {
				conCenter.x = conCenter.y = 0;
			}

			//Fit ellipse and get rotation
			rotation = getEllipseRotation(image, contours[i]);

			//Get pitch and yaw to contour
			yaw = calculateYaw(conCenter.x, centerX, lensFocalLength);
			pitch = calculatePitch(conCenter.y, centerY, lensFocalLength);

			//Draw Contour and Center
			//TODO: FIX ELLIPSE DRAWING
			//drawContours(image, contours[i], -1, (23,184, 80));
			circle(image, conCenter, 6, (255,255,255));

			ContourData output;
			output.center = conCenter;
			output.rotation = rotation;
			output.contour = contours[i];

			//if (find(biggestContours.begin(), biggestContours.end(), output) == biggestContours.end()) {
				//output contour is not in the list already!
				
			biggestContours.push_back(output);
			//}
	
		}
	}

		//Sort the set of contours from the left to the right in image plane
	if (biggestContours.size() > 2)
	{
		sort(biggestContours.begin(), biggestContours.end(), contourDataLeftToRight);
		
		//Look at pairs of contours to see if they have the proper orientation
		for (int i = 0; i < biggestContours.size()-1; i++) 
		{
			double tilt1 = biggestContours[i].rotation;
			double tilt2 = biggestContours[i+1].rotation;
			
			Point center1 = biggestContours[i].center;
			Point center2 = biggestContours[i+1].center;

			if (sgn(tilt1) != sgn(tilt2) && tilt1 < 0 && tilt2 > 0)
			{
				TargetData output;
				output.center = ((center1+center2)/2);
				output.yaw = calculateYaw(output.center.x, centerX, lensFocalLength);
				targets.push_back(output);
				//biggest and smallest compare
			}
		}
		
		if (targets.size() > 0) 
		{
			//There are targets!
			sort(targets.begin(), targets.end(), targetDataSmallestYaw);
			TargetData finalTarget = targets[0];
			
			int centerOfTarget = finalTarget.center.x;
			int frameHeight = image.size().height;
			
			yawOut = finalTarget.yaw;
			
			line(image, Point(centerOfTarget, frameHeight), Point(centerOfTarget, 0), Scalar(255, 0, 0), 2);
			// double currentAngleError = findTargets.yaw; // This gets passed somewhere
		}
	}
	// return image here but method isn't a Mat type;
}	

//calculateDistance+ellipse approximation
double TargetingComputer::translateRotation(double rotation, int width, int height)
{
	if (width < height) {
		rotation = -1 * (rotation - 90);
	}
	if (rotation > 90) {
		rotation = -1 * (rotation - 180);
	}
	rotation *= -1;
	return round(rotation);
}

double TargetingComputer::calculateDistance(double heightOfCamera, double heightOfTarget, double pitch)
 {
	double heightOfTargetFromCamera = heightOfTarget - heightOfCamera;

	/*
	d = distance 
	h = heightOfTargetFromCamera
	a = angle = pitch

                        /|
                       / |
                      /  |h
                     /a  |
              camera -----
                       d

	Using Tangent, distance can be found
	*/

	return fabs(heightOfTargetFromCamera / tan((PI * pitch)/180));
}

double TargetingComputer::calculateYaw(int pixelX, int centerX, double hFocalLength)
{
	double yaw = (atan((pixelX - centerX) / hFocalLength)) * 180 / PI;
	return round(yaw);
}

double TargetingComputer::calculatePitch(int pixelY, int centerY, double vFocalLength) 
{
	double arcTan = atan((pixelY - centerY) / vFocalLength);
	double pitch = (180 / PI) * arcTan;
	
	/*
	If the code doesn't work, try this: 
	pitch *= -1;
	*/

	return round(pitch);
}

double TargetingComputer::getEllipseRotation(Mat& image, vector<Point> contour)
{
	double rotation;
	double width, height;
	if (contour.size() < 5) return 0.0; //Better way?
	try {
		RotatedRect ellipse = fitEllipse(contour);

		rotation = ellipse.angle;
		width = ellipse.size.width, height = ellipse.size.height;
		rotation = translateRotation(rotation, width, height);
		
		int smallerSide;
		if (width > height) {
			smallerSide = height;
		} else {
			smallerSide = width;
		}
		cv::ellipse(image, ellipse, Scalar(23, 184, 80), 3);

		return rotation;
	} catch (exception e) {
		RotatedRect rect = minAreaRect(contour);
		boxPoints(rect, contour);
		rotation = rect.angle;
		width = rect.size.width, height = rect.size.height;
		rotation = translateRotation(rotation, width, height);
	}
	return rotation;
}

void TargetingComputer::solveforPnP(Mat& srcIn, const CornersParameters& params, Mat& rvec, Mat& tvec)
{
	vector<Point2f> corners;
	vector<Point3f> objectPoints;
	Mat 	cameraMatrix = (Mat_<double>(3, 3) << 1377.2489869703666, 0.0, 954.3594888938399, 0.0, 1372.6876247853452, 519.7568828166236, 0.0, 0.0, 1.0), 
			distCoeffs = (Mat_<double>(5, 1) << 0.16230294913890325, -0.5637642312820847, -0.006238902102087386, -0.0033466813142761054, 0.585443988714991); // Read in distortion coeffs from somewhere
	
	retrieveCorners(src_gray, objectPoints, params);	
	cv::solvePnP(objectPoints, srcIn, cameraMatrix, distCoeffs, rvec, tvec);
}

void TargetingComputer::retrieveCorners(Mat& src_grayIn, vector<Point3f>& corners, const CornersParameters& params)
{
	goodFeaturesToTrack(src_grayIn, corners, params.maxCorners, params.qualityLevel, params.minDistance, params.blockSize, params.gradientSize, true, params.k);
}

int main(int argc, char** argv)
{
	CameraSettings settings;
	bool ping = true;
	
	settings.exposure;
	settings.contrast;
	settings.brightness;
	settings.gain;
	settings.saturation;

    std::string arg = argv[1];
    int cameraNum = 0;
    try {
        std::size_t pos;
        cameraNum = std::stoi(arg, &pos);
        if (pos < arg.size()) {
            std::cerr << "Trailing characters after number: " << arg << '\n';
        }
    } catch (std::invalid_argument const &ex) {
        std::cerr << "Invalid number: " << arg << '\n';
    } catch (std::out_of_range const &ex) {
        std::cerr << "Number out of range: " << arg << '\n';
    }
	//Open default camera
	VideoCapture cap(cameraNum);

	if (cap.isOpened() == false)
	{
		cout << "Cannot open camera" << endl;
		cin.get();
		return -1;
	}

    // cap.set(CAP_PROP_FRAME_WIDTH, 800);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 600);

	double dWidth = cap.get(CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT);

	cout << "Resolution is: " << dWidth << " x " << dHeight << endl;

	string rv_name = "Raw Video";
	string tc_name = "Targeting Computer";
	string tm_name = "Targeting Mask";

	namedWindow(rv_name);
	namedWindow(tc_name);
	namedWindow(tm_name);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat frame;
	Mat mask;

	const int MAX_EXPOSURE = 10;
	const int MAX_CONTRAST = 255;
	const int MAX_BRIGHTNESS = 100;
	const int MAX_GAIN = 255;
	const int MAX_SATURATION = 255;

	int exposure = 0;
	createTrackbar("Exposure:", rv_name, &exposure, MAX_EXPOSURE);
	
	int contrast = 0;
	createTrackbar("Contrast:", rv_name, &contrast, MAX_CONTRAST);

	int brightness = 0;
	createTrackbar("Brightness:", rv_name, &brightness, MAX_BRIGHTNESS);

	int gain = 0;
	createTrackbar("Gain:", rv_name, &gain, MAX_GAIN);

	int saturation = 0;
	createTrackbar("Saturation:", rv_name, &saturation, MAX_SATURATION);

	TargetingComputer targetingComputer;

	zsock_t *socket = zsock_new_pub("tcp://*:5801");//"ipc://example.sock");
	assert(socket);

	zsock_set_conflate(socket, 1);

	while (true && !zsys_interrupted)
	{
		if (exposure != cap.get(CAP_PROP_EXPOSURE)) {
			cap.set(CAP_PROP_EXPOSURE, exposure - 10);
		}
		if (contrast != cap.get(CAP_PROP_CONTRAST)) {
			cap.set(CAP_PROP_CONTRAST, contrast);
		}
		if (brightness != cap.get(CAP_PROP_BRIGHTNESS)) {
			cap.set(CAP_PROP_BRIGHTNESS, brightness);
		}
		if (gain != cap.get(CAP_PROP_GAIN)) {
			cap.set(CAP_PROP_GAIN, gain);
		}
		if (saturation != cap.get(CAP_PROP_SATURATION)) {
			cap.set(CAP_PROP_SATURATION, saturation);
		}
		
		bool bSuccess = cap.read(src); // read a new frame from video 

		//Breaking the while loop if the frames cannot be captured
		if (bSuccess == false)
		{
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			break;
		}

		mask = src.clone();

		targetingComputer.threshold_video(src, mask);
		Mat processed = targetingComputer.getContours(src, mask);
		float yawOut = targetingComputer.getTargetYaw();

		// if (ping) zsock_send(socket, "s", "PING??????????????????????????");
		// else zsock_send(socket, "s", "PONG!!!!!!!!!!!!!!!!!!!!!!");
		// ping = !ping;
		//zsock_send(socket, "si", "YAW", (int)(yawOut*100000));
		zsock_send(socket, "s", to_string(yawOut).c_str());
		cout << to_string(yawOut).c_str() << endl;

		//show the frame in the created window
		imshow(rv_name, src);
		imshow(tc_name, processed);
		imshow(tm_name, mask);

		//wait for for 10 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		//If the any other key is pressed, continue the loop 
		//If any key is not pressed withing 10 ms, continue the loop 
		if (waitKey(10) == 27)
		{
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
	}

	zsock_destroy(&socket);
	return 0;
}
