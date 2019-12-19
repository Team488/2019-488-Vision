#include "rstc.h"

Mat src; Mat src_gray; Mat canny_output; Mat drawing;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

const double PI = 3.14159265358;

bool display = false;

void TargetingComputer::threshold_RGBvideo(Mat& frame, Mat& mask) 
{
	medianBlur(frame, mask, 5);
	cvtColor(mask, mask, COLOR_BGR2HSV);
	inRange(mask, lower_color, upper_color, mask);
}

void TargetingComputer::threshold_IRvideo(Mat& frame, Mat& mask)
{
	medianBlur(mask, mask, 5);
	threshold(mask, mask, lower_bin, upper_bin, THRESH_BINARY);
}

void TargetingComputer::thresholdIRWithoutBlur(Mat& frame, Mat& mask)
{
	threshold(mask, mask, lower_bin, upper_bin, THRESH_BINARY);
}

Mat TargetingComputer::getContours(Mat& frame, Mat& mask, vector<Point2f>& corners, bool& fixOut)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_KCOS, Point(0, 0));
	int centerX = floor(resolutionH / 2); //-0.5?
	int centerY = floor(resolutionV / 2); //-0.5?
	Mat image = frame.clone();

	//TODO add a largest calculator
	if (contours.size() != 0) {
		fixOut = findTargets(contours, image, corners, centerX, centerY);
	}
	return image;
}

bool TargetingComputer::findTargets(vector<vector<Point>> contours, Mat& image, vector<Point2f>& corners, int centerX, int centerY) 
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
	yawOut = 0.0;

	vector<RotatedRect> minRect( contours.size() );
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
			yaw = calculateYaw(conCenter.x);
			pitch = calculatePitch(conCenter.y);

			//Draw Contour and Center
			//TODO: FIX ELLIPSE DRAWING
			//drawContours(image, contours[i], -1, (23,184, 80));
			//circle(image, conCenter, 6, (255,255,255));

			ContourData output;
			getRotatedRect(image, contours[i], output.rect);
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
	if (biggestContours.size() >= 2)
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
				output.yaw = calculateYaw(output.center.x);
				output.rectOne = biggestContours[i].rect;
				output.rectTwo = biggestContours[i+1].rect;
				//cout << "Center: " << output.center.x << endl;
				//cout << "Yaw: " << output.yaw << endl;
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

			//Corners to solvePnP
			Point2f rectPointsOne[4];
			Point2f rectPointsTwo[4];
			finalTarget.rectOne.points(rectPointsOne);
			finalTarget.rectTwo.points(rectPointsTwo);
			corners.insert(corners.end(), &rectPointsOne[0], &rectPointsOne[4]);
			corners.insert(corners.end(), &rectPointsTwo[0], &rectPointsTwo[4]);

			return true;


			// double currentAngleError = findTargets.yaw; // This gets passed somewhere
		} else {
			return false;
		}
	} else {
		return false;
	}
	// return image here but method isn't a Mat type;
	return false;
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

double TargetingComputer::calculateYaw(int pixelX)
{
	double yaw = ((double)(pixelX - centerX)/(double)centerX)*((double)horizontalView/2.0);
	//double yaw = (atan((pixelX - centerX)/hFocalLength)) * 180 / PI;
	return round(-yaw);
}

double TargetingComputer::calculatePitch(int pixelY) 
{
	// double arcTan = atan((pixelY - centerY) / vFocalLength);
	// double pitch = (180 / PI) * arcTan;
	double pitch = ((pixelY - centerY)/centerY)*(verticalView/2.0);

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
		//cv::ellipse(image, ellipse, Scalar(23, 184, 80), 3);

		return rotation;
	} catch (exception e) {
		RotatedRect rect = minAreaRect(contour);
		rotation = rect.angle;
		width = rect.size.width, height = rect.size.height;
		rotation = translateRotation(rotation, width, height);
	}
	return rotation;
}

double TargetingComputer::getRotatedRect(Mat& image, vector<Point>& contour, RotatedRect& outRect){
	outRect = minAreaRect(contour);
	double rotation = outRect.angle;
	double width = outRect.size.width;
	double height = outRect.size.height;
	rotation = translateRotation(rotation, width, height);
	return rotation;
}

bool TargetingComputer::solveforPnP(Mat& srcIn, const vector<Point2f>& corners, const CornersParameters& params, Mat& rvec, Mat& tvec) {
	//vector<Point3f> objectPoints;
	// Mat	cameraMatrix = Mat::eye(3, 3, CV_32F);
	// Mat distCoeffs = Mat::zeros(5, 1, CV_32F);
	// Mat 	cameraMatrix = (Mat_<double>(3, 3) << 1377.2489869703666, 0.0, 954.3594888938399, 0.0, 1372.6876247853452, 519.7568828166236, 0.0, 0.0, 1.0), 
			// distCoeffs = (Mat_<double>(5, 1) << 0.16230294913890325, -0.5637642312820847, -0.006238902102087386, -0.0033466813142761054, 0.585443988714991); // TODO: FIX THIS Read in distortion coeffs from somewhere
	Mat cameraMatrix = (Mat_<double>(3, 3) << 389.0146686037677, 0.0, 409.155800232932, 0.0, 390.08648648172885, 254.832826921863, 0.0, 0.0, 1.0), 
	 		distCoeffs = (Mat_<double>(5, 1) << -0.003878081470044212, -0.024002777408139987, 0.0026403748239745934, -0.004934124697019439, 0.01575788918960726);
	if (corners.size() == 8) {
		cout << "Got 8 points!" << endl;
		cv::solvePnP(tapePoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
		return true;
	} else
	{
		cout << "Only had " << corners.size() << " points!" << endl;
		return false;
	}
}

bool TargetingComputer::solveforPnP(Mat& srcIn, const CornersParameters& params, Mat& rvec, Mat& tvec)
{
	vector<Point2f> corners;
	//vector<Point3f> objectPoints;
	//Mat	cameraMatrix = Mat::eye(3, 3, CV_32F);
	//Mat distCoeffs = Mat::zeros(5, 1, CV_32F);]
	//This is the actual left IR camera
	 Mat cameraMatrix = (Mat_<double>(3, 3) << 389.0146686037677, 0.0, 409.155800232932, 0.0, 390.08648648172885, 254.832826921863, 0.0, 0.0, 1.0), 
	 		distCoeffs = (Mat_<double>(5, 1) << -0.003878081470044212, -0.024002777408139987, 0.0026403748239745934, -0.004934124697019439, 0.01575788918960726);
	
	retrieveCorners(srcIn, corners, params);
	if (corners.size() == 8) {
		cout << "Got 8 points!" << endl;
		cv::solvePnP(tapePoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
		return true;
	} else
	{
		cout << "Only had " << corners.size() << " points!" << endl;
		return false;
	}
	
}

void TargetingComputer::retrieveCorners(Mat& src_grayIn, vector<Point2f>& corners, const CornersParameters& params)
{
	goodFeaturesToTrack(src_grayIn, corners, params.maxCorners, params.qualityLevel, params.minDistance, Mat(), params.blockSize, params.gradientSize, true, params.k);
}

int main(int argc, char** argv)
{
	string port = "5000";
	string ip = "127.0.0.1";
	bool remoteView = false;
	if (argc == 1) {
		//run as usual
	}
	else if (argc == 3) {
		ip = argv[1];
		port = argv[2];
		remoteView = true;
	} else if (argc == 4) {
		ip = argv[1];
		port = argv[2];
		remoteView = true;
		display = to_bool(string(argv[3]));
	} else {
		cerr << "Error!  Call with morten IP PORT (local view = false)" << endl;
		cerr << "Using defaults - nothing at all" << endl;
	}
	
	CameraSettings settings;
	bool ping = true;
	
	settings.exposure;
	settings.contrast;
	settings.brightness;
	settings.gain;
	settings.saturation;

	int iWidth = 848;
	int iHeight = 480;

	cout << "Resolution is: " << iWidth << " x " << iHeight << endl;

	string rv_name = "Raw Video";
	string tc_name = "Targeting Computer";
	string tm_name = "Targeting Mask";

	if (display) {
		namedWindow(rv_name);
		namedWindow(tc_name);
		namedWindow(tm_name);
	}

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat frame;
	Mat mask;

	TargetingComputer targetingComputer;
	DataToJSON jsonWriter;
	string jsonOut;
	VideoWriter out;
	if (remoteView) {
		int codec = VideoWriter::fourcc('X','2','6','4');
		const cv::String output_pipeline = "appsrc ! video/x-raw, format=(string)BGR width=(int)"+ to_string(iWidth) +", height=(int)"+ to_string(iHeight) +", framerate=(fraction)60/1 ! videoconvert ! omxh264enc quality-level=0 control-rate=2 bitrate=400000 ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! udpsink host="+ip+" port="+port;
		//const cv::String output_pipeline = "appsrc ! video/x-raw, format=(string)BGR width=(int)"+ to_string(iWidth) +", height=(int)"+ to_string(iHeight) +", framerate=(fraction)60/1 ! videoconvert ! x265enc quality-level=0 control-rate=2 ! video/x-h265, stream-format=(string)byte-stream ! h265parse ! rtph265pay ! udpsink host="+ip+" port="+port;
		out = VideoWriter(output_pipeline, codec, 60, Size(iWidth,iHeight));
		if (out.isOpened() == false)
		{
			cout << "Cannot open VideoWriter" << endl;
			return -1;
		}
	}

	zsock_t *socket = zsock_new_pub("tcp://*:5801");//"ipc://example.sock");
	assert(socket);

	zsock_set_conflate(socket, 1);
	// int iWidth = targetingComputer.getHResolution();
	// int iHeight = targetingComputer.getVResolution();

	Mat mask_out;
	Mat corner_mask;
	Mat topMask = cv::Mat::zeros(cv::Size(848, 480), CV_8UC1);
	rectangle(topMask,Point(0,100),Point(848,480),Scalar(255), -1, 8);

	SharedMat ir_mat(RSFRONT_IRONE_MEMORY_NAME, RSFRONT_IRONE_HEADER_NAME);

	Mat rvec, tvec, R, T;
	vector<Point2f> corners;
	bool fixAcquired;
	Status status = inactive;

	while (true && !zsys_interrupted)
	{
		fixAcquired = false;
		status = active;
		int frameNum = ir_mat.waitForFrame();
		corners.clear();
		src = ir_mat.mat;
		
		src.copyTo(corner_mask, topMask);
		src.copyTo(mask, topMask);

		targetingComputer.threshold_IRvideo(src, mask);
		// targetingComputer.thresholdIRWithoutBlur(src, corner_mask);

		Mat processed = targetingComputer.getContours(src, mask, corners, fixAcquired);
		float yawOut = targetingComputer.getTargetYaw();

		if (fixAcquired) status = retro_target_found;

		if (corners.size() == 8) {
			for(int i = 0; i < 8; i++) {
				circle(processed, corners[i], 5, Scalar(255));
			}

			if (targetingComputer.solveforPnP(mask, corners, targetingComputer.getCornerParams(), rvec, tvec))
			{
				cout << "RVEC: " << rvec << endl;
				cout << "TVEC: " << tvec << endl;

				//Calculate camera location to target @ (0,0,0) centered
				Rodrigues(rvec, R);
				R = R.t();
				tvec = -R * tvec;
				T = Mat::eye(4, 4, R.type());
				T( Range(0,3), Range(0,3)) = R * 1;
				T( Range(0,3), Range(3,4)) = tvec * 1;

				cout << "World position of camera to target: " << T << endl;
			}
		}


		jsonOut = jsonWriter.updateJSONToString(fixAcquired, status, yawOut, 0, 0);
		

		// if (ping) zsock_send(socket, "s", "PING??????????????????????????");
		// else zsock_send(socket, "s", "PONG!!!!!!!!!!!!!!!!!!!!!!");
		// ping = !ping;
		//zsock_send(socket, "si", "YAW", (int)(yawOut*100000));
		//zsock_send(socket, "s", to_string(yawOut).c_str());
		zsock_send(socket, "s", jsonOut.c_str());
		//cout << to_string(yawOut).c_str() << endl;

		if(display) {
			//show the frame in the created window
			imshow(rv_name, src);
			imshow(tc_name, processed);
			imshow(tm_name, mask);
		}

		if(remoteView) {
			cvtColor(processed, mask_out, COLOR_GRAY2BGR);
			out.write(mask_out);
		}

		//wait for for 10 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		//If the any other key is pressed, continue the loop 
		//If any key is not pressed withing 10 ms, continue the loop 
		if (waitKey(1) == 27)
		{
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
	}

	zsock_destroy(&socket);
	return 0;
}
