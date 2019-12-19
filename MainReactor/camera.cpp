#include "camera.h"

//TODO: Get the rest of the settings in here
bool Camera::loadCameraSettings (VideoCapture& capture, const string& path) {
    settings = {-488, -488, -488, -488, -488};
    type = NOCAMSET;
    ifstream inputFileStream(path);
    if (inputFileStream.fail()) {
        cerr << "Failed To Open File" << endl;
        return false;
    }
    // Reads all the text from the given file
    string jsonStr((istreambuf_iterator<char>(inputFileStream)),
                    istreambuf_iterator<char>());
    value json;
    string err = parse(json, jsonStr);
    if (!err.empty()) {
        cerr << err << endl;
    }
    //Parse the JSON out
    settings.exposure   = stoi(json.get("Exposure").to_str());
    settings.contrast   = stoi(json.get("Contrast").to_str());
    settings.brightness = stoi(json.get("Brightness").to_str());
    settings.gain       = stoi(json.get("Gain").to_str());
    settings.saturation = stoi(json.get("Saturation").to_str());
    string camType      = json.get("CameraType").to_str();
    
    //Try and set the Cam Type
    if (camType == "RGB") type = RGB;
    else if (camType == "IR") type = IR;
    else if (camType == "DEPTH") type = DEPTH;
    else { type = NOCAMSET; return false; }

    //Try and set the camera settings
    if (settings.exposure != -488)
        capture.set(CAP_PROP_EXPOSURE, settings.exposure);
    if (settings.contrast != -488)
        capture.set(CAP_PROP_CONTRAST, settings.contrast);
	if (settings.brightness != -488)
        capture.set(CAP_PROP_BRIGHTNESS, settings.brightness);
	if (settings.gain != -488)
        capture.set(CAP_PROP_GAIN, settings.gain);
    if (settings.saturation != -488)
        capture.set(CAP_PROP_SATURATION, settings.saturation);

    return true;
}

bool Camera::saveCameraSettings (const string& path, CameraSettings settings) {
    ofstream outputFileStream(path);
    if (outputFileStream.fail()) {
        cerr << "Failed To Open File" << endl;
        return false;
    }

    // Reformatting the Camera Data
    outputFileStream << "{ \n" << 
                        "\t\"Exposure\": "   << settings.exposure   << ",\n" <<
                        "\t\"Contrast\": "   << settings.contrast   << ",\n" <<
                        "\t\"Brightness\": " << settings.brightness << ",\n" << 
                        "\t\"Gain\": "       << settings.gain       << ",\n" <<
                        "\t\"Saturation\": " << settings.saturation << ",\n" <<
                        "\t\"CameraType\": " << "\"" << settings.camType << "\"" << "\n" <<
                        "}";

    return true;
}

bool Camera::saveCameraSettings (const string& path, const VideoCapture& cap, const CameraType& camType)
{
    return saveCameraSettings(path, getCurrentSettings(cap, camType));
}

CameraSettings Camera::getCurrentSettings(const VideoCapture& cap, const CameraType& type)
{
    CameraSettings settings;

    // Getting the Camera Setting Values
    settings.exposure   = cap.get(CAP_PROP_EXPOSURE);
    settings.contrast   = cap.get(CAP_PROP_CONTRAST);
    settings.brightness = cap.get(CAP_PROP_BRIGHTNESS);
    settings.gain       = cap.get(CAP_PROP_GAIN);
    settings.saturation = cap.get(CAP_PROP_SATURATION);
    settings.camType    = type;

    return settings;
}

//One time setup
bool Camera::setup(VideoCapture &capture, Mat &cameraFrame, string path) {
    //if (headerHandle == "-488") return false;
    cout << "Load Settings!" << endl;
    if(!loadCameraSettings(capture, path)) {
        cerr << "Error loading settings!" << endl;
        return false;
    }

    //setup the shared Mat
    cout << "Update the SharedMat" << endl;
    shared.updateMemory(memoryHandle, headerHandle, cameraFrame);

    return true;
}


bool Camera::updateFrame(Mat &cameraFrame) {
    //Unwarp and undistort
    //TODO: Implement this

    //If we get it, apply the filter set for this particular camera
    Mat frameOut = applyFilters(cameraFrame);

    //Pipe it out!
    shared.updateFrame(frameOut);
    if (shared.mat.empty())
    {
        cerr << "ERROR: Can't grab camera frame." << endl;
        return false;
    }

    return true;
}

