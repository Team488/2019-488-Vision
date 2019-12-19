import os
import numpy as np
import cv2
import glob
import codecs, json

# default values for calibration
imagePath = 'images'
badImagePath = imagePath + '\\bad\\'
fileExt = 'jpg'
chessboardWidth = 7
chessboardHeight = 6

with open('calibration_settings.json') as settings_file:
        dataJSON = json.load(settings_file)
        chessboardJSON = dataJSON['ChessboardSize']
        chessboardWidth = chessboardJSON['width']
        chessboardHeight = chessboardJSON['height']
        imageJSON = dataJSON['Image']
        imagePath = imageJSON['path']
        badImagePath = imagePath + '\\bad\\'
        fileExt = imageJSON['file_ext']

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardWidth * chessboardHeight, 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardWidth, 0:chessboardHeight].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(imagePath + '\\*.' + fileExt)
# we assert here because the program will crash without images
assert images, 'No images found in \\' + imagePath + '\\'

i = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (chessboardWidth, chessboardHeight),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        i += 1
        print("Images processed with good data: " + str(i))
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
    else:
        # move images that aren't good to a subdirectory in the image path called bad
        if not os.path.exists(badImagePath):
            os.mkdir(badImagePath)
        fileName = fname[len(imagePath + '\\'):len(fname)]
        os.rename(fname, badImagePath + fileName)
        print('Moved: ' + fname + ' to ' + badImagePath + fileName)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

calibrationJSON =  {
                "CameraMatrix": json.dumps(mtx.tolist()),
                "DistortionCoeff": json.dumps(dist.tolist())
        }

json.dump(calibrationJSON, codecs.open('camera_calib.json', 'w', encoding='utf-8'), separators=(',', ':'), sort_keys=True, indent=4)

cv2.destroyAllWindows()
