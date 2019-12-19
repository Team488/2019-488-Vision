# You should replace these 3 lines with the output in calibration step
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
import glob

from common import splitfn

DIM=(3264, 2448)
K=np.array([[2263.308139034971, 0.0, 1568.2338734705725], [0.0, 2264.562609500345, 1260.8733211258661], [0.0, 0.0, 1.0]])
D=np.array([[-0.08229109707395792], [-0.11081410511303218], [0.28965062617980036], [-0.28489728805138115]])
def undistort(img_path):
    path, name, ext = splitfn(img_path)
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    outputpath = './output/'
    outfile = os.path.join(outputpath, name+ '_undistorted.png')
    cv2.imwrite(outfile, undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    images = glob.glob('WIN*.jpg')
    for p in images:
        undistort(p)
       