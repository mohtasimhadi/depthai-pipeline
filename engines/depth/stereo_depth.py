import cv2
import numpy as np
from utils.file_processing import get_filenames
from configs import OUTPUT_DIR, FL, BASELINE

def get_SBM_depth(imgL, imgR, out, focal_length, baseline, numDisparities, blockSize):
    stereo = cv2.StereoBM.create(numDisparities=numDisparities, blockSize=blockSize)
    disparity = stereo.compute(imgL, imgR)
    depth = np.array(focal_length*baseline/disparity)
    cv2.imwrite(out+imgL.split("/")[-1], depth)
    return depth

def get_SGBM_depth(imgL, imgR, out, focal_length, baseline, numDisparities, blockSize):
    stereo = cv2.StereoSGBM.create(numDisparities=numDisparities, blockSize=blockSize)
    disparity = stereo.compute(imgL, imgR)
    depth = np.array(focal_length*baseline/disparity)
    cv2.imwrite(out+imgL.split("/")[-1], depth)
    return depth

def get_depth(left_stereo, right_stereo, out=OUTPUT_DIR, disparity_algorithm='sgbm', focal_length=FL, baseline=BASELINE, numDisparities = 16, blockSize = 15):
    filenames = get_filenames(left_stereo)
    for filename in filenames:
        filename = filename.split("/")[-1]
        if disparity_algorithm == 'sgbm':
            return get_SGBM_depth(imgL=left_stereo+'/'+filename, imgR=right_stereo+'/'+filename, out=out, focal_length=focal_length, baseline=baseline, numDisparities=numDisparities, blockSize=blockSize)
        else:
            return get_SBM_depth(imgL=left_stereo+'/'+filename, imgR=right_stereo+'/'+filename, out=out, focal_length=focal_length, baseline=baseline, numDisparities=numDisparities, blockSize=blockSize)