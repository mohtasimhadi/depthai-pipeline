import cv2
import numpy as np
from utils.file_processing import get_filenames
from configs import OUTPUT_DIR, FL, BASELINE, INPUT_DIR

def get_SBM_depth(imgL, imgR, out, focal_length, baseline, numDisparities, blockSize):
    stereo = cv2.StereoBM.create(numDisparities=numDisparities, blockSize=blockSize)
    disparity = stereo.compute(imgL, imgR)
    depth = np.array(focal_length*baseline/disparity)
    cv2.imwrite(out, depth)
    return depth

def get_SGBM_depth(imgL, imgR, out, focal_length, baseline, numDisparities, blockSize):
    stereo = cv2.StereoSGBM.create(numDisparities=numDisparities, blockSize=blockSize)
    disparity = stereo.compute(imgL, imgR)
    depth = np.array(focal_length*baseline/disparity)
    cv2.imwrite(out, depth)
    return depth

def get_depth(left_stereo=INPUT_DIR+'/frames/mono1', right_stereo=INPUT_DIR+'/frames/mono2', out=OUTPUT_DIR+'/depth', disparity_algorithm='sgbm', focal_length=FL, baseline=BASELINE, numDisparities = 16, blockSize = 15):
    filenames = get_filenames(left_stereo)
    for filename in filenames:
        filename = filename.split("/")[-1]
        imgL = cv2.imread(left_stereo+'/'+filename)
        imgR = cv2.imread(right_stereo+'/'+filename)
        if disparity_algorithm == 'sgbm':
            get_SGBM_depth(imgL=imgL, imgR=imgR, out=out+'/'+filename, focal_length=focal_length, baseline=baseline, numDisparities=numDisparities, blockSize=blockSize)
        else:
            get_SBM_depth(imgL=imgL, imgR=imgR, out=out+'/'+filename, focal_length=focal_length, baseline=baseline, numDisparities=numDisparities, blockSize=blockSize)