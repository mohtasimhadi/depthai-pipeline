import os
import depthai as dai

with open('.env', 'r') as fh:
    vars_dict = dict(
        tuple(line.replace('\n', '').split('='))
        for line in fh.readlines() if not line.startswith('#')
    )

os.environ.update(vars_dict)

CAM_1 = os.getenv("CAM_1_MXID")
CAM_2 = os.getenv("CAM_2_MXID")

COLOR = True 		# Use color camera of mono camera

# DEPTH CONFIG
lrcheck  = True   			# Better handling for occlusions
extended = False  			# Closer-in minimum depth, disparity range is doubled
subpixel = True   			# Better accuracy for longer distance, fractional disparity 32-levels
confidence_threshold = 250 	# 0-255, 255 = low confidence, 0 = high confidence
min_range = 100 			# mm
max_range = 2000			# mm

# Median filter
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7
median   = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7

# CALIBRATION
CALIBRATION_DATA_DIR = str(os.getenv("calibration_data_dir")).replace("'", "")
CHECKERBOARD_LENGTH = int(os.getenv("checkerboard_length"))
CHECKEBOARD_WIDTH = int(os.getenv("checkerboard_width"))
SQUARE_ARM_SIZE = float(os.getenv("square_arm_size"))

# Input/Output
OUTPUT_DIR = os.getenv("OUT")