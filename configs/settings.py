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

# Median filter
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7
MEDIAN   = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7

# Input/Output
OUTPUT_DIR = os.getenv("OUT")