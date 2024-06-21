import os

with open('.env', 'r') as fh:
    vars_dict = dict(
        tuple(line.replace('\n', '').split('='))
        for line in fh.readlines() if not line.startswith('#')
    )

os.environ.update(vars_dict)

CALIBRATION_DATA_DIR = str(os.getenv("calibration_data_dir")).replace("'", "")
CHECKERBOARD_LENGTH = int(os.getenv("checkerboard_length"))
CHECKEBOARD_WIDTH = int(os.getenv("checkerboard_width"))
CHECKERBOARD_SIZE = (CHECKERBOARD_LENGTH, CHECKEBOARD_WIDTH)
SQUARE_SIZE = float(os.getenv("square_arm_size"))

FL=float(os.getenv("FL"))
FY=float(os.getenv("FY"))
FX=float(os.getenv("FX"))
NYU_DATA=False
FINAL_HEIGHT=int(os.getenv("FINAL_HEIGHT"))
FINAL_WIDTH=int(os.getenv("FINAL_WIDTH"))