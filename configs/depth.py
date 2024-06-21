import os

with open('.env', 'r') as fh:
    vars_dict = dict(
        tuple(line.replace('\n', '').split('='))
        for line in fh.readlines() if not line.startswith('#')
    )

os.environ.update(vars_dict)

LRCHECK  = True   			# Better handling for occlusions
EXTENDED = False  			# Closer-in minimum depth, disparity range is doubled
SUBPIXEL = True   			# Better accuracy for longer distance, fractional disparity 32-levels
CONFIDENCE_THRESHOLD = 250 	# 0-255, 255 = low confidence, 0 = high confidence
MIN_RANGE = 10  			# mm
MAX_RANGE = 5000			# mm

MODEL_CONFIGS = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

DEFAULT_MODEL = os.getenv("DEFAULT_MODEL")
MODEL_DIR = os.getenv("MODEL_DIR")