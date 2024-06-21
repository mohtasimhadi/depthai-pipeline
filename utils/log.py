RED         = '\033[91m'
GREEN       = '\033[92m'
BLUE        = '\033[94m'
YELLOW      = '\033[93m'
RESET       = '\033[0m'
ERROR       = RED    +  "[Error]    "   +RESET
WARNING     = YELLOW +  "[Warning]  "   +RESET
LOG         = BLUE   +  "[Log]      "   +RESET
SUCCESS     = GREEN  +  "[Success]  "   +RESET

def no_camera_found():
    print(f"{WARNING}No devices found!")
    exit()