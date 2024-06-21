import argparse
from utils.log import *
from camera import stream_pointcloud
from helpers.frame_extraction import extract_frames
from point_cloud.offline import generate_pointclouds
from calibration import calibrate_camera

functions = {
    "calibrate"         :   (calibrate_camera, []),
    "stream"            :   (stream_pointcloud, []),
    "frame_extraction"  :   (extract_frames, ['video_path', 'output_dir']),
    "offline_pcl"       :   (generate_pointclouds, ['img_path', 'output_dir', 'default_model', 'model_dir'])
}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Data Collection Pipeline")
    parser.add_argument('--operation', choices=functions.keys(), help=f'Function to call. Select from {functions.keys()}')

    for param_name in set(param_name for func, param_names in functions.values() for param_name in param_names):
        parser.add_argument(f'--{param_name}', help=f'Parameter {param_name}')

    args = parser.parse_args()
    
    if args.operation is None or args.operation ==  '':
        print(f"{ERROR}No operation is selected! Try running-\n{LOG+GREEN}python main.py{RESET} --operation <operation name>\n{LOG}Valid operations are {YELLOW}{', '.join(map(str, functions.keys()))}{RESET}")
        exit()

    func, param_names = functions[args.operation]

    func_args = []
    for param_name in param_names:
        param_value = getattr(args, param_name)
        if param_value is not None:
            func_args.append(param_value)
    
    func(*func_args)