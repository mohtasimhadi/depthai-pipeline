import cv2
import glob
import numpy as np
import os
import torch
import open3d as o3d
from PIL import Image

from depth_anything_v2.dpt import DepthAnythingV2
from utils.log import *
from configs.depth import MODEL_CONFIGS, DEFAULT_MODEL, MODEL_DIR
from configs.calibration import FL, FY, FX, NYU_DATA, FINAL_HEIGHT, FINAL_WIDTH
from configs.settings import RGB_IMAGE_DIR, OUTPUT_DIR


def get_model(default_model, model_dir):
    depth_anything = DepthAnythingV2(**MODEL_CONFIGS[default_model])
    depth_anything.load_state_dict(torch.load(model_dir, map_location='cpu'))
    depth_anything = depth_anything.to('cuda').eval()
    return depth_anything

def get_filenames(img_path):
    if os.path.isfile(img_path):
        if img_path.endswith('txt'):
            with open(img_path, 'r') as f:
                filenames = f.read().splitlines()
        else:
            filenames = [img_path]
    else:
        filenames = glob.glob(os.path.join(img_path, '**/*'), recursive=True)
    filenames.sort()
    return filenames

def generate_pointclouds(img_path = RGB_IMAGE_DIR, output_dir=OUTPUT_DIR, default_model=DEFAULT_MODEL, model_dir=MODEL_DIR):
    depth_anything = get_model(default_model=default_model, model_dir=model_dir)
    filenames = get_filenames(img_path = img_path)

    for k, filename in enumerate(filenames):
        print(f'{LOG}Progress {GREEN}{k+1}/{len(filenames)}{RESET}: {filename}')

        color_image = Image.open(filename).convert('RGB')

        image = cv2.imread(filename)
        pred = depth_anything.infer_image(image, FINAL_HEIGHT)

        # Resize color image and depth to final size
        resized_color_image = color_image.resize((FINAL_WIDTH, FINAL_HEIGHT), Image.LANCZOS)
        resized_pred = Image.fromarray(pred).resize((FINAL_WIDTH, FINAL_HEIGHT), Image.NEAREST)

        focal_length_x, focal_length_y = (FX, FY) if not NYU_DATA else (FL, FL)
        x, y = np.meshgrid(np.arange(FINAL_WIDTH), np.arange(FINAL_HEIGHT))
        x = (x - FINAL_WIDTH / 2) / focal_length_x
        y = (y - FINAL_HEIGHT / 2) / focal_length_y
        z = np.array(resized_pred)
        points = np.stack((np.multiply(x, z), np.multiply(y, z), z), axis=-1).reshape(-1, 3)
        colors = np.array(resized_color_image).reshape(-1, 3) / 255.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        filename = filename.split("/")[-1]
        o3d.io.write_point_cloud(f"{output_dir}/{filename}.ply", pcd)
    print(f"{SUCCESS}Pointclouds saved in {output_dir}")

        