import depthai as dai
from camera.recording import Camera
from typing import List
from helpers.log import *
from point_cloud.point_cloud_visualizer import PointCloudVisualizer

def stream_pointcloud():
    device_infos = dai.Device.getAllAvailableDevices()
    if len(device_infos) == 0:
        print(f"{WARNING}No devices found!")
        exit()
    else:
        print(f"{LOG}Found", len(device_infos), "devices")

    device_infos.sort(key=lambda x: x.getMxId(), reverse=True)

    cameras: List[Camera] = []

    for device_info in device_infos:
        cameras.append(Camera(device_info, len(cameras)+1, show_video=True, show_point_cloud=True))


    PointCloudVisualizer(cameras)