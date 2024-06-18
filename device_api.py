import depthai as dai
from configs.settings import *

pipeline = dai.Pipeline()

with dai.Device(pipeline, dai.DeviceInfo(CAM_1)) as device:
    print("MxId:", device.getDeviceInfo().getMxId())
    print('USB speed:',device.getUsbSpeed())
    print('Connected cameras:',device.getConnectedCameras())