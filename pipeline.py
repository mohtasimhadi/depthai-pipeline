import os, json
import depthai as dai
from host_sync import HostSync
from datetime import timedelta

output_dir = 'depthai_output'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

def timeDeltaToMilliS(td):
    return td.total_seconds() * 1000

import depthai as dai

pipeline = dai.Pipeline()

monoL_pipeline = pipeline.create(dai.node.MonoCamera)
monoL_pipeline.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoL_pipeline.setBoardSocket(dai.CameraBoardSocket.CAM_B)

monoR_pipeline = pipeline.create(dai.node.MonoCamera)
monoR_pipeline.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoR_pipeline.setBoardSocket(dai.CameraBoardSocket.CAM_C)

depth_pipeline = pipeline.create(dai.node.StereoDepth)
depth_pipeline.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth_pipeline.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth_pipeline.setLeftRightCheck(False)
depth_pipeline.setExtendedDisparity(False)

depth_pipeline.setSubpixel(False)
monoL_pipeline.out.link(depth_pipeline.left)
monoR_pipeline.out.link(depth_pipeline.right)

encoder_depth = pipeline.create(dai.node.VideoEncoder)
encoder_depth.setDefaultProfilePreset(monoL_pipeline.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)
depth_pipeline.disparity.link(encoder_depth.input)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
encoder_depth.bitstream.link(xout_depth.input)

color_pipeline = pipeline.create(dai.node.ColorCamera)
color_pipeline.setBoardSocket(dai.CameraBoardSocket.CAM_A)
color_pipeline.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
color_pipeline.setFps(30)

color_encoder = pipeline.create(dai.node.VideoEncoder)
color_encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
color_pipeline.video.link(color_encoder.input)

xout_color = pipeline.create(dai.node.XLinkOut)
xout_color.setStreamName('color')
color_encoder.bitstream.link(xout_color.input)

monoL_encoder = pipeline.create(dai.node.VideoEncoder)
monoL_encoder.setDefaultProfilePreset(monoL_pipeline.getFps(), dai.VideoEncoderProperties.Profile.H264_MAIN)
monoL_pipeline.out.link(monoL_encoder.input)

monoR_encoder = pipeline.create(dai.node.VideoEncoder)
monoR_encoder.setDefaultProfilePreset(monoR_pipeline.getFps(), dai.VideoEncoderProperties.Profile.H264_MAIN)
monoR_pipeline.out.link(monoR_encoder.input)

xout_monoL = pipeline.create(dai.node.XLinkOut)
xout_monoL.setStreamName('monoL')
monoL_encoder.bitstream.link(xout_monoL.input)

xout_monoR = pipeline.create(dai.node.XLinkOut)
xout_monoR.setStreamName('monoR')
monoR_encoder.bitstream.link(xout_monoR.input)

imu_pipeline = pipeline.create(dai.node.IMU)
xout_imu   = pipeline.createXLinkOut()
xout_imu.setStreamName("imu")
imu_pipeline.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 500)
imu_pipeline.setBatchReportThreshold(1)
imu_pipeline.setMaxBatchReports(30)
imu_pipeline.out.link(xout_imu.input)

with dai.Device(pipeline) as device:
    queues = []
    queues.append(device.getOutputQueue(name="depth", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="color", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="monoL", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="monoR", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="imu", maxSize=30, blocking=True))
    imu_queue = device.getOutputQueue(name='imu', maxSize=30, blocking=True)
    sync = HostSync()

    calibData = device.readCalibration()
    
    calib_dict = {
        'left_intrinsics'           : calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, dai.Size2f(1080, 720)),
        'right_intrinsics'          : calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, dai.Size2f(1080, 720)),
        'rgb_intrinsics'            : calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, dai.Size2f(1920, 1080)),
        'left_distortion'           : calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B),
        'right_distortion'          : calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C),
        'rgb_distortion'            : calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A),
        'extrinsics_left_to_right'  : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C),
        'extrinsics_right_to_left'  : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_B),
        'extrinsics_left_to_rgb'    : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A),
        'extrinsics_right_to_rgb'   : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_A)
    }
    file_calibration = os.path.join(output_dir, 'calibration.json')
    with open(file_calibration, 'w') as f:
        json.dump(calib_dict, f, indent=4)

    imu_list = []
    # baseTs = None

    file_color = open(os.path.join(output_dir, 'color.h265'), 'wb')
    file_monoL = open(os.path.join(output_dir, 'monoL.h264'), 'wb')
    file_monoR = open(os.path.join(output_dir, 'monoR.h264'), 'wb')
    file_depth = open(os.path.join(output_dir, 'depth.mjpeg'), 'wb')
    file_times = open(os.path.join(output_dir, 'timestamps.txt'), 'wb')
    file_imus = open(os.path.join(output_dir, 'imu_data.txt'), 'wb')
    print("Press Ctrl+C to stop encoding...")
    try:
        while True:
            imu_message = imu_queue.get()
            if imu_message is not None:
                imu_packets = imu_message.packets
                for imu_packet in imu_packets:
                    acceleroValues = imu_packet.acceleroMeter
                    gyroValues = imu_packet.gyroscope
                    acceleroTs = acceleroValues.getTimestampDevice()
                    gyroTs = gyroValues.getTimestampDevice()

                    imu_data = {
                        "acceleroMeter" : {
                            "x": acceleroValues.x,
                            "y": acceleroValues.y,
                            "z": acceleroValues.z,
                            "timestamp": acceleroTs
                        },
                        "gyroscope"     : {
                            "x": gyroValues.x,
                            "y": gyroValues.y,
                            "z": gyroValues.z,
                            "timestamp": gyroTs
                        }
                    }
                    file_imus.write(("{'imu_timestamp': '" + str(imu_message.getTimestampDevice()) + "',    'IMU': " + str(imu_data) + '\n').encode())

            for queue in queues:
                new_message = queue.get()
                message = sync.add_msg(queue.getName(), new_message)
                if message:
                    file_depth.write(message['depth'].getData())
                    file_color.write(message['color'].getData())
                    # message['color'].getData().tofile(file_color)
                    file_monoL.write(message['monoL'].getData())
                    file_monoR.write(message['monoR'].getData())
                    # print(str(message['monoR'].getData()))
                    file_times.write(("{'camera_timestamp': '" + str(message['depth'].getTimestampDevice()) + "', 'frame_number': " + str(message['depth'].getSequenceNum())+"}\n").encode())
    except KeyboardInterrupt:
        pass