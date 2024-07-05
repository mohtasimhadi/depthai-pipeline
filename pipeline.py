import depthai as dai

pipeline = dai.Pipeline()

monoL = pipeline.create(dai.node.MonoCamera)
monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)

monoR = pipeline.create(dai.node.MonoCamera)
monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)

depth = pipeline.create(dai.node.StereoDepth)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(False)
depth.setExtendedDisparity(False)

depth.setSubpixel(False)
monoL.out.link(depth.left)
monoR.out.link(depth.right)

encoder_depth = pipeline.create(dai.node.VideoEncoder)
encoder_depth.setDefaultProfilePreset(monoL.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)
depth.disparity.link(encoder_depth.input)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
encoder_depth.bitstream.link(xout_depth.input)

color = pipeline.create(dai.node.ColorCamera)
color.setBoardSocket(dai.CameraBoardSocket.CAM_A)
color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
color.setFps(30)

color_encoder = pipeline.create(dai.node.VideoEncoder)
color_encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
color.video.link(color_encoder.input)

xout_color = pipeline.create(dai.node.XLinkOut)
xout_color.setStreamName('color')
color_encoder.bitstream.link(xout_color.input)

monoL_encoder = pipeline.create(dai.node.VideoEncoder)
monoL_encoder.setDefaultProfilePreset(monoL.getFps(), dai.VideoEncoderProperties.Profile.H264_MAIN)
monoL.out.link(monoL_encoder.input)

monoR_encoder = pipeline.create(dai.node.VideoEncoder)
monoR_encoder.setDefaultProfilePreset(monoR.getFps(), dai.VideoEncoderProperties.Profile.H264_MAIN)
monoR.out.link(monoR_encoder.input)

xout_monoL = pipeline.create(dai.node.XLinkOut)
xout_monoL.setStreamName('monoL')
monoL_encoder.bitstream.link(xout_monoL.input)

xout_monoR = pipeline.create(dai.node.XLinkOut)
xout_monoR.setStreamName('monoR')
monoR_encoder.bitstream.link(xout_monoR.input)

with dai.Device(pipeline) as device:

    queue_depth = device.getOutputQueue(name="depth", maxSize=30, blocking=True)
    queue_color = device.getOutputQueue(name="color", maxSize=30, blocking=True)
    queue_monoL = device.getOutputQueue(name="monoL", maxSize=30, blocking=True)
    queue_monoR = device.getOutputQueue(name="monoR", maxSize=30, blocking=True)

    with open('depth.mjpeg', 'wb') as file_depth, open('color.h265', 'wb') as file_color, open('monoL.h264', 'wb') as file_monoL, open('monoR.h264', 'wb') as file_monoR:
        print("Press Ctrl+C to stop encoding...")
        try:
            while True:
                packet_depth = queue_depth.get()
                file_depth.write(packet_depth.getData())
                
                packet_color = queue_color.get()
                packet_color.getData().tofile(file_color)
                
                packet_monoL = queue_monoL.get()
                file_monoL.write(packet_monoL.getData())
                
                packet_monoR = queue_monoR.get()
                file_monoR.write(packet_monoR.getData())
        except KeyboardInterrupt:
            pass

    print("To view the encoded data, convert the stream files into playable video files using the commands below:")
    print("ffmpeg -framerate 30 -i depth.mjpeg -c copy depth.mp4")
    print("ffmpeg -framerate 30 -i color.h265 -c copy color.mp4")
    print("ffmpeg -framerate 30 -i monoL.h264 -c copy monoL.mp4")
    print("ffmpeg -framerate 30 -i monoR.h264 -c copy monoR.mp4")