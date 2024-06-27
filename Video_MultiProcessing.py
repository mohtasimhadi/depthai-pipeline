import depthai as dai
import threading
import time
import os
import pandas as pd

def record_video(device_id, mxid):
    pipeline = dai.Pipeline()

    
    camRgb = pipeline.create(dai.node.ColorCamera)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    
    ve1 = pipeline.create(dai.node.VideoEncoder)
    ve2 = pipeline.create(dai.node.VideoEncoder)
    ve3 = pipeline.create(dai.node.VideoEncoder)

    imu = pipeline.create(dai.node.IMU)

    ve1Out = pipeline.create(dai.node.XLinkOut)
    ve2Out = pipeline.create(dai.node.XLinkOut)
    ve3Out = pipeline.create(dai.node.XLinkOut)
    xlinkOut = pipeline.create(dai.node.XLinkOut)

    ve1Out.setStreamName('ve1Out')
    ve2Out.setStreamName('ve2Out')
    ve3Out.setStreamName('ve3Out')
    xlinkOut.setStreamName("imu")

    # Properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    monoLeft.setCamera("left")
    monoRight.setCamera("right")

    # Create encoders, one for each camera, consuming the frames and encoding them using H.264 / H.265 encoding
    ve1.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H264_MAIN)
    ve2.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    ve3.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H264_MAIN)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW, dai.IMUSensor.MAGNETOMETER_RAW], 30)

    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    monoLeft.out.link(ve1.input)
    camRgb.video.link(ve2.input)
    monoRight.out.link(ve3.input)
    ve1.bitstream.link(ve1Out.input)
    ve2.bitstream.link(ve2Out.input)
    ve3.bitstream.link(ve3Out.input)
    imu.out.link(xlinkOut.input)

    base_folder = "input/lab_data/"
    video_folder = os.path.join(base_folder, "video")
    if not os.path.exists(video_folder):
        os.makedirs(video_folder)
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        def timeDeltaToMilliS(delta) -> float:
            return delta.total_seconds()*1000
        
        calibData = device.readCalibration()

        outQ1 = device.getOutputQueue(name='ve1Out', maxSize=30, blocking=True)
        outQ2 = device.getOutputQueue(name='ve2Out', maxSize=30, blocking=True)
        outQ3 = device.getOutputQueue(name='ve3Out', maxSize=30, blocking=True)
        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        baseTs = None

        with open(os.path.join(video_folder, f'{mxid}_mono1.h264'), 'wb') as fileMono1H264, \
             open(os.path.join(video_folder, f'{mxid}_color.h265'), 'wb') as fileColorH265, \
             open(os.path.join(video_folder, f'{mxid}_mono2.h264'), 'wb') as fileMono2H264, \
             open(os.path.join(video_folder, f'{mxid}_imu_data.txt'), 'wb') as fileIMUTData:
            #  open(f'{timestamp}_{product_name}_{mxid}_color.h265', 'wb') as fileColorH265, \
            
            df = pd.DataFrame(columns=[
                "camera_id", "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "FOV"
            ])

            # Loop through cameras (CAM_A, CAM_B, CAM_C)
            for camera_socket in [dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C]:
                # Get intrinsics and distortion coefficients
                intrinsics = calibData.getCameraIntrinsics(camera_socket)
                distortion_coeffs = calibData.getDistortionCoefficients(camera_socket)
                fov = calibData.getFov(camera_socket)

                # Create a new row for each camera with data
                new_row = {
                    "camera_id": camera_socket.name,
                    "fx": intrinsics[0][0],
                    "fy": intrinsics[1][1],
                    "cx": intrinsics[2][0],
                    "cy": intrinsics[2][1],
                    "k1": distortion_coeffs[0],
                    "k2": distortion_coeffs[1],
                    "p1": distortion_coeffs[2],
                    "p2": distortion_coeffs[3],
                    "FOV": fov
                }

                # Append the new row to the DataFrame
                df = pd.concat([df, pd.DataFrame.from_records([new_row])], ignore_index=True)

                # Print or use the DataFrame containing data for all cameras
                print(df)
                df.to_csv(os.path.join(video_folder, f'{mxid}_calibration_data.csv'))


            print(f"Device {device_id} (MXID: {mxid}): Press Ctrl+C to stop encoding...")
            while True:
                try:
                    # Empty each queue
                    while outQ1.has():
                        outQ1.get().getData().tofile(fileMono1H264)

                    while outQ2.has():
                        outQ2.get().getData().tofile(fileColorH265)

                    while outQ3.has():
                        outQ3.get().getData().tofile(fileMono2H264)
                    
                    while imuQueue.has():
                        imuData = imuQueue.get()
                        imuPackets = imuData.packets
                        for imuPacket in imuPackets:
                            acceleroValues = imuPacket.acceleroMeter
                            gyroValues = imuPacket.gyroscope

                            acceleroTs = acceleroValues.getTimestampDevice()
                            gyroTs = gyroValues.getTimestampDevice()
                            if baseTs is None:
                                baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                            acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
                            gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

                            imuF = "{:.06f}"
                            tsF  = "{:.03f}"

                            fileIMUTData.write(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms\n".encode())
                            fileIMUTData.write(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}\n".encode())
                            fileIMUTData.write(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms\n".encode())
                            fileIMUTData.write(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)}\n\n".encode())
                except KeyboardInterrupt:
                    # Keyboard interrupt (Ctrl + C) detected
                    break

            print(f"Device {device_id} (MXID: {mxid}): Recording stopped.")

if __name__ == "__main__":
    # List of cameras with their respective MXIDs and product names
    camera_info = [
        {"mxid": "18443010518B880E00"},
        {"mxid": "184430105185341300"}#Utilising both Products of the OAK-D-PRO
    ]
    t1 = threading.Thread(target=record_video, args=(1, '18443010518B880E00'))
    t2 = threading.Thread(target=record_video, args=(1, '184430105185341300'))
    #t2 = threading.Thread(target=record_video, args=(1, '19443010517FFD1200', 'OAK-D-S2-FF'))

    t1.start()
    time.sleep(2)
    t2.start()
    
    t1.join()
    t2.join()

    print("All recordings stopped.")