import cv2, os
from helpers.log import *

def extract_frames(video_path, output_folder):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(ERROR+"Can't open video file")
        return
    os.makedirs(output_folder, exist_ok=True)
    
    print(f"{LOG} Frames are being extracted. Please wait!")
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        output_path = os.path.join(output_folder, f"frame_{frame_count}.jpg")
        cv2.imwrite(output_path, frame)
        frame_count += 1
    cap.release()
    print(f"{LOG} Frames extracted: {frame_count}")
    print(f"{SUCCESS} Frames saved in: {output_folder}")