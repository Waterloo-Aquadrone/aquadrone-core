# DOCUMENTATION
#   Purpose: Break video files into seperate frames
# Command Line Input:
#   python video_frames.py [directory path to store frames] [path to video file]

import cv2
import os
import sys

def FrameCapture(path, directory):
    vidObj = cv2.VideoCapture(path)
    vidObj_length = int(vidObj.get(cv2.CAP_PROP_FRAME_COUNT)) + 1
    print("Number of frames in {}: {}".format(path, vidObj_length))

    padding = len(str(vidObj_length))
    count = 0
    success = True

    while success:
        success, image = vidObj.read()
        cv2.imwrite(directory +'/frame{}.jpg'.format(str(count).rjust(padding,'0')), image)
        count += 1

if __name__ == '__main__':
    directory_name = str(sys.argv[1])
    video = str(sys.argv[2])

    try:
        if( not os.path.exists(directory_name)):
            os.makedirs(directory_name)
        FrameCapture(video, directory_name)
    except Exception as e:
        print('Creation of directory {}  failed'.format(directory_name))
        print(str(e))
    else:
        print('Successfully created the directory {} '.format(directory_name))
