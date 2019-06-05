# DOCUMENTATION
#   Purpose: Break video files into seperate frames
# Command Line Input:
#   python video_frames.py [directory path to store frames] [path to video file]

import cv2
import os
import sys

def FrameCapture(path, directory):
    vidObj = cv2.VideoCapture(path)
    vidObj_length = int(vidObj.get(cv2.CAP_PROP_FRAME_COUNT))
    print("# of Frames: {}".format(vidObj_length))
    print("FPS: {}".format(vidObj.get(cv2.CAP_PROP_FPS)))

    padding = len(str(vidObj_length))
    count = 0

    while(vidObj.isOpened() and count < vidObj_length):
        success, image = vidObj.read()
        name = directory +'/frame{}.jpg'.format(str(count).rjust(padding,'0'))
        cv2.imwrite(name, image)
        print('Creating...{} -> {}'.format(name,success))
        count += 1

if __name__ == '__main__':
    directory_name = str(sys.argv[1])
    video = str(sys.argv[2])

    try:
        if( not os.path.exists(directory_name)):
            os.makedirs(directory_name)
        FrameCapture(video, directory_name)
    except Exception as e:
        print("ERROR: ", str(e))
    else:
        print('Completed processing video {}'.format(video))
