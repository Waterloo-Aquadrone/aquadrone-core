# DOCUMENTATION
#   Purpose: Break video files into seperate frames
#   How to Use:
#       1 - Set video variable to the of relative path of the video file
#       2 - (Optional) Change the name of the directory you want to
#           save the framse to by changing the 'directory_name'
    


import cv2
import os

def FrameCapture(path, directory):
    vidObj = cv2.VideoCapture(path)
    count = 0
    success = True

    while success:
        success, image = vidObj.read()
        cv2.imwrite(directory +'/frame%d.jpg' % count, image)
        count += 1

if __name__ == '__main__':
    # Name of the directory storing the frames
    directory_name = 'frames'
    # Relative file path of the video
    video = './video/ros.mp4'

    try:
        # Make sure the directory does not already exist
        os.mkdir(directory_name)
        FrameCapture(video, directory_name)
    except OSError:
        print('Creation of directory %s failed' % directory_name)
    else:
        print('Successfully created the directory %s ' % directory_name)
