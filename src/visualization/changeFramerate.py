#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Import everything needed to edit video clips 
import cv2
from dotenv import dotenv_values 
import os

# helper function to read in a video file, extract frames so as to play the given number of frames from the original video within the given duration at the new framerate
def changeFramerate(videoFile, newFramerate, numFrames, duration, newVideoFile):
    # Create a VideoCapture object and read from input file
    cap = cv2.VideoCapture(videoFile)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    # Get the frames per second
    fps = cap.get(cv2.CAP_PROP_FPS)
    # Get the number of frames
    numFramesOrig = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    # Get the width and height of frame
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(newVideoFile, fourcc, newFramerate, (width, height))
    # Calculate the number of frames to keep
    numFramesToKeep = int(newFramerate*duration)
    # Calculate the number of frames to skip
    numFramesToSkip = int(numFramesOrig/numFramesToKeep)
    # Loop through the frames
    for i in range(numFramesToKeep):
        # Set the frame to read
        frameToRead = i*numFramesToSkip
        # Set the frame to read
        cap.set(cv2.CAP_PROP_POS_FRAMES, frameToRead)
        # Read the frame
        ret, frame = cap.read()
        # Write the frame to the new video
        out.write(frame)
    # Release everything if job is finished
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    return

if __name__ == "__main__":
    # Read in environment variables
    config = dotenv_values(".env")
    # Setup filepaths for reading data
    filepath = config["currentDirectory"] + "data/verificationData/controlTest20/"
    # Read in video file
    videoFile = filepath + "fullAssemblySimulation_ROMPC__r60_0008.mp4"
    # Define new framerate
    newFramerate = 60
    # Define duration of video
    duration = 3
    # Define number of frames to keep
    numFrames = 3000
    # Define new video file
    newVideoFile = filepath + "realTimeSim.mp4"
    # Change framerate and save new video
    changeFramerate(videoFile, newFramerate, numFrames, duration, newVideoFile)
    print("New video saved as realTimeSim.avi")
    exit()