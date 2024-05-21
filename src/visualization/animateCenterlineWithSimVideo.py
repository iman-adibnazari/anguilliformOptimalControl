#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation
from dotenv import dotenv_values 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

import cv2



config = dotenv_values(".env")
# Filepaths for centerline data
filepath = config["currentDirectory"] +"data/centerlineData/" 
filenamePrefix = "centerlineExporter_step_" 
filenameSuffix = ".npy"

# Filepaths for input data
filepath_inputs = config["currentDirectory"] +"data/inputData/" 
filenamePrefix_inputs = "inputExporter_step_" 
filenameSuffix_inputs = ".npy"

# Filepath for video
filepath_video = config["currentDirectory"] +"data/videos/twoSegment__r60_0001.mp4" 
# initialize variables for reading video frames
cap = cv2.VideoCapture(filepath_video) 
# check if capture was successful
if not cap.isOpened(): 
    print("Could not open!")
else:
    print("Video read successful!")
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS)
    print('Total frames: ' + str(total_frames))
    print('width: ' + str(width))
    print('height: ' + str(height))
    print('fps: ' + str(fps))

# configure video to be saved
savePath = config["currentDirectory"] +"data/visualizations/" 
# files = dir_list = os.listdir(filepath)
numFiles = 800#np.size(files) 
print(numFiles)  
# Setup figure for plotting data
fig = plt.figure(figsize=(6, 8), dpi=300)
gs = mpl.gridspec.GridSpec(3,2,wspace=0.5,hspace=0.5,width_ratios=[2, 2],height_ratios=[1, 1,1])
ax = fig.add_subplot(gs[1,:])
ax1=fig.add_subplot(gs[2,0])
ax2=fig.add_subplot(gs[2,1])
ax3 = fig.add_subplot(gs[0,:])  


# duration = 5
# fps = numFiles/duration
fps = 40
duration = numFiles/fps

# find limits for plotting 
filename = filepath + filenamePrefix + "0" + filenameSuffix
# Read in data from file  
data = np.array(np.load(filename))  
xs = data[:,0]
ys = data[:,1]
zs = -data[:,2] 

xlim_max0 = np.max(xs)
xlim_min0 = np.min(xs)
ylim_max0 = np.max(ys)
ylim_min0 = np.min(ys)
zlim_max0 = np.max(zs)
zlim_min0 = np.min(zs)

xlim_max = xlim_max0 + (xlim_max0-xlim_min0)*0.1 
xlim_min = xlim_min0 - (xlim_max0-xlim_min0)*0.1 
ylim_max = ylim_max0 + (ylim_max0-ylim_min0)*0.1 
ylim_min = ylim_min0 - (ylim_max0-ylim_min0)*0.1 
zlim_max = zlim_max0 + (zlim_max0-zlim_min0)*10
zlim_min = zlim_min0 - (zlim_max0-zlim_min0)*10


ulim_max = 0.2
ulim_min = -0.2

# Initialize arrays for holding input data
filename_inputs = filepath_inputs + filenamePrefix_inputs + "0" + filenameSuffix_inputs


inputData = np.array([np.array(np.load(filename_inputs)).flatten()])
# function that draws each frame of the animation
def animate(t):
    global inputData
    global cap
    i = t*fps
    filename = filepath + filenamePrefix + int(round(i)).__str__() + filenameSuffix
    filename_inputs = filepath_inputs + filenamePrefix_inputs + int(round(i)).__str__() + filenameSuffix_inputs

    # Read in data from files
    data = np.array(np.load(filename))  
    print(i)
    xs = data[:,0]
    ys = data[:,1]
    zs = -data[:,2]

    newInputData = np.array([np.array(np.load(filename_inputs)).flatten()]) 
    inputData = np.append(inputData,newInputData,axis=0)

    # plot centerline data 
    ax.clear()
    ax.scatter(xs, zs, marker='o',linewidths=0.1)
    # ax.axis('off')
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("z [mm]")

    ax.set_xlim(xlim_min,xlim_max)
    ax.set_ylim(zlim_min,zlim_max)



    # Plot segment 0 pressures 
    ax1.clear()
    ax1.plot(inputData[0:-1,0], inputData[0:-1,1],color = 'blue')
    ax1.scatter(inputData[-1,0], inputData[-1,1], marker='o',color = 'red')
    ax1.set_xlim(ulim_min,ulim_max)
    ax1.set_ylim(ulim_min,ulim_max)
    ax1.set_xlabel(r"$P_{0,c0}$")
    ax1.set_ylabel(r"$P_{0,c1}$")

    # Plot segment 1 pressures
    ax2.clear()
    ax2.plot(inputData[0:-1,2], inputData[0:-1,3],color = 'blue')
    ax2.scatter(inputData[-1,2], inputData[-1,3], marker='o',color = 'red')
    ax2.set_xlim(ulim_min,ulim_max)
    ax2.set_ylim(ulim_min,ulim_max)
    ax2.set_xlabel(r"$P_{1,c0}$")
    ax2.set_ylabel(r"$P_{1,c1}$")

    # Plot simulation image 
    cap.set(1,int(round(i)))
    success = cap.grab()
    ret, image = cap.retrieve()
    # crop image
    crop = image[350:650, :]

    # resize image 
    scale_percent = 300 # percent of original size
    width = int(crop.shape[1] * scale_percent / 100)
    height = int(crop.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(crop, dim, interpolation = cv2.INTER_AREA)

    ax3.clear()
    ax3.imshow(resized)
    ax3.axis('off')
    return mplfig_to_npimage(fig)


# # run the animation
# ani = FuncAnimation(fig, animate, frames=100, interval=100, repeat=False)

# plt.show()

# Save video
animation = VideoClip(animate, duration = duration)
animation.write_gif(savePath + "test.gif",fps=fps)

