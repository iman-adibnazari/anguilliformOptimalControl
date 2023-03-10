#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dotenv import dotenv_values 
# importing movie py libraries
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

# setup filepaths
config = dotenv_values(".env")
filepath = config["currentDirectory"] +"data/inputData/" 
filenamePrefix = "inputExporter_step_" 
filenameSuffix = ".npy"

savePath = config["currentDirectory"] +"data/visualizations/" 
files = dir_list = os.listdir(filepath)
numFiles = 100#np.size(files) 

# 


