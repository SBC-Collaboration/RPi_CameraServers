#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# The capture core of the RPi camera server.
# Incoporates frame sync, motion detection, 
# and returns ring buffer when requested

import arducam_mipicamera as arducam
import v4l2
from datetime import datetime
import os
import numpy as np
from PIL import Image
import time
#import RPi.GPIO as GPIO
import ctypes as ct
from count import count_above
import os
import json

class CaptureCore:
    def __init__(self):
        self.config_path = "config.json"
        self.load_config(self.config_path)
        self.init_camera()
        
    def load_config(self, config_path):
        print("Loading config")
        with open(config_path) as f:
            self.config = json.load(f)
        for k in self.config.keys():
            print("{:<15s} {:<10s}".format(k+":",repr(self.config[k])))
    
    def init_camera(self):
        self.camera = arducam.mipi_camera()
        self.camera.init_camera()
        
        print("Camera open")
        self.camera.set_resolution(*self.resolution)
        self.camera.set_mode(5)
        self.camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        self.camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,1)
        print("Camera set")

    def set_controls(self):
        try:
            print("Enable Auto Exposure...")
            self.camera.software_auto_exposure(enable = True)
        except Exception as e:
            print(e)

    def capture(self):
        frame = self.camera.capture(encoding = self.image_format)
        d = datetime.now().strftime(self.date_format)
        path = os.path.join(self.save_path, d+self.image_format)
        open(path, "wb")
        frame.as_array.tofile(path)
        #Remove frame from memory
        del frame

if __name__ == "__main__":
    c = CaptureCore()
