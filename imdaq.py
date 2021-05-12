#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# The capture core of the RPi camera server.
# Incoporates frame sync, motion detection, 
# and returns ring buffer when requested

import MIPI_Camera.RPI.python.arducam_mipicamera as arducam
import v4l2
from datetime import datetime
import os
import sys
import numpy as np
from PIL import Image
import time
import RPi.GPIO as GPIO
import ctypes as ct
from python.count import count_above
import os
import json
from multiprocessing import Process
import threading
camera_lib = ct.cdll.LoadLibrary("libarducam_mipicamera.so")

class CaptureCore:
    def __init__(self):
        self.config_path = "config.json"
        self.load_config(self.config_path)
        self.init_camera()
        self.init_gpio()
        self.init_threading()
        
    def load_config(self, config_path):
        print("Loading config")
        with open(config_path) as f:
            self.config = json.load(f)
        for k in self.config.keys():
            print("{:<15s} {:<10s}".format(k+":",repr(self.config[k])))
        # size of each frame in buffer
        self.frame_size = np.product(self.config["resolution"])
        self.res = self.config["resolution"]
    
    def init_gpio(self):
        # using bcm mode ("GPIO #" number, not physical pin number)
        GPIO.setmode(GPIO.BCM)
        input_pins = self.config["input_pins"]
        GPIO.setup(input_pins["state_com"],GPIO.IN)
        
    
    def init_camera(self):
        self.camera = arducam.mipi_camera()
        self.camera.init_camera()
        
        print("Camera open")
        self.camera.set_resolution(*self.res)
        # use mode 5 or 11 for 1280x800 2lane raw8 capture
        self.camera.set_mode(self.config["mode"])
        self.camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        self.camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        print("Camera set")

    def init_threading(self):
        start_proc = threading.Event()
        next_frame = threading.Event()
        start_cap = threading.Event()

    def capture(self):
        # capture single image
        frame = self.camera.capture(encoding = self.image_format)
        d = datetime.now().strftime(self.date_format)
        path = os.path.join(self.save_path, d+self.image_format)
        open(path, "wb")
        frame.as_array.tofile(path)
        #Remove frame from memory
        del frame

    def capture_frame(self, i):
        # capture a frame in continuous capture
        if i==self.config["max_frames"]: 
            i = 0
        else: 
            frame = self.camera.capture(encoding="raw")
            self.buffer[i] = ct.cast(frame.buffer_ptr[0].data, ct.POINTER(ct.c_ubyte*self.frame_size)).contents
            i+=1
        return i

    def take_remaining_images(self, i):
        for j in range(self.config["frames_after"]):
            i+=1
            if(i==self.config["max_frames"]):
                i=0
            frame = self.camera.capture(encoding="raw")
            self.buffer[i] = ct.cast(frame.buffer_ptr[0].data, ct.POINTER(ct.c_ubyte*self.frame_size)).contents
        
        print("Remaining frames taken.")
        # roll buffer position so the last taken image is positioned last
        self.buffer = np.roll(self.buffer, -(i+1), axis=0)
    
    def save_images(self):
        t_overall = time.time()
        self.buffer = np.reshape(self.buffer,tuple(np.array([self.config["max_frames"], self.res[1], self.res[0]])))
        for i in range(self.config["max_frames"]):
            #im = Image.fromarray(np.reshape(self.buffer[i],tuple(np.array(self.res[::-1]))))
            #im = im.convert("L")
            im = Image.fromarray(self.buffer[i]).convert("L")
            im.save(self.config["save_path"]+"Buffer-"+"{:02}".format(i)+self.config["image_format"])
        print("Images saved. Time: %.3fs"%(time.time()-t_overall))

    def continuous_capture(self, t=1):
        # initialize c-type buffer
        bu = (ct.c_ubyte*self.frame_size) * self.config["max_frames"]
        self.buffer = bu()
        # capture a frame first for comparison
        frame = self.camera.capture(encoding="raw", quality=90)
        frame.buffer_ptr[0].data = self.buffer[0]
        t_end = time.time() + t
        t_overall = time.time()
        i = 0
        
        try:
            while (time.time()<t_end):
                i = self.capture_frame(i)
                # print FPS every time buffer is filled
                if i==0:
                    print("FPS: {:3.1f}".format(self.config["max_frames"]/(time.time()-t_overall)))
                    t_overall = time.time()
        except KeyboardInterrupt:
                print('Interrupted')
        
        self.take_remaining_images(i)
        self.camera.close_camera()
        print("Camera closed. Saving images . . .")
        self.save_images()
        
if __name__ == "__main__":
    c = CaptureCore()
    c.continuous_capture(t=120)