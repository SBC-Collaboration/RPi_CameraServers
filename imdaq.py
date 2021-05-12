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
import RPi.GPIO as GPIO
import ctypes as ct
from count import count_above
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
    
    def init_gpio(self):
        # using bcm mode ("GPIO #" number, not physical pin number)
        GPIO.setmode(GPIO.BCM)
        input_pins = self.config["input_pins"]
        GPIO.setup(input_pins["state_com"],GPIO.IN)
        
    
    def init_camera(self):
        self.camera = arducam.mipi_camera()
        self.camera.init_camera()
        
        print("Camera open")
#        if self.config["frame_sync"]:
#            regs = self.config["registers"]
#            for reg in regs:
#                self.camera.write_sensor_reg(reg[0],reg[1])
        self.camera.set_resolution(*self.config["resolution"])
        self.camera.set_mode(11)
        self.camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        self.camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        print("Camera set")

    def init_threading(self):
        start_proc = threading.Event()
        next_frame = threading.Event()
        start_cap = threading.Event()

    def capture(self):
        frame = self.camera.capture(encoding = self.image_format)
        d = datetime.now().strftime(self.date_format)
        path = os.path.join(self.save_path, d+self.image_format)
        open(path, "wb")
        frame.as_array.tofile(path)
        #Remove frame from memory
        del frame
    
    def continuous_capture(self, t=1):
        bu = (ct.c_ubyte*1024000) * 100
        self.buffer = bu()
        frame = self.camera.capture(encoding="raw", quality=90)
        frame.buffer_ptr[0].data = self.buffer[0]
        t_overall = time.time()
        t_start = time.time()
        t_end = t_start + t
        i = 0
        capture_time = []
        save_time = []
        
        image_format = arducam.IMAGE_FORMAT(arducam.image_encodings["raw"], 50)
        
        try:
            while (time.time()<t_end):
                if i==100: 
                    i = 0
                    capture_time = np.array(capture_time)
                    print("FPS: {:3.1f}, {:3d}".format(100/(time.time()-t_overall),
                                                       len(capture_time[capture_time>0.011])))
                    capture_time = []
                    t_overall = time.time()
                else: 
                    t_start=time.time()
                    frame = self.camera.capture(encoding="raw")
                    t_start = time.time()
                    self.buffer[i] = ct.cast(frame.buffer_ptr[0].data, ct.POINTER(ct.c_ubyte*1024000)).contents
                    save_time.append(time.time()-t_start)
                    #del frame
                    i+=1
        except KeyboardInterrupt:
                print('Interrupted')
        
        for j in range(self.config["frames_after"]):
            i+=1
            if(i==100):
                i=0
            frame = self.camera.capture(encoding="raw")
            self.buffer[i] = ct.cast(frame.buffer_ptr[0].data, ct.POINTER(ct.c_ubyte*1024000)).contents
        
        print("Remaining frames taken.")
        t = time.time()
        self.buffer = np.roll(self.buffer, -(i+1), axis=0)
        
        self.camera.close_camera()
        print("Camera closed. Saving images . . .")
        
        for i in range(self.config["max_frames"]//5):
            im = Image.fromarray(np.reshape(self.buffer[i],
                                            tuple(np.array(self.config["resolution"][::-1]))))
            im = im.convert("L")
            im.save(self.config["save_path"]+"Buffer-"+"{:02}".format(i)+self.config["image_format"])
        
        print("Images saved")
        
if __name__ == "__main__":
    c = CaptureCore()
    c.continuous_capture(t=120)