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
        print("Loading config . . .")
        with open(config_path) as f:
            self.config = json.load(f)
        for k in self.config.keys():
            print("{:<15s} {:<10s}".format(k+":",repr(self.config[k])))
        print("")
        # size of each frame in buffer
        self.frame_size = np.product(self.config["resolution"])
        self.res = self.config["resolution"]
        self.buffer_len = self.config["buffer_len"]
    
    def init_gpio(self):
        # using bcm mode ("GPIO #" number, not physical pin number)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.input_pins = self.config["input_pins"]
        GPIO.setup(self.input_pins["state_com"],GPIO.IN)
        GPIO.setup(self.input_pins["trig_en"],GPIO.IN)
        GPIO.setup(self.input_pins["trig_latch"],GPIO.IN)
        self.output_pins = self.config["output_pins"]
        GPIO.setup(self.output_pins["state"],GPIO.OUT)
        GPIO.setup(self.output_pins["trig"],GPIO.OUT)
    
    def init_camera(self):
        self.camera = arducam.mipi_camera()
        self.camera.init_camera()
        
        print("Camera open.")
        self.camera.set_resolution(*self.res)
        # use mode 5 or 11 for 1280x800 2lane raw8 capture
        self.camera.set_mode(self.config["mode"])
        self.camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        self.camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        print("Camera set.\n")

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
        frame = self.camera.capture(encoding="raw", quality = 90)
        self.timestamps[i] = time.time()
        self.buffer[i] = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res)
        return (i+1) % self.buffer_len

    def take_remaining_images(self, i):
        for j in range(self.config["frames_after"]):
            frame = self.camera.capture(encoding="raw")
            self.timestamps[i] = time.time()
            self.buffer[i] = ct.cast(frame.buffer_ptr[0].data, ct.POINTER(ct.c_ubyte*self.frame_size)).contents
            i = (i+1) % self.buffer_len
        
        print("\nRemaining frames taken.")
        # roll buffer position so the last taken image is positioned last
        self.buffer = np.roll(self.buffer, -i, axis=0)
        self.timestamps = np.roll(self.timestamps, -i)

    def process(self, i):
        np.subtract(self.buffer[i-1],self.buffer[i-2],out=self.diff_buffer)
        counter = count_above(self.diff_buffer, self.config["adc_threshold"])
        print("counter:  ",counter, end="\t")
        if counter>self.config["pix_threshold"] and GPIO.input(self.input_pins["trig_en"]):
            GPIO.output(self.output_pins["trig"],1)
            print("Detected motion. Trigger sent.")

    def save_images(self):
        t_overall = time.time()
        np.savetxt(self.config["save_path"]+"timestamps.csv", self.timestamps, fmt="%.9f")
        self.buffer = np.reshape(self.buffer,tuple(np.array([self.buffer_len, self.res[1], self.res[0]])))
        
        for i in range(self.buffer_len):
            im = Image.fromarray(self.buffer[i]).convert("L")
            im.save(self.config["save_path"]+"Buffer-"+"{:02}".format(i)+self.config["image_format"])
        print("Images saved. Time: %.3fs"%(time.time()-t_overall))

    def start_event(self, t=1):
        # initialize buffer
        self.buffer = np.zeros((self.buffer_len, *self.res),dtype=np.uint8)
        self.diff_buffer = np.zeros(self.res, dtype=np.uint8)
        self.timestamps = np.zeros(100)
        
        t_end = time.time() + t
        t_overall = time.time()
        i = 0
        
        try:
            while (time.time()<t_end):
                if GPIO.input(self.input_pins["trig_latch"]):
                    # quit loop when trigger is latched
                    print("Trigger latched.")
                    break
                
                # take a frame
                i = self.capture_frame(i)
                #self.process(i)
                
                # print FPS every time buffer is filled
                if i==0:
                    print("FPS: {:3.2f}".format(self.buffer_len/(time.time()-t_overall)))
                    t_overall = time.time()
                
        except KeyboardInterrupt:
                print('Interrupted.')
        
        self.take_remaining_images(i)
        self.camera.close_camera()
        print("Camera closed. Saving images . . .")
        self.save_images()
        
if __name__ == "__main__":
    c = CaptureCore()
    c.start_event(t=120)
