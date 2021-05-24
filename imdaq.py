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
import pigpio as pg
import ctypes as ct
import python.count as count
import os
import json
import multiprocessing as mp
camera_lib = ct.cdll.LoadLibrary("libarducam_mipicamera.so")

class CaptureCore:
    def __init__(self):
        self.config_path = "config.json"
        self.load_config(self.config_path)
        self.init_camera()
        self.init_gpio()
        self.init_multiprocessing()
        
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
        # using BCM numbering: "GPIO #" number, not physical pin number
        self.input_pins = self.config["input_pins"]
        self.output_pins = self.config["output_pins"]
        # start daemon by pigpiod or
        # sudo systemctl enable pigpiod.service
        self.gpio = pg.pi()
        self.gpio.set_mode(self.input_pins["state_com"],pg.INPUT)
        self.gpio.set_mode(self.input_pins["trig_en"],pg.INPUT)
        self.gpio.set_mode(self.input_pins["trig_latch"],pg.INPUT)
        self.gpio.set_mode(self.output_pins["state"],pg.OUTPUT)
        self.gpio.set_mode(self.output_pins["trig"],pg.OUTPUT)
    
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

    def init_multiprocessing(self):
        self.manager = mp.Manager()
        self.lock = mp.Lock()
        i = mp.Value("i", 0)
        self.capture_process = mp.Process(target=self.capture_frame, args=(i, self.lock))
        self.detection_process = mp.Process(target=self.detect_motion, args=(i, self.lock))
        self.next_frame = mp.Event()

    def capture(self):
        # capture single image
        frame = self.camera.capture(encoding = self.image_format)
        d = datetime.now().strftime(self.date_format)
        path = os.path.join(self.save_path, d+self.image_format)
        open(path, "wb")
        frame.as_array.tofile(path)
        #Remove frame from memory
        del frame

    def capture_frame(self, i, lock):
        # capture a frame in continuous capture
        frame = self.camera.capture(encoding="raw", quality = 90)
        self.timestamps[i] = time.time()
        # presentaion timestamp
        # pts = frame.buffer_ptr[0].pts
        with lock:
            self.buffer[i] = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res)

    def take_remaining_images(self, i):
        for j in range(self.config["frames_after"]):
            frame = self.camera.capture(encoding="raw")
            self.timestamps[i] = time.time()
            self.buffer[i] = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res)
            i = (i+1) % self.buffer_len
        
        print("\nRemaining frames taken.")
        # roll buffer position so the last taken image is positioned last
        self.buffer = np.roll(self.buffer, -i, axis=0)
        self.timestamps = np.roll(self.timestamps, -i)

    def detect_motion(self, i, lock):
        with lock:
            frame1 = self.buffer[i-1]
            frame2 = self.buffer[i]
        # cython code for subtracting frames and count pixels above threshold
        counter = count.diff_count(frame1, frame2, self.config["adc_threshold"])
        if counter>self.config["pix_threshold"] and self.gpio.read(self.input_pins["trig_en"]):
            # send out a pulse of 1 us
            self.gpio.gpio_trigger(self.output_pins["trig"],1,1)
            print("Detected motion: %d.\t Trigger sent."%counter)

    def save_images(self):
        t_overall = time.time()
        # save timestamps of images
        np.savetxt(self.config["save_path"]+"timestamps.csv", self.timestamps, fmt="%.9f")
        self.buffer = np.reshape(self.buffer,tuple(np.array([self.buffer_len, self.res[1], self.res[0]])))

        for i in range(self.buffer_len):
            im = Image.fromarray(self.buffer[i]).convert("L")
            im.save(self.config["save_path"]+"Buffer-"+"{:02}".format(i)+self.config["image_format"])
        print("Images saved. Time: %.3fs"%(time.time()-t_overall))

    def start_event(self, t=1):
        # initialize buffer
        self.buffer = np.zeros((self.buffer_len, *self.res),dtype=np.uint8)
        self.timestamps = np.zeros(100)
        
        t_end = time.time() + t
        t_overall = time.time()
        i = 0
        
        try:
            while (time.time()<t_end):
                if self.gpio.read(self.input_pins["trig_latch"]):
                    # quit loop when trigger is latched
                    print("Trigger latched.")
                    break
                
                # take a frame
                self.capture_frame(i, self.lock)
                self.detect_motion(i, self.lock)
                i = (i+1) % self.buffer_len
                
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
