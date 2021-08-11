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
import python.count as count
import os
import json
import multiprocessing as mp
from multiprocessing.pool import ThreadPool
import pandas as pd
import shutil

# set to lowest niceness for highest priority
pid = os.getpid()
os.system("sudo renice -n -20 -p " + str(pid))

class CaptureCore:
    def __init__(self):
        print("Loading config . . .")
        self.load_config()
        self.init_gpio()
        self.init_buffer()
        self.init_multiprocessing()

    # first try to load config file from remote directory
    # falls back to local config if remote not found
    def load_config(self):
        with open("config.json") as f:
            self.config = json.load(f)
        try:
            with open(self.config["config_path"]) as f:
                self.config = json.load(f)
            shutil.copy(self.config["config_path"],"config.json")
        except FileNotFoundError:
            print("Remote config file not found. Using default file.")
            with open("config.json") as f:
                self.config = json.load(f)

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
        GPIO.setup(self.output_pins["state"],GPIO.OUT, initial=0)
        GPIO.setup(self.output_pins["trig"],GPIO.OUT, initial=0)
        GPIO.output(self.output_pins["state"],GPIO.LOW)
        GPIO.output(self.output_pins["trig"],GPIO.LOW)

    # initialize image buffer and camera info buffer
    def init_buffer(self):
        self.timestamp = mp.Array("d", np.zeros(self.buffer_len))
        self.pts = mp.Array(ct.c_uint64, np.zeros(self.buffer_len, dtype=np.uint64))
        self.timediff = mp.Array(ct.c_uint64, np.zeros(self.buffer_len, dtype=np.uint64))
        self.skipped = mp.Array(ct.c_uint64, np.zeros(self.buffer_len, dtype=int))
        self.pixdiff = mp.Array("i", np.zeros(self.buffer_len, dtype=int))
        self.ind = mp.Value("i", 0)
        self.raw_arr = mp.RawArray(ct.c_uint8, self.buffer_len*self.res[0]*self.res[1])
        self.buffer = np.frombuffer(self.raw_arr, dtype=np.uint8).reshape([self.buffer_len,*self.res])
        print("Buffer initialized.")

    # initialize camera instance
    def init_camera(self):
        self.camera = arducam.mipi_camera()
        self.camera.init_camera()
        
        print("Camera open.")
        self.config_camera()

    # load camera configurations
    def config_camera(self):
        self.camera.set_resolution(*self.res)
        
        # use mode 5 or 11 for 1280x800 2lane raw8 capture
        self.camera.set_mode(self.config["mode"])
        self.camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        self.camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        print("Camera set.")

    # initiate events and processes for multiprocessing
    def init_multiprocessing(self):
        self.frame_taken = mp.Event()
        self.buffer_copied = mp.Event()
        self.capture_process = mp.Process(target=self.capture)
        self.detection_process = mp.Process(target=self.detect_motion)
        self.trigger_latched = mp.Value("b", False)
        print("Multiprocessing initialized.")

    # captures one frame and save to disk
    def capture_frame(self):
        self.load_config()
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        
        # capture single image and save to current save directory
        frame = self.camera.capture(encoding = "raw")
        buffer = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res[::-1])
        im = Image.fromarray(buffer).convert("L")
        d = datetime.now().strftime(self.config["date_format"])
        img_path = self.config["cam_name"]+"-"+d+self.config["image_format"]
        path = os.path.join(self.config["save_path"], img_path)        
        im.save(path)

        print("Image captured: %s"%img_path)

    # image capturing process
    def capture(self):
        # create camera instance, and initialize
        camera = arducam.mipi_camera()
        camera.init_camera()
        
        print("Camera open.")
        camera.set_resolution(*self.config["resolution"])
        # use mode 5 or 11 for 1280x800 2lane raw8 capture
        camera.set_mode(self.config["mode"])
        camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        print("Camera set.\n")

        t_overall = time.time()

        # loop when trigger not latched
        while not self.trigger_latched.value:
            i = self.ind.value

            # capture a frame in continuous capture
            frame = camera.capture(encoding="raw", quality = 90)
            self.timestamp[i] = time.time()

            # wait for detection thread to finish copying from buffer
            self.buffer_copied.wait()
            self.buffer_copied.clear()

            # presentaion timestamp
            self.pts[i] = frame.buffer_ptr[0].pts
            # diff in pts from last frame
            self.timediff[i] = self.pts[i] - self.pts[i-1]
            # number of frame skipped from last frame, gives 2ms margin
            self.skipped[i] = int((self.timediff[i]-2000)/10000)
            # saves image to buffer
            self.buffer[i] = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res)
            self.ind.value = (i+1) % self.buffer_len
            self.frame_taken.set()

            # print FPS every time buffer is filled
            if i==0:
                fps = self.buffer_len/(time.time()-t_overall)
                if fps <500: # hide first fps number
                    print("FPS: {:3.2f}".format(fps), end="\t")
                    print("Dropped: %d"%np.sum(self.skipped), end="\t")
                    print("Motion: {:8d}".format(np.max(self.pixdiff)))
                t_overall = time.time()

        # take remaining frames
        for j in range(self.config["frames_after"]):
            i = self.ind.value
            frame = camera.capture(encoding="raw", quality=90)
            self.timestamp[i] = time.time()
            self.pts[i] = frame.buffer_ptr[0].pts
            self.timediff[i] = self.pts[i] - self.pts[i-1]
            self.skipped[i] = int((self.timediff[i]-2000)/10000)
            self.buffer[i] = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res)
            self.ind.value = (i+1) % self.buffer_len
        
        print("Remaining frames taken.", end=" ")
        # roll buffer position so the last taken image is positioned last
        i = self.ind.value
        self.buffer[:] = np.roll(self.buffer, -i, axis=0)

        camera.close_camera()
        print("Camera closed.")

    def detect_motion(self):
        # create two local buffer of two frames
        frame1 = np.zeros(self.config["resolution"], dtype=np.uint8)
        frame2 = np.zeros(self.config["resolution"], dtype=np.uint8)
        # tells capturing process that buffer copying is complete
        self.buffer_copied.set()

        # loop until it receives trigger latch from FIFO
        while not self.trigger_latched.value:
            self.frame_taken.wait()
            self.frame_taken.clear()

            i = self.ind.value
            # copy the previous two frames from shared buffer to local buffer
            np.copyto(frame1, self.buffer[i-2])
            np.copyto(frame2, self.buffer[i-1])
            self.buffer_copied.set()

            # cython code for subtracting frames and count pixels above threshold
            counter = count.diff_count(frame1, frame2, self.config["adc_threshold"])
            self.pixdiff[i] = counter

            if counter>self.config["pix_threshold"] and GPIO.input(self.config["input_pins"]["trig_en"]):
                # send out a pulse of 100 us
                GPIO.output(self.config["output_pins"]["trig"],GPIO.HIGH)
                time.sleep(0.0001)
                GPIO.output(self.config["output_pins"]["trig"],GPIO.LOW)
                print("Detected motion: %d.\t Trigger sent."%counter)

    def save_frame(self, i):
        # saves each from to file
        im = Image.fromarray(self.buffer[i]).convert("L")
        filename = self.config["save_path"]+self.config["cam_name"]+"-img{:02}".format(i)+self.config["image_format"]
        im.save(filename)

    def save_images(self):
        print("Saving images . . .")
        t_overall = time.time()
        i = self.ind.value

        # organize camera info to dataframe, and roll to the correct index
        self.caminfo = pd.DataFrame({
            "timestamp": np.array(self.timestamp), 
            "pts": np.array(self.pts), 
            "timediff": np.array(self.timediff), 
            "skipped": np.array(self.skipped), 
            "pixdiff": np.array(self.pixdiff)})
        self.caminfo["timestamp"] = np.roll(self.caminfo["timestamp"], -i)
        self.caminfo["pts"] = np.roll(self.caminfo["pts"], -i)
        self.caminfo["timediff"] = np.roll(self.caminfo["timediff"], -i)
        self.caminfo["skipped"] = np.roll(self.caminfo["skipped"], -i)
        self.caminfo["pixdiff"] = np.roll(self.caminfo["pixdiff"], -i)
        self.caminfo.to_csv(self.config["save_path"]+self.config["cam_name"]+"-info.csv", float_format="%.9f")
        
        # reshapes image buffer and saves to disk
        self.buffer = np.reshape(self.buffer,tuple(np.array([self.buffer_len, self.res[1], self.res[0]])))
        # pool = ThreadPool(4)
        # pool.map(self.save_frame, range(self.buffer_len))
        for i in range(self.buffer_len):
            im = Image.fromarray(self.buffer[i]).convert("L")
            filename = self.config["save_path"]+self.config["cam_name"]+"-img{:02}".format(i)+self.config["image_format"]
            im.save(filename)

        print("Images saved. Time: %.0fs.\n"%(time.time()-t_overall))

    def start_event(self, t=10):
        self.init_gpio()
        self.init_camera()
        print("Waiting for event to start . . .")
        #while not GPIO.input(self.input_pins["state_com"]):
        while True:
            time.sleep(0.001)
            # use trig_en when not in event to capture a single frame
            if GPIO.input(self.input_pins["trig_en"]):
                c.capture_frame()

        self.camera.close_camera()

        print("Loading config . . .")
        self.load_config()
        self.init_buffer()
        self.init_multiprocessing()

        t_overall = time.time()
        GPIO.output(self.output_pins["state"], GPIO.HIGH)

        try:
            # start processes
            self.capture_process.start()
            self.detection_process.start()
            print("Processes started.\n")

            time.sleep(1)
            while not GPIO.input(self.input_pins["trig_latch"]):
            # and time.time()-t_overall<t:
                time.sleep(0.001)

            self.trigger_latched.value = True
            print("\nTrigger latched.")

        except KeyboardInterrupt:
                print('Interrupted.')

        # wait for both processes to finish
        self.capture_process.join()
        self.detection_process.join()

        self.save_images()
        GPIO.output(self.output_pins["state"], GPIO.LOW)
        GPIO.cleanup()
        
if __name__ == "__main__":
    c = CaptureCore()
    while True:
        c.start_event(t=1200)
    GPIO.cleanup()
