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
import socket
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
import logging, logging.handlers as handlers

# set to lowest niceness for highest priority
pid = os.getpid()
os.system("sudo renice -n -20 -p " + str(pid))

class CaptureCore:
    def __init__(self):
        self.init_logging()
        self.load_config()
        self.init_gpio()
        self.init_buffer()
        self.init_multiprocessing()

    def init_logging(self):
        self.cam_name = socket.gethostname()

        logging.basicConfig(level=logging.INFO,
                            format=self.cam_name+': %(message)s')

        fileHandler = handlers.WatchedFileHandler(f"./{self.cam_name}.log")
        fileHandler.setLevel(logging.DEBUG)
        formatter = logging.Formatter(f'{self.cam_name}-%(asctime)s %(message)s', datefmt='%H:%M:%S')
        fileHandler.setFormatter(formatter)
        logging.getLogger('').addHandler(fileHandler)

    def load_config(self):
        logging.info("Loading config . . .")
        with open("config.json") as f:
            self.config = json.load(f)
        try:
            with open(self.config["config_path"]) as f:
                self.config = json.load(f)
            shutil.copy(self.config["config_path"],"config.json")
        except FileNotFoundError:
            logging.error("Remote config file not found. Using default file.")
            with open("config.json") as f:
                self.config = json.load(f)

    def init_gpio(self):
        # using bcm mode ("GPIO #" number, not physical pin number)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.config["state_comm_pin"],GPIO.IN)
        GPIO.setup(self.config["trig_en_pin"],GPIO.IN)
        GPIO.setup(self.config["trig_latch_pin"],GPIO.IN)
        GPIO.setup(self.config["state_pin"],GPIO.OUT, initial=0)
        GPIO.setup(self.config["trig_pin"],GPIO.OUT, initial=0)
        GPIO.output(self.config["state_pin"],GPIO.LOW)
        GPIO.output(self.config["trig_pin"],GPIO.LOW)

    def init_buffer(self):
        # initialize buffer
        if self.config["mode"] in [5,11]:
            self.res = [1280, 800]
            self.frame_size = np.product(self.res)
        else:
            logging.error("Camera mode not supported!")
        self.timestamp = mp.Array("d", np.zeros(self.config["buffer_len"]))
        self.pts = mp.Array(ct.c_uint64, np.zeros(self.config["buffer_len"], dtype=np.uint64))
        self.timediff = mp.Array(ct.c_uint64, np.zeros(self.config["buffer_len"], dtype=np.uint64))
        self.skipped = mp.Array(ct.c_uint64, np.zeros(self.config["buffer_len"], dtype=int))
        self.pixdiff = mp.Array("i", np.zeros(self.config["buffer_len"], dtype=int))
        self.ind = mp.Value("i", -1)
        self.raw_arr = mp.RawArray(ct.c_uint8, self.config["buffer_len"]*self.res[0]*self.res[1])
        self.buffer = np.frombuffer(self.raw_arr, dtype=np.uint8).reshape([self.config["buffer_len"],*self.res])
        logging.info("Buffer initialized.")

    def init_camera(self):
        self.camera = arducam.mipi_camera()
        try:
            self.camera.init_camera()
        except RuntimeError:
            logging.error("Camera not found!")
            return

        logging.info("Camera open.")
        self.camera.set_resolution(*self.res)
        
        # use mode 5 or 11 for 1280x800 2lane raw8 capture
        self.camera.set_mode(self.config["mode"])
        self.camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        self.camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        self.camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        logging.info("Camera set.")

    def init_multiprocessing(self):
        self.camera_found = mp.Event()
        self.frame_taken = mp.Event()
        self.buffer_copied = mp.Event()
        self.capture_process = mp.Process(target=self.capture)
        self.detection_process = mp.Process(target=self.detect_motion)
        self.trigger_latched = mp.Value("b", False)
        logging.info("Multiprocessing initialized.")

    def capture_frame(self):
        self.load_config()

        camera = arducam.mipi_camera()
        try:
            camera.init_camera()
        except RuntimeError:
            logging.error("Camera not found!")
            return

        logging.info("Camera open.")
        camera.set_resolution(*self.res)
        camera.set_mode(self.config["mode"])
        camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        logging.info("Camera set.")
        
        # capture single image and save to current save directory
        frame = camera.capture(encoding="raw", quality = 90)
        buffer = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res[::-1])
        im = Image.fromarray(buffer).convert("L")
        d = datetime.now().strftime(self.config["date_format"])
        path = os.path.join(self.config["data_path"], self.config["cam_name"]+"-"+d+"."+self.config["image_format"])        
        im.save(path)
        logging.info("Image captured and saved.")
        camera.close_camera()
        logging.info("Camera closed.")

    # image capturing process
    def capture(self):
        # create camera instance, and initialize
        camera = arducam.mipi_camera()
        try:
            camera.init_camera()
        except RuntimeError:
            return

        self.camera_found.set()
        logging.info("Camera open.")
        camera.set_resolution(*self.res)
        # use mode 5 or 11 for 1280x800 2lane raw8 capture
        camera.set_mode(self.config["mode"])
        camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        camera.set_control(v4l2.V4L2_CID_EXPOSURE,self.config["exposure"])
        logging.info("Camera ready.\n")

        t_overall = time.time()
        GPIO.output(self.config["state_pin"], GPIO.HIGH)

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
            self.ind.value = (i+1) % self.config["buffer_len"]
            self.frame_taken.set()

            # print FPS every time buffer is filled
            if i==0:
                fps = self.config["buffer_len"]/(time.time()-t_overall)
                motion = np.max(self.pixdiff)
                if fps<500: # omit the first time
                    logging.info("FPS: %3.2f, Dropped: %4d, Motion: %4d",fps,np.sum(self.skipped),motion)
                t_overall = time.time()

        # take remaining frames
        for j in range(self.config["post_trig"]):
            i = self.ind.value
            frame = camera.capture(encoding="raw", quality=90)
            self.timestamp[i] = time.time()
            self.pts[i] = frame.buffer_ptr[0].pts
            self.timediff[i] = self.pts[i] - self.pts[i-1]
            self.skipped[i] = int((self.timediff[i]-2000)/10000)
            self.buffer[i] = np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=self.res)
            self.ind.value = (i+1) % self.config["buffer_len"]
        
        logging.info("Remaining frames taken.")
        # roll buffer position so the last taken image is positioned last
        i = self.ind.value
        self.buffer[:] = np.roll(self.buffer, -i, axis=0)

        camera.close_camera()
        logging.info("Capture thread quitted. Camera closed.")

    def detect_motion(self):
        # create two local buffer of two frames
        frame1 = np.zeros(self.res, dtype=np.uint8)
        frame2 = np.zeros(self.res, dtype=np.uint8)
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

            if counter>self.config["pix_threshold"] and GPIO.input(self.config["trig_en_pin"]):
                # send out a pulse of 100 us
                GPIO.output(self.config["trig_pin"],GPIO.HIGH)
                time.sleep(0.0001)
                GPIO.output(self.config["trig_pin"],GPIO.LOW)
                logging.info("Detected motion: %d.\t Trigger sent."%counter)
        
        logging.info("Motion detection thread quitted.")

    def save_frame(self, i):
        # saves each from to file
        im = Image.fromarray(self.buffer[i]).convert("L")
        filename = self.config["data_path"]+self.config["cam_name"]+"-img{:02}".format(i)+"."+self.config["image_format"]
        im.save(filename)

    def save_images(self):
        logging.info("Saving images . . .")
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
        self.caminfo.to_csv(self.config["data_path"]+self.config["cam_name"]+"-info.csv", float_format="%.9f")
        
        # reshapes image buffer and saves to disk
        self.buffer = np.reshape(self.buffer,tuple(np.array([self.config["buffer_len"], self.res[1], self.res[0]])))
        # pool = ThreadPool(4)
        # pool.map(self.save_frame, range(self.config["buffer_len"]))
        for i in range(self.config["buffer_len"]):
            im = Image.fromarray(self.buffer[i]).convert("L")
            filename = self.config["data_path"]+self.config["cam_name"]+"-img{:02}".format(i)+"."+self.config["image_format"]
            im.save(filename)

        logging.info("Images saved. Time: %.0fs."%(time.time()-t_overall))

    def start_event(self):
        self.init_gpio()
        logging.info("Waiting for event to start . . .")
        while not GPIO.input(self.config["state_comm_pin"]):
            time.sleep(0.001)
            # use trig_en when not in event to capture a single frame
            if GPIO.input(self.config["trig_en_pin"]):
                self.capture_frame_process = mp.Process(target=self.capture_frame)
                self.capture_frame_process.start()
                self.capture_frame_process.join()
                return

        self.load_config()
        self.init_buffer()
        self.init_multiprocessing()

        t_overall = time.time()

        try:
            # start processes
            self.capture_process.start()
            # wait maximum 5 seconds for camera to open
            if not self.camera_found.wait(5):
                logging.error("Camera not found. Quitting.")
                return
            self.detection_process.start()
            logging.info("Processes started.")

            time.sleep(1)
            while not GPIO.input(self.config["trig_latch_pin"]):
            # and time.time()-t_overall<t:
                time.sleep(0.001)

            self.trigger_latched.value = True
            logging.info("Trigger latched.")

        except KeyboardInterrupt:
                logging.error('Interrupted.\n')

        self.capture_process.join()
        self.detection_process.join()

        self.save_images()
        GPIO.output(self.config["state_pin"], GPIO.LOW)
        #GPIO.cleanup()
        
if __name__ == "__main__":
    c = CaptureCore()
    c.start_event()
    logging.info("Program finished.")
