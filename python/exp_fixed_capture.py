#creates .jpg file in Captures folder in SBCcode/Cameras/RPi_CameraServers/python/Captures
#file is labelled with data and time when taken

import arducam_mipicamera as arducam
import v4l2
from datetime import datetime
import os
from PIL import Image
import numpy as np

def set_controls(camera):
    try:
        print("Enable Auto Exposure...")
        camera.software_auto_exposure(enable = True)
    except Exception as e:
        print(e)

def capture(camera, exp):
    camera.set_control(v4l2.V4L2_CID_EXPOSURE,exp)
    frame = camera.capture(encoding = 'raw', quality=90)
    d1 = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    # path = os.path.join(os.getcwd(), "Captures", d1+".jpg")
    path = "/home/pi/RPi_CameraServers/Captures/"+d1+"-exp%d.bmp"%exp
    im = Image.fromarray(np.ctypeslib.as_array(frame.buffer_ptr[0].data,shape=[800,1280])).convert("L")
    # open(path, "wb")
    # frame.as_array.tofile(path)
    im.save(path)
    # Remove frame from memory
    del frame
            
if __name__ == "__main__":
    try:
        camera = arducam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        print("Setting the resolution...")
        fmt = camera.set_resolution(1920, 1080)
        camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        camera.set_control(v4l2.V4L2_CID_HFLIP,1)
        print("Current resolution is {}".format(fmt))
        # set_controls(camera)
        for exp in [100,200,300,500,800,1000]:        
            capture(camera, exp)
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e)
