#creates .jpg file in Captures folder in SBCcode/Cameras/RPi_CameraServers/python/Captures
#file is labelled with data and time when taken

import arducam_mipicamera as arducam
import v4l2
from datetime import datetime
import os

def set_controls(camera):
    try:
        print("Enable Auto Exposure...")
        camera.software_auto_exposure(enable = True)
    except Exception as e:
        print(e)

def capture(camera):
    frame = camera.capture(encoding = 'jpeg')
    d1 = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    path = os.path.join(os.getcwd(), "Captures", d1+".jpg")
    path = "/home/pi/RPi_CameraServers/Captures/"+d1+".bmp"
    open(path, "wb")
    frame.as_array.tofile(path)
            #Remove frame from memory
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
            # camera.set_control(v4l2.V4L2_CID_EXPOSURE,300)
            # camera.set_control(v4l2.V4L2_CID_GAIN,4)
            print("Current resolution is {}".format(fmt))
            set_controls(camera)
            capture(camera)
            print("Close camera...")
            camera.close_camera()
        except Exception as e:
            print(e)
