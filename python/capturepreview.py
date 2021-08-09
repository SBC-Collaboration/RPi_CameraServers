import v4l2
import arducam_mipicamera as arducam
from datetime import datetime
import time
import os
import getkey

def capture(camera):
    frame = camera.capture(encoding = 'jpeg')
    d1 = datetime.utcnow().isoformat(sep=' ', timespec='milliseconds')
    path = "../Captures/"+d1+".jpg"
    open(path, "wb")
    frame.as_array.tofile(path) 
    del frame  # removes frame from memory

def capture_multiple(camera, n, delay):
    d1 = datetime.now().isoformat(sep=' ', timespec='seconds')
    os.mkdir("../Captures/{}/".format(d1))
    for i in range(n):
        frame = camera.capture(encoding = 'jpeg')
        path = "../Captures/{}/img{:02}.jpg".format(d1,i)
        open(path, "wb")
        frame.as_array.tofile(path) 
        del frame  # removes frame from memory
        time.sleep(delay)

message = "Press 'c' to capture, 'v' to capture 100 frames, UP ARROW/DOWN ARROW then ENTER to increase/decrease exposure, or input 'e' to stop"

try:
    camera = arducam.mipi_camera()
    print("Open camera...")
    camera.init_camera()
    print("Setting the resolution...")
    fmt = camera.set_resolution(1920, 1080)
    print("Current resolution is {}".format(fmt))
    camera.software_auto_exposure(enable=False)
    exposure = 500
    camera.set_control(v4l2.V4L2_CID_EXPOSURE, exposure)  
    print('Default exposure: 500')
    camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
    camera.set_control(v4l2.V4L2_CID_HFLIP, 1)
    print("Start preview...")
    camera.start_preview(fullscreen=False, window=(0, 0, 1280, 720))

    while True:
        print(message)
        k = getkey.getkey()
        print(k)
        if k == 'c':
            capture(camera)
            print('Image captured!')
        elif k == 'v':
            capture_multiple(camera, 100, 0.1)
        elif k == 'e':
            print("Stop preview...")
            camera.stop_preview()
            print("Close camera...")
            camera.close_camera()
            break
        elif k == getkey.keys.UP:
            exposure += 100
            camera.set_control(v4l2.V4L2_CID_EXPOSURE, exposure) 
            print('Exposure set to: ', exposure)  
        elif k == getkey.keys.DOWN:
            if exposure > 100:
                exposure -= 100
                camera.set_control(v4l2.V4L2_CID_EXPOSURE, exposure) 
                print('Exposure set to: ', exposure)
            else:
                print('Minimum Exposure reached!')     
        else:
            print('Error: invalid input')
except Exception as e:
    print(e)
