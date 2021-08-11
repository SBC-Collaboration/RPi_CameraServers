# RPi Camera Server

This is the driver for camera setup for SBC Collaboration. It is hosted at [https://github.com/SBC-Collaboration/RPi_CameraServers](https://github.com/SBC-Collaboration/RPi_CameraServers). It uses Arducam OV9281 with Raspberry Pi 4B.

## Instruction
The `imdaq.py` file is the main capture core of the camera. It will first load the configuration settings from will and set the camera with the appropriate settings. Then it will wait for the signal from event builder to start, before it continuously capture frames using frame sync and save to a ring buffer. When it receives a trigger latch, it will take remaining frames and save all frames in buffer to disk, with the information of those frames. Then it will load the configuration file again and wait for the signal to start another event.

## Setup
To successfully establish connection with the camera, and to make sure the image acquisition works properly, please follow the steps below to setup the environment on the Raspberry Pi. The instructions are based on the procedure [here](https://github.com/ArduCAM/MIPI_Camera/tree/master/RPI).
- Install python packages: `v4l2`, `multiprocessing`, and `cython`.
- Enable Camera and I2C in raspberry pi setting: run `sudo raspi-config` in the terminal, and then select `Interface Options`.
- Install SDK library by running the following commands in the terminal in order:
  `cd MIPI_Camera/RPI/`
  `make install`
  `chmod +x enable_i2c_vc.sh `
  `./enable_i2c_vc.sh`

## Configurations
The `config.json` file contains necessary configurations for the camera to operate. It can be edited directly or through `configure.py` script. Below are the options:
- **exposure** (`int`): Length of exposure for each frame. Empirically, each unit is about 7.7us.
- **resolution** (`list`): The resolution of each frame. The default resolution is 1280x800.
- **mode** (`int`): The mode the camera should be operated in. Choose mode 11 when frame sync is on, and mode 5 when frame sync is off.
- **buffer_len** (`int`): The number of frames that the ring buffer can store at once. After this number is reached, the oldest frames will be lost.
- **frames_after** (`int`): The number of frames to be taken after the camera receives a trigger to end the event. 
- **adc_threshold** (`int`): The value that a pixel needs to change from previous frame to be recorded as different.
- **pix_threshold** (`int`): The number of pixels that need to be different from previous frame to trigger.
- **save_path** (`str`): The path where images and logs should be saved to after an event.
- **image_format** (`str`): The format that the image should be saved in.
- **date_format** (`str`): The formating of datetime in file name.
- **input_pins** (`dict`): The GPIO pins on Raspberry Pi that should be used as input. The pins should be referred using BCM numbering, i.e. the number after "GPIO" in the label, not the physical pin number. 
  - **state_com** (`int`): The signal from the Event Builder that tells the RPi which state is should be in. Possible states are "in an event", and "idle".
  - **trig_en** (`int`): The signal from the Event Builder that enables triggering from the frames. The camera will only produce a trigger when this is enabled.
  - **trigger_latch** (`int`): The latched trigger comes from the Trigger FIFO whenever a trigger is broadcasted in the system. The camera will proceed to take remaining images and save to disk once received this.
- **output_pins** (`dict`): The GPIO pins on RPi that should be used as output. Uses BCM numbering.
  - **state** (`int`): The state that the RPi communicates back to the Event Builder.
  - **trig** (`int`): The trigger that RPi generates that it sends to the Trigger FIFO.

## Dependency
This repository requires [MIPI_Camera](https://github.com/ArduCAM/MIPI_Camera) driver provided by Arducam. A version is already included. To save space, when updating the driver, please only copy `RPI` folder into the `MIPI_Camera` folder.