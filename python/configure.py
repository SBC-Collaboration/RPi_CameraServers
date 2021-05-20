import json

config_path = "../config.json"

config = {}
config["exposure"] = 250
config["resolution"] = [1280,800]
config["frame_sync"] = True
config["mode"] = 11 #5
config["buffer_len"] = 100
config["frames_after"] = 50
config["adc_threshold"] = 10
config["pix_threshold"] = 199 #15
config["save_path"] = "./Captures/"
config["image_format"] = ".png"
config["date_format"] = "%Y-%m-%d_%H:%M:%S"
config["input_pins"] = {"state_com": 5,
                        "trig_en": 6,
                        "trig_latch": 13}
config["output_pins"] = {"state": 23,
                         "trig": 24}

# mode: 11, width: 1280, height: 800, pixelformat: GREY, desc: Used for ov9281 2lanes raw8 1280x800 external trigger mode
# mode: 5, width: 1280, height: 800, pixelformat: GREY, desc: Used for ov9281 2lans raw8

def save_config(config_path):
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

save_config(config_path)