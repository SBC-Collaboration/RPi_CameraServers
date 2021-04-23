import json

config_path = "config.json"

config = {}
config["adc_threshold"] = 10
config["exposure"] = 500
config["pix_threshold"] = 199 #15
config["max_frames"] = 100
config["frames_after"] = 50
config["frame_rate"] = 100
config["resolution"] = [1280,800]
config["save_path"] = "./Captures/"
config["image_format"] = ".jpg"
config["date_format"] = "%Y-%m-%d_%H:%M:%S"
config["input_pins"] = {"state_com": 7,
                        "trig_en": 9,
                        "trig_latch": 11}
config["output_pins"] = {"state": 2,
                         "trig": 4}
config["frame_sync"] = False
config["registers"] = [[0x4F00, 0x01],
                       [0x3030, 0x04],
                       [0x303F, 0x01],
                       [0x302C, 0x00],
                       [0x302F, 0x7F],
                       [0x3823, 0x30],
                       [0x0100, 0x00],]

def save_config(config_path):
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

save_config(config_path)