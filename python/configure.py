import json

config_path = "config.json"

config = {}
config["adc_threshold"] = 10
config["exposure"] = 1
config["pix_threshold"] = 199 #15
config["max_frames"] = 100
config["frames_after"] = 50
config["frame_rate"] = 100
config["resolution"] = [1280,800]
config["save_path"] = "./Captures/"
config["image_format"] = ".jpg"
config["date_format"] = "%Y-%m-%d_%H:%M:%S"

def save_config(config_path):
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

save_config(config_path)