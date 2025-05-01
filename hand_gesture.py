from depthai_sdk import OakCamera
import depthai

# Download & deploy a model from Roboflow Universe
# https://universe.roboflow.com

with OakCamera(replay="INSERT_PATH_TO_VIDEO_FILE.mp4") as oak:
    color = oak.create_camera('color')
    model_config = {
        'source': 'roboflow', # Specify that we are downloading the model from Roboflow
        'model':'rock-paper-scissors-sxsw-dpius/2',
        'key':'1ExOYUcWYjQ7hxafJHcj' # FAKE Private API key, replace with your own!
    }
    nn = oak.create_nn(model_config, color)
    oak.visualize(nn, fps=True)
    oak.start(blocking=True)