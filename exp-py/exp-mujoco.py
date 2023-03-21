#@title Import packages for plotting and creating graphics
import mujoco
import time
import itertools
import numpy as np
from typing import Callable, NamedTuple, Optional, Union, List

# Graphics and plotting.
#print('Installing mediapy:')
#!command -v ffmpeg >/dev/null || (apt update && apt install -y ffmpeg)
#!pip install -q mediapy
import mediapy as media
import matplotlib.pyplot as plt

# More legible printing from numpy.
np.set_printoptions(precision=3, suppress=True, linewidth=100)


xml = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

#duration = 3.8  # (seconds)
#framerate = 60  # (Hz)

# Simulate and display video.
#frames = []
#mujoco.mj_resetData(model, data)  # Reset state and time.
#while data.time < duration:
#  mujoco.mj_step(model, data)
#  if len(frames) < data.time * framerate:
#    renderer.update_scene(data)
#    pixels = renderer.render().copy()
#    frames.append(pixels)
#media.show_video(frames, fps=framerate)