import argparse
from tqdm import tqdm
from pathlib import Path

import mujoco

from mujoco.usd import exporter

DURATION = 10
FRAMERATE = 24

def generate_cards_usd():

  # load a model to mujoco
  model_path = "/Users/abhishek/Documents/research/mujoco/test/engine/testdata/catenary.xml"
  m = mujoco.MjModel.from_xml_path(model_path)
  d = mujoco.MjData(m)

  # create an instance of the USDExporter
  exp = exporter.USDExporter(model=m,
                             output_directory_name="catenary",
                             output_directory_root="../usd_trajectories/")

  cam = mujoco.MjvCamera()
    
  # step through the simulation for the given duration of time
  while d.time < DURATION:
    mujoco.mj_step(m, d)
    if exp.frame_count < d.time * FRAMERATE:
      exp.update_scene(data=d, camera=cam)

  exp.save_scene(filetype="usd")

if __name__ == "__main__":
    generate_cards_usd()