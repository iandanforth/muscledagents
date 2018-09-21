import sys
import os
from mujoco_py import load_model_from_path, MjSim, MjViewer

model_base_path = os.path.join(
    "..",
    "muscledagents",
    "envs",
    "mujoco",
    "assets",
)
model_name = sys.argv[1]
model_path = os.path.join(model_base_path, model_name)
model = load_model_from_path(model_path)
sim = MjSim(model)
viewer = MjViewer(sim)

for i in range(15000):
    sim.step()
    viewer.render()
