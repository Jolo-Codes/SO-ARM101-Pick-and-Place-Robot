import mujoco
import mujoco.viewer
import numpy as np
import time

#Loads our data from our model
model = mujoco.MjModel.from_xml_path("SO101-Model/scene.xml")
data = mujoco.MjData(model)

#Opens the model so we can play around with it.
mujoco.viewer.launch(model, data)