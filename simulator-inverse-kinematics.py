import mujoco
import mujoco.viewer
import numpy as np
import time

#Loads our data from our model
model = mujoco.MjModel.from_xml_path("SO101-Model/scene.xml")
data = mujoco.MjData(model)


#Lists out al of the actuators
xml_actuators = [model.actuator(i).name for i in range(model.nu)]
print("dog")
print("Actuators found in XML:", xml_actuators)

mujoco.mj_forward(model, data)

for i in range(model.nbody):
    name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_BODY, i
    )
    print(name, data.xpos[i])



#Opens the model so we can play around with it.
mujoco.viewer.launch(model, data)


# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start_time = time.time()
#     while viewer.is_running():
#         data.ctrl[0] = 2
#         data.ctrl[1] = 3
#         mujoco.mj_step(model, data)
#         viewer.sync()

# def solve_3dof_ik(translation_vectors, effector_angle):






