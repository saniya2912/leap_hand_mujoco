import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path('/home/sysidea/leap_hand_mujoco/model/leap hand/leaphand.xml')
data = mujoco.MjData(model)

print(f'Number of actuators in the model: {model.nu}')


# create the viewer object
viewer =mujoco.viewer.launch(model, data) #mujoco_viewer.MujocoViewer(model, data)

# # simulate and render
# for _ in range(10000):
#     if viewer.is_alive:
#         mujoco.mj_step(model, data)
#         viewer.render()
#     else:
#         break

# close
viewer.close()

