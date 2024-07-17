import mujoco
import mujoco.viewer
import numpy as np
import time
from dm_control import mjcf
from dm_control.utils.inverse_kinematics import qpos_from_site_pose


#add path
xml_file = "../assets/kuavo_s4/meshes/biped_s4.xml"
mjcf_model = mjcf.from_path(xml_file)
physics = mjcf.Physics.from_mjcf_model(mjcf_model)


start_time = time.time()
ik_result = qpos_from_site_pose(
                        physics,
                        "r_hand_site",
                        target_pos=[0.0, 0.0, 0.0],
                        target_quat=[0, 0, 1, 0],
                        tol=1e-10,
                        max_steps=400,
                    )
end_time = time.time()

print(ik_result)
print(ik_result.qpos.shape)
print(f"Time taken: {end_time - start_time}")



# # 加载模型
model = mujoco.MjModel.from_xml_path(xml_file)  # 替换为您的模型文件路径
data = mujoco.MjData(model)

# mujoco.viewer.
print(model)
geom_id = model.geom("marker_sphere")
print('geom_id = ', geom_id)
data.geom_xpos[geom_id.id] = [0.0, 0.0, 0.0]


n_joints = model.njnt

for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    # print(f'joint index: {i}, joint name: {joint_name}')


# ret = qpos_from_site_pose(model, 'l_hand_roll', [0.5, 0.5, 0.5], [0, 0, 0, 1])
# print(ret)


# #Put a position of the joints to get a test point
# pi = np.pi
# data.qpos = [1, 1, 1, 1, 1, 1, 1]

# #Initial joint position
# qpos0 = data.qpos.copy()

# #Step the simulation.
# mujoco.mj_forward(model, data)

# #Use the last piece as an "end effector" to get a test point in cartesian 
# # coordinates
# target = data.body('wrist_3_link').xpos
# print("Target =>",target)

# #Plot results
# print("Results")
# mujoco.mj_resetDataKeyframe(model, data, 1)
# mujoco.mj_forward(model, data)
# init_point = data.body('wrist_3_link').xpos.copy()

# data.qpos = qpos0
# mujoco.mj_forward(model, data)
# result_point = data.body('wrist_3_link').xpos.copy()

# print("initial point =>", init_point)
# print("Desire point =>", result_point, "\n")

# 使用 MuJoCo 视图显示模型
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.qpos = ik_result.qpos
        mujoco.mj_step(model, data)
        viewer.sync()

# while True:
#     mujoco.mj_step(model, data)
#     viewer.render()
#     if viewer.is_alive() is False:
#         break