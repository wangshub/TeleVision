import mujoco
import mujoco.viewer
import numpy as np

#add path
xml_file = "/home/nuc/Github/mujoco_menagerie/universal_robots_ur5e/scene.xml"

# 加载模型
model = mujoco.MjModel.from_xml_path(xml_file)  # 替换为您的模型文件路径
data = mujoco.MjData(model)


#Put a position of the joints to get a test point
pi = np.pi
data.qpos = [3*pi/2, -pi/2, pi/2, 3*pi/2, 3*pi/2, 0]

#Initial joint position
qpos0 = data.qpos.copy()

#Step the simulation.
mujoco.mj_forward(model, data)

#Use the last piece as an "end effector" to get a test point in cartesian 
# coordinates
target = data.body('wrist_3_link').xpos
print("Target =>",target)

#Plot results
print("Results")
mujoco.mj_resetDataKeyframe(model, data, 1)
mujoco.mj_forward(model, data)
init_point = data.body('wrist_3_link').xpos.copy()

data.qpos = qpos0
mujoco.mj_forward(model, data)
result_point = data.body('wrist_3_link').xpos.copy()

print("initial point =>", init_point)
print("Desire point =>", result_point, "\n")

# 使用 MuJoCo 视图显示模型
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        
        viewer.sync()