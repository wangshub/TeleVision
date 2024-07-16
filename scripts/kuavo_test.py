import pybullet as p
import pybullet_data
import time
import argparse


def get_parser():
    parser = argparse.ArgumentParser(description="Show 3D Robot Model in PyBullet")
    parser.add_argument(
        "model", type=str, nargs="?", help="path to model file"
    )
    return vars(parser.parse_args())

def camera_look_at(position, distance=2, yaw=0):
    assert len(position) == 3, 'need position = [x, y, z]'
    p.resetDebugVisualizerCamera(distance, yaw, 0, position)


def main():
    args = get_parser()

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([-45, 0, 0])
    plane_id = p.loadURDF("plane.urdf")

    camera_look_at(cubeStartPos)


    model_file = args.get('model')
    if 'urdf' in model_file.lower():
        robot_id = p.loadURDF(model_file, 
                              cubeStartPos, 
                              cubeStartOrientation,
                              useFixedBase=False)
    elif 'sdf' in model_file.lower():
        robot_id = p.loadSDF(model_file)
    else:
        assert Exception('cannot recog {}'.format(model_file))

    grav_id = p.addUserDebugParameter("gravity", -10, 10, -10)
    joint_ids = []
    param_ids = []

    for i in range(100):
        time.sleep(0.01)
        p.stepSimulation()

    # while True:
    #     p.stepSimulation()

    pos, _ = p.getBasePositionAndOrientation(robot_id)

    print(f'com_pos = {pos}')

    for joint in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint)
        link_state = p.getLinkState(robot_id, joint)
        mass = p.getDynamicsInfo(robot_id, joint)[0]
        print(f"{joint_info[1].decode('utf-8')} = {link_state[0]}, mass = {mass}")

        joint_name = joint_info[1]
        joint_type = joint_info[2]
        jointLowerLimit = joint_info[8]
        jointUpperLimit = joint_info[9]
        if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
            joint_ids.append(joint)
            param_ids.append(p.addUserDebugParameter(f'{joint_info[0]}|{joint_name.decode("utf-8")}', jointLowerLimit, jointUpperLimit, 0))

    p.setRealTimeSimulation(1)
    while True:
        time.sleep(0.01)
        p.setGravity(0, 0, p.readUserDebugParameter(grav_id))
        for i in range(len(param_ids)):
            config = param_ids[i]
            target_pos = p.readUserDebugParameter(config)
            p.setJointMotorControl2(robot_id, joint_ids[i], p.POSITION_CONTROL, target_pos, force=5 * 240.)


if __name__ == '__main__':
    main()
