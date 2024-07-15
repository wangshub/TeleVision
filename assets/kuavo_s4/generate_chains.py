from ikpy import chain


###### Left Arm ######
robotis_op3_left_arm_links = ["torso",
                              "l_arm_pitch",
                              "l_arm_roll",
                              "l_arm_yaw",
                              "l_forearm_pitch",
                              "l_hand_yaw",
                              "l_hand_pitch",
                              "l_hand_roll"
                              ]

robotis_op3_left_arm_joints = ["l_arm_pitch",
                               "l_arm_roll",
                               "l_arm_yaw",
                               "l_forearm_pitch",
                               "l_hand_yaw",
                               "l_hand_pitch",
                               "l_hand_roll"
                               ]       

robotis_op3_left_arm_elements = [x for pair in zip(robotis_op3_left_arm_links, robotis_op3_left_arm_joints) for x in pair] + ["l_hand_roll"]

# Note: 'Links' in IKPY correspond to 'Joints' in URDF terminology. 'Links' in URDF are stripped by IKPY. 
robotis_op3_left_arm_chain = chain.Chain.from_urdf_file(
    "urdf/biped_s4.urdf",
    base_elements=robotis_op3_left_arm_elements,
    last_link_vector=[0, 0.0, 0],
    active_links_mask=[False] + 7 * [True] + [False] + [False],
    symbolic=False,
    name="kuavo_s4_left_arm")

robotis_op3_left_arm_chain.to_json_file(force=True)


# exit(0)

###### Right Arm ######

robotis_op3_right_arm_links = ["torso",
                              "r_arm_pitch",
                              "r_arm_roll",
                              "r_arm_yaw",
                              "r_forearm_pitch",
                              "r_hand_yaw",
                              "r_hand_pitch",
                              "r_hand_roll"
                              ]

robotis_op3_right_arm_joints = ["r_arm_pitch",
                               "r_arm_roll",
                               "r_arm_yaw",
                               "r_forearm_pitch",
                               "r_hand_yaw",
                               "r_hand_pitch",
                               "r_hand_roll"
                               ]  

robotis_op3_right_arm_elements = [x for pair in zip(robotis_op3_right_arm_links, robotis_op3_right_arm_joints) for x in pair] + ["r_hand_roll"]

robotis_op3_right_arm_chain = chain.Chain.from_urdf_file(
    "urdf/biped_s4.urdf",
    base_elements=robotis_op3_right_arm_elements,
    last_link_vector=[0, -0.0, 0],
    active_links_mask=[False] + 7 * [True] + [False] + [False],
    symbolic=False,
    name="kuavo_s4_right_arm"
)

robotis_op3_right_arm_chain.to_json_file(force=True)


exit(0)

###### Left Leg ######

robotis_op3_left_leg_links = ["body_link",
                              "l_hip_yaw_link",
                              "l_hip_roll_link",
                              "l_hip_pitch_link",
                              "l_knee_link",
                              "l_ank_pitch_link",
                              "l_ank_roll_link"]

robotis_op3_left_leg_joints = ["l_hip_yaw",
                               "l_hip_roll",
                               "l_hip_pitch",
                               "l_knee",
                               "l_ank_pitch",
                               "l_ank_roll"]

robotis_op3_left_leg_elements = [x for pair in zip(robotis_op3_left_leg_links, robotis_op3_left_leg_joints) for x in pair] + ["l_ank_roll_link"]

robotis_op3_left_leg_chain = chain.Chain.from_urdf_file(
    "urdf/robotis_op3.urdf",
    base_elements=robotis_op3_left_leg_elements,
    last_link_vector=[0, 0, -0],
    active_links_mask=[False] + 6 * [True] + [False],
    symbolic=False,
    name="robotis_op3_left_leg"
)

robotis_op3_left_leg_chain.to_json_file(force=True)


###### Right Leg ######

robotis_op3_right_leg_links = ["body_link",
                              "r_hip_yaw_link",
                              "r_hip_roll_link",
                              "r_hip_pitch_link",
                              "r_knee_link",
                              "r_ank_pitch_link",
                              "r_ank_roll_link"]

robotis_op3_right_leg_joints = ["r_hip_yaw",
                               "r_hip_roll",
                               "r_hip_pitch",
                               "r_knee",
                               "r_ank_pitch",
                               "r_ank_roll"]

robotis_op3_right_leg_elements = [x for pair in zip(robotis_op3_right_leg_links, robotis_op3_right_leg_joints) for x in pair] + ["r_ank_roll_link"]

robotis_op3_right_leg_chain = chain.Chain.from_urdf_file(
    "urdf/robotis_op3.urdf",
    base_elements=robotis_op3_right_leg_elements,
    last_link_vector=[0, 0, 0],
    active_links_mask=[False] + 6 * [True] + [False],
    symbolic=False,
    name="robotis_op3_right_leg"
)

robotis_op3_right_leg_chain.to_json_file(force=True)
