import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
planeId = p.loadURDF("plane.urdf")
legsStartPos = [0, 0, 2]
p.setGravity(0, 0, -10)
legsStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
legsID = p.loadMJCF("legs.xml")[0]
p.resetBasePositionAndOrientation(legsID, legsStartPos, legsStartOrientation)

# start camera view perpendicular to figure
p.resetDebugVisualizerCamera(cameraDistance=4, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 1])


# joint and link numbers
base_link = 3

right_hip_joint = 5
right_knee_joint = 8
right_ankle_joint = 11
right_ankle_link = 10
right_foot_link = 12

left_hip_joint = 14
left_knee_joint = 17
left_ankle_joint = 20
left_ankle_link = 19
left_foot_link = 21

# target angles
base_angle = 0

swing_hip_0_2 = 0.7
swing_knee_0_2 = -1.3
swing_hip_1_3 = -0.7
swing_knee_1_3 = -0.05
swing_ankle_0_2 = 0.2
swing_ankle_1_3 = 0.2

stance_knee_0_2 = -0.05
stance_ankle_0_2 = 0.20
stance_knee_1_3 = -0.10
stance_ankle_1_3 = 0.2

# PD controller constants
kp = 800
kd = 80
cd_0_2 = 0
cv_0_2 = 0.2
cd_1_3 = 2.2
cv_1_3 = 0

# simulation parameters
time_step = 0.01
p.setTimeStep(time_step)
foot_contact_tol = 0.0001


def stance_leg_torque(swing_hip_torque):
    body = p.getJointState(legsID, base_link)
    tau_body = kp*(-base_angle - body[0]) - kd * body[1]
    tau_stance = -tau_body - swing_hip_torque
    return tau_stance


def balance_feedback(target_angle, stance_ankle_link, cd, cv):
    body = p.getLinkState(legsID, 3, computeLinkVelocity=1)
    body_position = body[0]
    body_velocity = body[6][0]
    ankle_position = p.getLinkState(legsID, stance_ankle_link)[0]

    d = body_position[0] - ankle_position[0]

    return target_angle + cd*d + cv*body_velocity


def in_contact(contact_points, contact_links):
    links_in_contact = []
    for contact_point in contact_points:
        links_in_contact.append(contact_point[4])

    for link in contact_links:
        if link in links_in_contact:
            return 1

    return 0


def set_torque_0():
    # swing leg
    right_hip = p.getJointState(legsID, right_hip_joint)

    # with feedback
    swing_hip_angle = balance_feedback(-swing_hip_0_2, left_ankle_link, cd_0_2, cv_0_2)
    tau_r_hip = kp * (swing_hip_angle - right_hip[0]) - kd * right_hip[1]

    # without
    # tau_r_hip = kp * (swing_hip_0_2 - right_hip[0]) - kd * right_hip[1]

    right_knee = p.getJointState(legsID, right_knee_joint)
    tau_r_knee = kp * (-swing_knee_0_2 - right_knee[0]) - kd * right_knee[1]

    right_ankle = p.getJointState(legsID, right_ankle_joint)
    tau_r_ankle = kp * (-swing_ankle_0_2 - right_ankle[0]) - kd * right_ankle[1]

    # stance leg
    tau_l_hip = stance_leg_torque(tau_r_hip)

    left_knee = p.getJointState(legsID, left_knee_joint)
    tau_l_knee = kp * (-stance_knee_0_2 - left_knee[0]) - kd * left_knee[1]

    left_ankle = p.getJointState(legsID, left_ankle_joint)
    tau_l_ankle = kp * (-stance_ankle_0_2 - left_ankle[0]) - kd * left_ankle[1]

    # update forces
    p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, right_ankle_joint, left_hip_joint,
        left_knee_joint, left_ankle_joint], controlMode=p.TORQUE_CONTROL, forces=[tau_r_hip, tau_r_knee,
        tau_r_ankle, tau_l_hip, tau_l_knee, tau_l_ankle])
    p.stepSimulation()


def set_torque_1():
    # swing leg
    right_hip = p.getJointState(legsID, right_hip_joint)
    # with feedback
    swing_hip_angle = balance_feedback(-swing_hip_1_3, left_ankle_link, cd_1_3, cv_1_3)
    tau_r_hip = kp * (swing_hip_angle - right_hip[0]) - kd * right_hip[1]

    # without
    # tau_r_hip = kp * (swing_hip_1_3 - right_hip[0]) - kd * right_hip[1]

    right_knee = p.getJointState(legsID, right_knee_joint)
    tau_r_knee = kp * (-swing_knee_1_3 - right_knee[0]) - kd * right_knee[1]

    right_ankle = p.getJointState(legsID, right_ankle_joint)
    tau_r_ankle = kp * (-swing_ankle_1_3 - right_ankle[0]) - kd * right_ankle[1]

    # stance leg
    tau_l_hip = stance_leg_torque(tau_r_hip)

    left_knee = p.getJointState(legsID, left_knee_joint)
    tau_l_knee = kp * (-stance_knee_1_3 - left_knee[0]) - kd * left_knee[1]

    left_ankle = p.getJointState(legsID, left_ankle_joint)
    tau_l_ankle = kp * (-stance_ankle_1_3 - left_ankle[0]) - kd * left_ankle[1]

    # update forces
    p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, right_ankle_joint, left_hip_joint,
        left_knee_joint, left_ankle_joint], controlMode=p.TORQUE_CONTROL, forces=[tau_r_hip, tau_r_knee,
        tau_r_ankle, tau_l_hip, tau_l_knee, tau_l_ankle])
    p.stepSimulation()


def set_torque_2():
    # swing leg
    left_hip = p.getJointState(legsID, left_hip_joint)
    # with feedback
    swing_hip_angle = balance_feedback(-swing_hip_0_2, right_ankle_link, cd_0_2, cv_0_2)
    tau_l_hip = kp * (swing_hip_angle - left_hip[0]) - kd * left_hip[1]

    # without
    # tau_l_hip = kp * (swing_hip_0_2 - left_hip[0]) - kd * left_hip[1]

    left_knee = p.getJointState(legsID, left_knee_joint)
    tau_l_knee = kp * (-swing_knee_0_2 - left_knee[0]) - kd * left_knee[1]

    left_ankle = p.getJointState(legsID, left_ankle_joint)
    tau_l_ankle = kp * (-swing_ankle_0_2 - left_ankle[0]) - kd * left_ankle[1]

    # stance leg
    tau_r_hip = stance_leg_torque(tau_l_hip)

    right_knee = p.getJointState(legsID, right_knee_joint)
    tau_r_knee = kp * (-stance_knee_0_2 - right_knee[0]) - kd * right_knee[1]

    right_ankle = p.getJointState(legsID, right_ankle_joint)
    tau_r_ankle = kp * (-stance_ankle_0_2 - right_ankle[0]) - kd * right_ankle[1]

    # update forces
    p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, right_ankle_joint, left_hip_joint,
        left_knee_joint, left_ankle_joint], controlMode=p.TORQUE_CONTROL, forces=[tau_r_hip, tau_r_knee,
        tau_r_ankle, tau_l_hip, tau_l_knee, tau_l_ankle])
    p.stepSimulation()


def set_torque_3():
    # swing leg
    left_hip = p.getJointState(legsID, left_hip_joint)
    # with feedback
    swing_hip_angle = balance_feedback(-swing_hip_1_3, right_ankle_link, cd_1_3, cv_1_3)
    tau_l_hip = kp * (swing_hip_angle - left_hip[0]) - kd * left_hip[1]

    # without
    # tau_l_hip = kp * (swing_hip_1_3 - left_hip[0]) - kd * left_hip[1]

    left_knee = p.getJointState(legsID, left_knee_joint)
    tau_l_knee = kp * (-swing_knee_1_3 - left_knee[0]) - kd * left_knee[1]

    left_ankle = p.getJointState(legsID, left_ankle_joint)
    tau_l_ankle = kp * (-swing_ankle_1_3 - left_ankle[0]) - kd * left_ankle[1]

    # stance leg
    tau_r_hip = stance_leg_torque(tau_l_hip)

    right_knee = p.getJointState(legsID, right_knee_joint)
    tau_r_knee = kp * (-stance_knee_1_3 - right_knee[0]) - kd * right_knee[1]

    right_ankle = p.getJointState(legsID, right_ankle_joint)
    tau_r_ankle = kp * (-stance_ankle_1_3 - right_ankle[0]) - kd * right_ankle[1]

    # update forces
    p.setJointMotorControlArray(legsID, [right_hip_joint, right_knee_joint, right_ankle_joint, left_hip_joint,
        left_knee_joint, left_ankle_joint], controlMode=p.TORQUE_CONTROL, forces=[tau_r_hip, tau_r_knee,
        tau_r_ankle, tau_l_hip, tau_l_knee, tau_l_ankle])
    p.stepSimulation()


# state definitions
def state0():
    t = time.time()
    while time.time() - t < 0.3:
        set_torque_0()
    state1()


def state1():
    no_contact = 1
    while no_contact:
        #swing leg right leg, stance left
        set_torque_1()
        contact_points = p.getContactPoints(bodyA=planeId, bodyB=legsID)
        #print("contact points: " + str(contact_points))
        if len(contact_points) > 0 and in_contact(contact_points, [right_foot_link, right_ankle_link]):
            no_contact = 0

    state2()


def state2():
    t = time.time()
    while time.time() - t < 0.3:
        set_torque_2()
    state3()


def state3():
    no_contact = 1
    while no_contact:
        # swing leg left leg, stance right
        set_torque_3()
        contact_points = p.getContactPoints(bodyA=planeId, bodyB=legsID)
        #print("contact points: " + str(contact_points))
        if len(contact_points) > 0 and in_contact(contact_points, [left_foot_link, left_ankle_link]):
            contact_distance = contact_points[0][8]
            if contact_distance < foot_contact_tol:
                no_contact = 0
    state0()


# start the FSM with a small time delay to allow character to fall into place
t = time.time()
while time.time() - t < 0.5:
    p.stepSimulation()
state0()
