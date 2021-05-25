import numpy as np
import ikfastpy
import rodrigues
import rtde_control
import rtde_receive
import time

# Initialize kinematics for UR5 robot arm
ur5_kin = ikfastpy.PyKinematics()
n_joints = ur5_kin.getDOF()

joint_angles = [0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717] # in radians

# Test forward kinematics: get end effector pose from joint angles
print("\nTesting forward kinematics:\n")
print("Joint angles:")
print(joint_angles)
ee_pose = ur5_kin.forward(joint_angles)
ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
print("\nEnd effector pose:")
print(ee_pose)
print("\n-----------------------------")

# Test inverse kinematics: get joint angles from end effector pose
print("\nTesting inverse kinematics:\n")
joint_configs = ur5_kin.inverse(ee_pose.reshape(-1).tolist())
n_solutions = int(len(joint_configs)/n_joints)
print("%d solutions found:"%(n_solutions))
joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)
for joint_config in joint_configs:
    print(joint_config)

# Check cycle-consistency of forward and inverse kinematics
assert(np.any([np.sum(np.abs(joint_config-np.asarray(joint_angles))) < 1e-4 for joint_config in joint_configs]))
print("\nTest passed!")


rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
actual_q = rtde_r.getActualQ()
print("actual_q: " + str(actual_q))
for joint_config in joint_configs:
    print("Moving to pose:")
    print(joint_config)
    rtde_c.moveJ(joint_config)
    # anstatt von moveJ speedJ nutzen
    # innnerhalb eines loop die joints solange bewegen lassen bis Zielwinkel erreicht ist
    # ungefähr so wie in rtde_test.cpp
    time.sleep(1)

# Stop the RTDE control script
rtde_c.stopScript()