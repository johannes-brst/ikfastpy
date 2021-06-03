import numpy as np
import ikfastpy
import rodrigues
import rtde_control
import rtde_receive
import time
import math

def newJointSpeed(joint_config, actual_q, joint_speed, max_vel):
    print("newJointSpeed")
    print("joint_config: "+str(joint_config))
    print("actual_q: "+str(actual_q))
    tmp_speed = np.subtract(joint_config, actual_q)
    for i in range(len(joint_config)):
        if(abs(tmp_speed[i]) > 0.02):
            joint_speed[i] = tmp_speed[i]*(max_vel/max(np.absolute(tmp_speed)))
        else:
            joint_speed[i] = 0
    print("joint_speed: "+str(joint_speed))
    return joint_speed

def findClosestSolution(joint_configs, actual_q):
    print("findClosestSolution: ")
    diffs = [0] * len(joint_configs)
    for i in range(len(joint_configs)):   
        diffs[i] = sum(abs(x - y) for x, y in zip(sorted(joint_configs[i]), sorted(actual_q)))
    print("index clostest sol: " + str(diffs.index(min(diffs))))
    return diffs.index(min(diffs))


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
rtde_c.moveJ([1.755020, -1.131027, 2.141150,  0.456918,  0.842970, -0.346717])
actual_q = rtde_r.getActualQ()
print("actual_q: " + str(actual_q))
joint_speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dt = 0.02 #1.0/500 
max_vel = 3.14
acceleration = 40
joint_config = joint_configs[findClosestSolution(joint_configs,actual_q)]
#for joint_config in joint_configs:
    #print("Moving to pose:")
    #print(joint_config)
    #rtde_c.moveJ(joint_config)
    #time.sleep(1)
    
    # TODO: endloses Pendeln über Ziel Winkel. Schaukelt sich immer stärker auf
    # hohe wahrscheinlichkeit das endeffektor an andere Stelle am Roboter hängen bleibt
    # wegen unkoordinierten verhalten ohne collisions berücksichtigung 
actual_q = rtde_r.getActualQ()
joint_speed = newJointSpeed(joint_config, actual_q, joint_speed, max_vel)
is_all_zero = np.all((joint_speed == 0))
while(is_all_zero == False):
    start = time.time()
    rtde_c.speedJ(joint_speed, acceleration,dt)
    end = time.time()
    duration = end - start
    if duration < dt:
        time.sleep(dt - duration)
    actual_q = rtde_r.getActualQ()
    joint_speed = newJointSpeed(joint_config, actual_q, joint_speed, max_vel)
    is_all_zero = np.all((joint_speed == 0))


# Stop the RTDE control script
rtde_c.stopScript()

