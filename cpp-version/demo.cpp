#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <cmath>        // std::abs
#include <thread>
#include <chrono>
#include <vector>
#include "ikfast_wrapper.h"
using namespace ur_rtde;
using namespace std::chrono;
using namespace robots; 

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char *argv[])
{
    Kinematics ur5e_kin;
    int n_joints = ur5e_kin.num_of_joints;
    printf("n_joints = %i\n",n_joints);
    // Test forward kinematics: get end effector pose from joint angles
    std::vector<double> joint_angles = {0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717};

    printf("\nTesting forward kinematics:\n");
    printf("Joint angles:");
    for (int i = n_joints - 1; i >= 0; i--) 
        std::cout << joint_angles[i];
    printf("\n");
    float sT = 0.002; //sample Time
    RTDEControlInterface rtde_control("127.0.0.1");//172.30.10.1
    RTDEReceiveInterface rtde_receive("127.0.0.1");
    std::vector<double> init_q = rtde_receive.getActualQ();
    std::vector<double> startpos = {0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717};
    rtde_control.moveJ(startpos);
    int steps = 2000;
    int max_vel = 3	;

	rtde_control.speedStop();
    rtde_control.stopScript();
	rtde_receive.disconnect();
    return 0;
}
