#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <cmath>        // std::abs
#include <thread>
#include <chrono>
using namespace ur_rtde;
using namespace std::chrono;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char *argv[])
{
    float sT = 0.002; //sample Time
    RTDEControlInterface rtde_control("127.0.0.1");//172.30.10.1
    RTDEReceiveInterface rtde_receive("127.0.0.1");
    std::vector<double> init_q = rtde_receive.getActualQ();
    std::vector<double> startpos = {0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717};
    rtde_control.moveJ(startpos);
    int steps = 2000;
    int max_vel = 3	;
    printf("Joint 0: %f\n",rtde_receive.getActualQ().at(0));
    printf("Joint 1: %f\n",rtde_receive.getActualQ().at(1));
    printf("Joint 2: %f\n",rtde_receive.getActualQ().at(2));
    printf("Joint 3: %f\n",rtde_receive.getActualQ().at(3));
    printf("Joint 4: %f\n",rtde_receive.getActualQ().at(4));
    printf("Joint 5: %f\n",rtde_receive.getActualQ().at(5));

    printf("TCP offset x: %f\n", rtde_receive.getActualTCPPose().at(0));
    printf("TCP offset y: %f\n", rtde_receive.getActualTCPPose().at(1));
    printf("TCP offset z: %f\n", rtde_receive.getActualTCPPose().at(2));
    printf("TCP offset rx: %f\n", rtde_receive.getActualTCPPose().at(3));
    printf("TCP offset ry: %f\n", rtde_receive.getActualTCPPose().at(4));
    printf("TCP offset rz: %f\n", rtde_receive.getActualTCPPose().at(5));


    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //rtde_control.moveJ(startpos);
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    /*for (int j = 0; j < steps; j++)
	{   
        if(j < 1000)
        {
            joint_speed[0] = sgn<float>(1)*max_vel;
        }else
        {
            joint_speed[0] = sgn<float>(-1)*max_vel;
        }
        
        auto t_start = high_resolution_clock::now();
		rtde_control.speedJ(joint_speed, std::abs(0.5),0.0001);
		auto t_stop = high_resolution_clock::now();
    	auto t_duration = std::chrono::duration<double>(t_stop - t_start);
		if (t_duration.count() < sT)
		{
			std::this_thread::sleep_for(std::chrono::duration<double>(sT - t_duration.count()));
		}		
	}*/
	rtde_control.speedStop();
    rtde_control.stopScript();
	rtde_receive.disconnect();
    return 0;
}
