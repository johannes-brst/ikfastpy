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

template<typename T>
std::string vec2csv(const std::vector<T>& vec) {
  std::string s;
  for (typename std::vector<T>::const_iterator it = vec.begin(); it != vec.end(); ++it) {

    // Simpler, but requires boost and offers minimal formatting control
    //s += boost::lexical_cast<std::string>(*it);

    // Doesn't require boost, offers more formatting control
    std::ostringstream ss;
    //ss << std::fixed << std::setprecision(2);
    ss << *it;
    s += ss.str();
    s += ", ";

  }
  if (s.size() >= 2) {   // clear the trailing comma, space
    s.erase(s.size()-2);
  }
  return s;
}

template<typename T>
std::vector<float> vecSubtract(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<float> result = {0,0,0,0,0,0};
  for (int i = 0; i <  vec1.size(); ++i) {
      result[i] = vec1[i] - vec2[i];
  }
  return result;
}

int findClosestSolution(std::vector<double> *solutions, std::vector<double> actual_q)
{
    printf("findClosestSolution\n");
    int index_min = 0;
    float min = 999;
    float difference = 0;
    for(int i = 0; i < 8; i++){
        if(sizeof(solutions[i]) > 1){
            difference = 0;
            for(int j = 0; j < actual_q.size()-1; j++){
                difference += std::abs(solutions[i][j] - actual_q[j]);
            }
            if(difference < min){
                min = difference;
                index_min = i;
            }
        }  
    }
    std::cout << vec2csv(solutions[index_min]);
    printf("\n");
    return index_min;
}

std::vector<double> newJointSpeed(std::vector<double> joint_config, std::vector<double> actual_q, std::vector<double> joint_speed, float max_vel)
{
    //printf("newJointSpeed\n");
    std::vector<float> tmp_speed = vecSubtract(joint_config, actual_q);
    std::vector<float> abs_tmp_speed= {0,0,0,0,0,0};
    for(int i = 0; i < tmp_speed.size() - 1; i++){
        abs_tmp_speed[i] = std::abs(tmp_speed[i]);
    }
    for(int i = 0; i < joint_config.size() - 1; i++){
        if(std::abs(tmp_speed[i]) > 0.02){
            joint_speed[i] = tmp_speed[i]*(max_vel/ *max_element(abs_tmp_speed.begin(),abs_tmp_speed.end()));
        }
        else{
            joint_speed[i] = 0.0f;
        }
    }

    return joint_speed;
}


int main(int argc, char *argv[])
{
    Kinematics ur5e_kin;
    int n_joints = ur5e_kin.num_of_joints;
    printf("n_joints = %i\n",n_joints);
    // Test forward kinematics: get end effector pose from joint angles
    std::vector<float> joint_angles = {0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717};

    printf("Joint angles:\n");
    for (int i = 0; i <= n_joints - 1; i++) 
        std::cout << joint_angles[i] << " ";
    printf("\n");
    std::vector<float> ee_pose = ur5e_kin.forward(joint_angles);

    // std::vector<float> ee_pose = {0.9994 , 0.013, -0.028,  -0.002,
    //                           -0.022 ,-0.3375, -0.9410,  -0.330,
    //                         -0.02284,  0.9411, -0.3370,  0.375};

    std::vector<float> joint_configs = ur5e_kin.inverse(ee_pose);

    printf("\n");
    
    std::vector<double> sol1 =  {0};
    std::vector<double> sol2 =  {0};
    std::vector<double> sol3 =  {0};
    std::vector<double> sol4 =  {0};
    std::vector<double> sol5 =  {0};
    std::vector<double> sol6 =  {0};
    std::vector<double> sol7 =  {0};
    std::vector<double> sol8 =  {0};
    std::vector<double> solutions[8];

    for (int i = 0; i <= joint_configs.size() - 1; i++){
        if(i < 6){
            sol1 = std::vector<double>(joint_configs.begin(), joint_configs.begin() + 6);
            i+=5;
            solutions[0] = sol1;
        }
        else if(i < 12){
            sol2 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[1] = sol2;
        }
        else if(i < 18){
            sol3 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[2] = sol3;
        }
        else if(i < 24){
            sol4 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[3] = sol4;
        }
        else if(i < 30){
            sol5 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[4] = sol5;
        }
        else if(i < 36){
            sol6 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[5] = sol6;
        }
        else if(i < 42){
            sol7 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[6] = sol7;
        }
        else if(i < 48){
            sol8 = std::vector<double>(joint_configs.begin() + i, joint_configs.begin() + i + 6);
            i+=5;
            solutions[7] = sol8;
        }
    } 
    printf("Solutions: \n");
    for(int j = 0; j < 8; j++){
        std::vector<double> temp = solutions[j];
        std::cout << vec2csv(temp);        
        //printf("%i", j);
        printf("\n");
    }
 
    printf("\n");
    float sT = 0.02; //sample Time
    RTDEControlInterface rtde_control("127.0.0.1");//172.30.10.1
    RTDEReceiveInterface rtde_receive("127.0.0.1");
    std::vector<double> actual_q = rtde_receive.getActualQ();
    std::vector<double> startpos = {1.755020, -1.131027, 2.141150,  0.456918,  0.842970, -0.346717};
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    rtde_control.moveJ(startpos);
    std::vector<double> goal = solutions[findClosestSolution(solutions, actual_q)];
    
    float max_vel = 3.14;
    float acceleration = 40;
    float dt = 0.02;
    actual_q = rtde_receive.getActualQ();
    joint_speed = newJointSpeed(goal, actual_q, joint_speed, max_vel);

    bool is_all_zero = true;
    for(int i = 0; i < joint_speed.size(); i++){
        if(joint_speed[i] != 0){
            is_all_zero = false;
            break;
        }
    }

    while(is_all_zero == false){
        auto t_start = high_resolution_clock::now();
		rtde_control.speedJ(joint_speed, acceleration,dt);
		auto t_stop = high_resolution_clock::now();
    	auto t_duration = std::chrono::duration<double>(t_stop - t_start);
		if (t_duration.count() < sT)
		{
			std::this_thread::sleep_for(std::chrono::duration<double>(sT - t_duration.count()));
		}
        actual_q = rtde_receive.getActualQ();
        joint_speed = newJointSpeed(goal, actual_q, joint_speed, max_vel);

        is_all_zero = true;
        for(int i = 0; i < joint_speed.size(); i++){
            if(joint_speed[i] != 0){
                is_all_zero = false;
                break;
            }
        }
    }

	rtde_control.speedStop();
    rtde_control.stopScript();
	rtde_receive.disconnect();
    return 0;
}
