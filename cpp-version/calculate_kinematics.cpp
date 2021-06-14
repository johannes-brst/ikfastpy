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

std::vector<double> solutions[8];

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
    std::vector<float> result = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  for (int i = 0; i <  vec1.size(); ++i) {
      result[i] = vec1[i] - vec2[i];
  }
  return result;
}

int findClosestSolution(std::vector<double> actual_q)
{
    printf("findClosestSolution\n");
    int index_min = 0;
    float min = 999.0f;
    float difference = 0.0f;
    for(int i = 0; i < 8; i++){
        if(sizeof(solutions[i]) > 1){
            difference = 0.0f;
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
    std::vector<float> abs_tmp_speed= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    for(int i = 0; i < tmp_speed.size() - 1; i++){
        abs_tmp_speed[i] = std::abs(tmp_speed[i]);
    }
    for(int i = 0; i < joint_config.size() ; i++){
        if(std::abs(tmp_speed[i]) > 0.002f){
            joint_speed[i] = tmp_speed[i];
            if(*max_element(abs_tmp_speed.begin(),abs_tmp_speed.end()) > 3.14f){
                joint_speed[i] = tmp_speed[i]*(max_vel/ *max_element(abs_tmp_speed.begin(),abs_tmp_speed.end()));
            }
        }
        else{
            joint_speed[i] = 0.0f;
        }
    }

    return joint_speed;
}

void calculateKinematics(std::vector<float> ee_pose){
    Kinematics ur5e_kin;
    int n_joints = ur5e_kin.num_of_joints;
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
    return;
}

void moveArm(std::string ip, std::vector<float> ee_pose){

    calculateKinematics(ee_pose);
    printf("\n");

    float sT = 0.002; //sample Time
    RTDEControlInterface rtde_control(ip);
    RTDEReceiveInterface rtde_receive(ip);
    std::vector<double> actual_q = rtde_receive.getActualQ();
    
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    
    
    std::vector<double> goal = solutions[findClosestSolution(actual_q)];
    
    float max_vel = 3.14f;
    float acceleration = 40.0f;
    float dt = 0.02f;
    actual_q = rtde_receive.getActualQ();

    printf("current joint angles:\n");
    for (int i = 0; i <= actual_q.size() - 1; i++) 
        std::cout << actual_q[i] << " ";
    printf("\n");
    joint_speed = newJointSpeed(goal, actual_q, joint_speed, max_vel);

    bool is_all_zero = true;
    for(int i = 0; i < joint_speed.size(); i++){
        if(joint_speed[i] != 0.0f){
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
            if(joint_speed[i] != 0.0f){
                is_all_zero = false;
                break;
            }
        }
    }

    rtde_control.speedJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, acceleration,dt);
    actual_q = rtde_receive.getActualQ();

    printf("reached joint angles:\n");
    for (int i = 0; i <= actual_q.size() - 1; i++) 
        std::cout << actual_q[i] << " ";
    printf("\n");

	rtde_control.speedStop();
    rtde_control.stopScript();
	rtde_receive.disconnect();  
}


int main(int argc, char *argv[])
{

    // TODO: rodrigues calculation in cpp for rotation vector -> rotation matrix
    //std::vector<float> ee_pose = { 0.99948378f, 0.01387675f, -0.02897591f, -0.000905f,
    //-0.02259421f ,-0.33758036f ,-0.94102551f  ,-0.330611f,
    //-0.02284007f,  0.94119442f ,-0.33709256f  ,0.375722f};

    // alternativly for testing use the forward calcution from ikfast: 
    Kinematics ur5e_kin;
    int n_joints = ur5e_kin.num_of_joints;

    //wished end position
    std::vector<float> joint_angles = {0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717};

    std::vector<float> ee_pose = ur5e_kin.forward(joint_angles);

    moveArm("127.0.0.1", ee_pose);
    return 0;
}
