#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <cmath>        // std::abs
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <numeric>
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
std::vector<double> vecSubtract(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<double> result(vec1.size(),0.0);
  for (int i = 0; i <  vec1.size(); ++i) {
      result.at(i) = vec1.at(i) - vec2.at(i);
  }
  return result;
}

template<typename T>
std::vector<double> vecAdd(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<double> result(vec1.size(),0.0);
  for (int i = 0; i <  vec1.size(); ++i) {
      result.at(i) = vec1.at(i) + vec2.at(i);
  }
  return result;
}

template<typename T>
std::vector<double> vecAddValue(const std::vector<T>& vec1, const T value) {
    std::vector<double> result(vec1.size(),0.0);
    for (int i = 0; i <  vec1.size(); ++i) {
        result.at(i) = vec1.at(i) + value;
    }
    return result;
}

int findClosestSolution(std::vector<double> actual_q)
{
    printf("findClosestSolution\n");
    int index_min = 0;
    double min = 999.0;
    double difference = 0.0;
    for(int i = 0; i < 8; i++){
        if(sizeof(solutions[i]) > 1){
            difference = 0.0;
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

/* calculates new joint speed based on max velocity and remaining angle difference.
If no joint has a distance more than 3.14 the joint speed equals the difference from actual to goal (per joint) multiplied by a constant*/
std::vector<double> newJointSpeed(std::vector<double> joint_config, std::vector<double> actual_q, std::vector<double> joint_speed, double max_vel)
{
    //printf("newJointSpeed\n");
    std::vector<double> tmp_speed = vecSubtract(joint_config, actual_q);
    std::vector<double> abs_tmp_speed= {0.0,0.0,0.0,0.0,0.0,0.0};
    for(int i = 0; i < tmp_speed.size() - 1; i++){
        abs_tmp_speed[i] = std::abs(tmp_speed[i]);
    }
    for(int i = 0; i < joint_config.size() ; i++){
        if(std::abs(tmp_speed[i]) > 0.002){
            joint_speed[i] = std::min(tmp_speed[i]*5,max_vel);
            if(*max_element(abs_tmp_speed.begin(),abs_tmp_speed.end()) > 3.14){
                joint_speed[i] = tmp_speed[i]*(max_vel/ *max_element(abs_tmp_speed.begin(),abs_tmp_speed.end()));
            }
        }
        else{
            joint_speed[i] = 0.0f;
        }
    }

    return joint_speed;
}

/*calculate kinematics and gather all solutions*/
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

    // split solution from joint_configs into seperate vectors
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

/* move the arm with ur_rtde to the best found solution*/
void moveArm(std::string ip, std::vector<float> ee_pose){

    calculateKinematics(ee_pose);
    printf("\n");

    double sT = 0.002; //sample Time
    RTDEControlInterface rtde_control(ip);
    RTDEReceiveInterface rtde_receive(ip);
    //std::vector<double> startpos = {1.755020, -1.131027, 2.141150,  0.456918,  0.842970, -0.346717};
    //rtde_control.moveJ(startpos);

    std::vector<double> actual_q = rtde_receive.getActualQ();
    
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    
    
    std::vector<double> goal = solutions[findClosestSolution(actual_q)];
    
    double max_vel = 3.14;
    double acceleration = 40.0;
    double dt = 0.02;
    actual_q = rtde_receive.getActualQ();

    printf("current joint angles:\n");
    for (int i = 0; i <= actual_q.size() - 1; i++) 
        std::cout << actual_q[i] << " ";
    printf("\n");
    joint_speed = newJointSpeed(goal, actual_q, joint_speed, max_vel);

    bool is_all_zero = true;
    for(int i = 0; i < joint_speed.size(); i++){
        if(joint_speed[i] != 0.0){
            is_all_zero = false;
            break;
        }
    }

    //move arm until the goal position is reached
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
            if(joint_speed[i] != 0.0){
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


template<typename T>
std::vector<double>  DivideVectorByValue(const std::vector<T>& v, const T k){
    std::vector<double> result(v.size(), 0.0);// 
    for (int i = 0; i < v.size(); ++i) {
        result.at(i) = v.at(i) / k;
    }

    return result;
}

template<typename T>
std::vector<double>  MultiplyVectorByValue(const std::vector<T>& v, const T k){
    std::vector<double> result(v.size(), 0.0);// 
    for (int i = 0; i < v.size(); ++i) {
        result.at(i) = v.at(i) * k;
    }

    return result;
}

template<typename T>
std::vector<double>  MultiplyVector(const std::vector<T>& v, const std::vector<T>& k){
    std::vector<double> result(v.size(), 0.0);// 
    for (int i = 0; i < v.size(); ++i) {
        result.at(i) = v.at(i) * k.at(i);
    }

    return result;
}
   
template<typename T>
std::vector<double> S(const std::vector<T>& n){
    std::vector<double> Sn(9, 0.0);// 
    Sn.at(1) = n.at(2) * -1;
    Sn.at(2) = n.at(1);
    Sn.at(3) = n.at(2);
    Sn.at(5) = n.at(0) * -1;
    Sn.at(6) = n.at(1) * -1;
    Sn.at(7) = n.at(0);
    return Sn;
}

/* WIP Rodrigues formula
Input: 1x3 array of rotations about x, y, and z
Output: 3x3 rotation matrix*/
std::vector<double> rodrigues(std::vector<double>& r){
    double norm = 0.0;
    for (int i = 0; i < r.size(); ++i) {
        norm += r.at(i) * r.at(i);
    }
    double theta = norm;
    std::vector<double> R(9, 0.0);// 
    if(theta > 1e-30){
        std::vector<double> n = DivideVectorByValue(r, theta);
        std::vector<double> Sn = S(n);
        std::vector<double> eye(9, 0.0);// 
        eye.at(0) = 1.0;
        eye.at(4) = 1.0;
        eye.at(8) = 1.0;
        std::vector<double> mult = MultiplyVectorByValue(Sn, sin(theta));
        R = vecAdd(eye, mult);
        R = vecAddValue(R, ((1 - std::cos(theta)) * std::inner_product(std::begin(Sn), std::end(Sn), std::begin(Sn), 0.0)));
    }
    else{
        std::vector<double> Sr = S(r);
        double theta2 = theta*theta;
        std::vector<double> eye(9, 0.0);// 
        eye.at(0) = 1.0;
        eye.at(4) = 1.0;
        eye.at(8) = 1.0;
        R = vecAdd(eye, MultiplyVectorByValue(Sr, 1 - sin(theta2)/6.0));
        R = vecAddValue(R, (0.5f - std::cos(theta2)/24.0f) * std::inner_product(std::begin(Sr), std::end(Sr), std::begin(Sr), 0.0));
    }
    
    return R;
}

int main(int argc, char *argv[])
{

    // TODO: rodrigues calculation in cpp for rotation vector -> rotation matrix
    //std::vector<double> ee_pose = { 0.99948378f, 0.01387675f, -0.02897591f, -0.000905f,
    //-0.02259421f ,-0.33758036f ,-0.94102551f  ,-0.330611f,
    //-0.02284007f,  0.94119442f ,-0.33709256f  ,0.375722f};

    // alternativly for testing use the forward calcution from ikfast: 
    Kinematics ur5e_kin;
    int n_joints = ur5e_kin.num_of_joints;

    //wished end position
    std::vector<float> joint_angles = {0.755020, -2.131027, 2.141150,  0.456918,  0.842970, -0.346717};

    std::vector<float> ee_pose = ur5e_kin.forward(joint_angles);

    moveArm("127.0.0.1", ee_pose);

    printf("Starting rodrigues test:\n");
    std::vector<double> test = {1.914787, -0.006242, -0.037102};
    std::vector<double> rod_test(9);
    rod_test = rodrigues(test);
    printf("Test: \n");
    for(int j = 0; j < rod_test.size() - 1; j++){
        std::cout << rod_test.at(j);        
        printf("\n");
    }

    return 0;
}
