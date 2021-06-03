#ifndef __IKFAST_WRAPPER_H__
#define __IKFAST_WRAPPER_H__

namespace robots { 
    class Kinematics { 
        public: 
            int num_of_joints, num_free_parameters; 
            Kinematics(); 
            ~Kinematics();
            std::vector<float> forward(std::vector<float> joint_config);
            std::vector<float> inverse(std::vector<float> ee_pose);
    }; 
}
#endif // __IKFAST_WRAPPER_H__