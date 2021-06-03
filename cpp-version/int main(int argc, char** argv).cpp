int main(int argc, char** argv)
{
    IKREAL_TYPE eerot[9],eetrans[3];

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#else
    // for IKFast 54
    unsigned int num_of_joints = getNumJoints();
    unsigned int num_free_parameters = getNumFreeParameters();
#endif
 
    std::string cmd;
    if (argv[1]) cmd = argv[1];

    //printf("command: %s \n\n", cmd.c_str() );

    if (cmd.compare("ik") == 0) // ik mode
    {
    } // endif ik mode

    else if (cmd.compare("fk") == 0) // fk mode
    {
        

    }
    else if (cmd.compare("iktiming") == 0) // generate random ik and check time performance
    {
        if( argc != 2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute iktiming \n\n"
                   "         For fixed number of iterations, generates random joint angles, then  \n"
                   "         calculates fk, calculates ik, measures average time taken. \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        srand( (unsigned)time(0) ); // seed random number generator
        float min = -3.14;
        float max = 3.14;
        float rnd;

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
        unsigned int num_of_tests = 1000000; 
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000; 
#endif

        for (unsigned int i=0; i < num_of_tests; i++)
        {
            // Measure avg time for whole process
            //clock_gettime(CLOCK_REALTIME, &start_time); 

            // Put random joint values into array
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                float rnd = (float)rand() / (float)RAND_MAX;
                joints[i] = min + rnd * (max - min);
            }
            /*
            printf("Joint angles:  ");
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                printf("%f  ", joints[i] );
            }
            printf("\n");
            */

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeFk(joints, eetrans, eerot); // void return
#else
            // for IKFast 54
            fk(joints, eetrans, eerot); // void return
#endif

            // Measure avg time for IK
            clock_gettime(CLOCK_REALTIME, &start_time);
#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 
            */

            clock_gettime(CLOCK_REALTIME, &end_time); 
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests; 
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

        return 1;
  
    }
    else if (cmd.compare("iktiming2") == 0) // for fixed joint values, check time performance of ik
    {
        if( argc != 2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute iktiming2 \n\n"
                   "         For fixed number of iterations, with one set of joint variables, this  \n"
                   "         finds the ik solutions and measures the average time taken. \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
        unsigned int num_of_tests = 1000000; 
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000; 
#endif

        // fixed rotation-translation matrix corresponding to an unusual robot pose
        eerot[0] = 0.002569;  eerot[1] = -0.658044;  eerot[2] = -0.752975;  eetrans[0] = 0.121937;
        eerot[3] = 0.001347;  eerot[4] = -0.752975;  eerot[5] = 0.658048;  eetrans[1] = -0.276022;
        eerot[6] = -0.999996; eerot[7] = -0.002705; eerot[8] = -0.001047; eetrans[2] = 0.005685;

        for (unsigned int i=0; i < num_of_tests; i++)
        {
            clock_gettime(CLOCK_REALTIME, &start_time);

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 
            */

            clock_gettime(CLOCK_REALTIME, &end_time); 
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests; 
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

        return 1;

    } else {
        printf("\n"
               "Usage: \n\n");
        printf("         ./compute fk j0 j1 ... j%d \n\n"
               "         Returns the forward kinematic solution given the joint angles (in radians). \n\n", num_of_joints-1 );
        printf("\n"
               "         ./compute ik  t0 t1 t2  qw qi qj qk  free0 ... \n\n"
               "         Returns the ik solutions given the transformation of the end effector specified by \n"
               "         a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k). \n"
               "         There are %d free parameters that have to be specified. \n\n", num_free_parameters );
        printf(" \n"
               "         ./compute ik  r00 r01 r02 t0  r10 r11 r12 t1  r20 r21 r22 t2  free0 ...\n\n"
               "         Returns the ik solutions given the transformation of the end effector specified by \n"
               "         a 3x3 rotation R (rXX), and a 3x1 translation (tX). \n"
               "         There are %d free parameters that have to be specified. \n\n", num_free_parameters );
        printf("\n"
               "         ./compute iktiming \n\n"
               "         For fixed number of iterations, generates random joint angles, then \n"
               "         calculates fk, calculates ik, measures average time taken. \n\n", num_of_joints-1 );
        printf("\n"
               "         ./compute iktiming2 \n\n"
               "         For fixed number of iterations, with one set of joint variables, this \n"
               "         finds the ik solutions and measures the average time taken. \n\n", num_of_joints-1 );

        return 1;
    }

    return 0;
}