
#include "globalOptimization.h"
#include "graphFactors.h"

GlobalOptimization::GlobalOptimization()
{
    newUWB = false;
    newUWBdistance = false;
    WUWB_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&GlobalOptimization::optimize, this); // Create new thread for optimize()
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

/* Add odom pose to localPoseMap, globalPoseMap and global_path */
void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
    mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(),
                             OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;

    /* Calculate globalPoseMap and global_path */
    Eigen::Quaterniond globalQ;
    /* Fetch a <3, 3> block start from (0,0), which is a rotation matrix,
       a <3, 1> block start from (0,3), which is a Translation vector. */
    globalQ = WUWB_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WUWB_T_WVIO.block<3, 3>(0, 0) * OdomP + WUWB_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

// void GlobalOptimization::getUWBanchors(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ,
                                    //   Eigen::Vector3d (&anchorP)[4])
void GlobalOptimization::getUWBanchors(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ,
                                      Eigen::MatrixXd &anchorP)
{
    odomP = lastP;
    odomQ = lastQ;
    // for (int i = 0; i < sizeof(anchorP)/sizeof(anchorP[0]); i++)
    // {
    //     anchorP[i] = last_UWB_anchorsP[i]; // Fixed order of 4 ahchors Positions.
    // }
    anchorP = last_UWB_anchorsPs;
}

/* Use the same global factor with UWB as they not appear in the same situation. */
void GlobalOptimization::inputUWB(double t, double x, double y, double z, double posAccuracy)
{
    vector<double> tmp{x, y, z, posAccuracy};
    UWBPositionMap[t] = tmp; // For the same t exist in this Map, replace and update the data.
    newUWB = true;
}

/* Use the same global factor with UWB as they not appear in the same situation. */
void GlobalOptimization::inputUWBdistance(double t, double x, double y, double z,
    double distance, double disAccuracy)
{
    // vector<double> tmp{x, y, z, distance, disAccuracy};
    // // printf("new uwb: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
    // UWBDistanceMap[t] = tmp;

    /* Which UWB anchors, to update last_UWB_anchorsPs map. */
    int anchor_i = 0;
    bool find_anchor = false;
    for (anchor_i = 0; anchor_i < last_UWB_anchorsPs.cols(); anchor_i++)
    {
        if (norm(x - last_UWB_anchorsPs(0,anchor_i))<0.01 &&
            norm(y - last_UWB_anchorsPs(1,anchor_i))<0.01 &&
            norm(z - last_UWB_anchorsPs(2,anchor_i))<0.01)
            {
                vector<double> tmp2{x, y, z,
                                    last_UWB_anchorsPs(0,anchor_i),
                                    last_UWB_anchorsPs(1,anchor_i),
                                    last_UWB_anchorsPs(2,anchor_i),
                                    distance, disAccuracy};
                globalAnchorMap[t]= tmp2; // Use last estimate as [3]-[5]
                find_anchor = true;
                break;
            }
    }

    // /* No need update last_UWB_anchorsPs in this context. */
    // if (find_anchor)
    // {
    //     /* Update coresponding anchors estimation with the newest position. */
    //     map<double, vector<double>>::reverse_iterator iter;
    //     for (iter = globalAnchorMap.rbegin(); iter != globalAnchorMap.rend(); iter++)
    //     {
    //         if (norm(x - iter->second[0])<0.01 &&
    //             norm(y - iter->second[1])<0.01 &&
    //             norm(z - iter->second[2])<0.01)
    //         {
    //             last_UWB_anchorsPs(3,anchor_i)= iter->second[3];
    //             last_UWB_anchorsPs(4,anchor_i)= iter->second[4];
    //             last_UWB_anchorsPs(5,anchor_i)= iter->second[5];
    //         }
    //     }
    // }

     /* Add a new anchor to anchor map. */
    // if (anchor_i >= last_UWB_anchorsPs.cols()) // Only equivalent will exist.
    if (!find_anchor)
    {
        int last_anchor_num = last_UWB_anchorsPs.cols();
        /* 6 = anchor_initial_pos_dimension + anchor_position_estimate_dimension. */
        Eigen::MatrixXd new_UWB_anchorsPs(6, last_anchor_num + 1);
        for (int i = 0; i < last_anchor_num; i++)
        {
            new_UWB_anchorsPs.block<6, 1>(0,i) = last_UWB_anchorsPs.block<6, 1>(0,i);
        }

        last_UWB_anchorsPs = new_UWB_anchorsPs; // maybe need a lock (but note another thread).

        /* Add initial anchors position to map from given data. */
        last_UWB_anchorsPs(0,anchor_i)= x;
        last_UWB_anchorsPs(1,anchor_i)= y;
        last_UWB_anchorsPs(2,anchor_i)= z;
         /* Initial anchors estimation using anchor's initial position. */
        last_UWB_anchorsPs(3,anchor_i)= x;
        last_UWB_anchorsPs(4,anchor_i)= y;
        last_UWB_anchorsPs(5,anchor_i)= z;

        vector<double> tmp2{x, y, z, x, y, z, distance, disAccuracy};
        globalAnchorMap[t]= tmp2; // better to use last estimate as [3]-[5]
    }

    newUWBdistance = true;
}

void GlobalOptimization::optimize()
{
    while(true)
    {
        bool global_use_distance = false; // true; // Use distance measurement or not
        if(newUWB)
        {
            newUWB = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            /* Create a new problem to be solved */
            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 10; // 5;
            ceres::Solver::Summary summary;
            // ceres::LossFunction *loss_function;
            ceres::LossFunction *loss_function, *loss_function_anchors;
            loss_function = new ceres::HuberLoss(1.0);
            loss_function_anchors = new ceres::HuberLoss(1.0);
            // ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
            ceres::Manifold* local_parameterization = new ceres::QuaternionManifold(); // Manifold

            /* Add parameters for the optimization problem, the cost function */
            mPoseMap.lock();
            /* Add local poses in localPoseMap to factor graph */
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];

            double p_array[length][3]; // UWB anchors' position variables to be estimated

            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization); // Variables to be optimized
                problem.AddParameterBlock(t_array[i], 3);
            }

            if(newUWBdistance && (globalAnchorMap.size() > 0) && global_use_distance)
            {
                // int lengthAnchorMap = globalAnchorMap.size(); // TODO when same with global pose sizes.
                map<double, vector<double>>::iterator iterAnchor;
                iterAnchor = globalAnchorMap.begin();
                for (int i = 0; iterAnchor != globalAnchorMap.end(); i++, iterAnchor++)
                {
                    p_array[i][0] = iterAnchor->second[3];
                    p_array[i][1] = iterAnchor->second[4];
                    p_array[i][2] = iterAnchor->second[5];
                    problem.AddParameterBlock(p_array[i], 3);
                }
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterUWB;

            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                /* Add vio factor */
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4],
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    /* Add VIO relative (R,T) error. NULL: using identity loss function. */
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
                }

                /* Add UWB factor using UWB position measurements */
                double t = iterVIO->first;
                iterUWB = UWBPositionMap.find(t); // Find UWB poses with the same t
                if (iterUWB != UWBPositionMap.end())
                {   /* UWB cost functions. */
                    ceres::CostFunction* uwb_function = TError::Create(iterUWB->second[0], iterUWB->second[1],
                                                                       iterUWB->second[2], iterUWB->second[3]);
                    //printf("inverse weight %f \n", iterUWB->second[3]);
                    problem.AddResidualBlock(uwb_function, loss_function, t_array[i]);
                }
            }

            /* Add UWB range factor */
            if(newUWBdistance && (globalAnchorMap.size() > 0)&& global_use_distance)
            {
                map<double, vector<double>>::iterator iterUWBrange; // , iterGlobal;
                int i = 0;
                for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
                // for (iterGlobal = globalPoseMap.begin(); iterGlobal != globalPoseMap.end(); iterGlobal++, i++)
                {
                    double t = iterVIO->first;
                    map<double, vector<double>>::iterator iterGlobal, iterUWBAnchor;
                    // map<double, vector<double>>::iterator iterGlobal, iterUWBAnchor, iterAnchorNext;
                    iterGlobal = globalPoseMap.find(t);
                    iterUWBAnchor = globalAnchorMap.find(t);
                    // iterAnchorNext = iterUWBAnchor;
                    // iterAnchorNext++;
                    if (iterUWBAnchor != globalAnchorMap.end() && iterGlobal != globalPoseMap.end())
                    {   /* UWB cost functions, and the mearsurements . */
                        ceres::CostFunction* uwbrange_function = DError::Create(
                            // DError::Create(iterUWBrange->second[3], iterUWBrange->second[4]);
                            // iterVIO->second[0], iterVIO->second[1], iterVIO->second[2], iterUWBrange->second[3], iterUWBrange->second[4]);
                            iterGlobal->second[0], iterGlobal->second[1], iterGlobal->second[2],
                            // iterUWBrange->second[3], iterUWBrange->second[4]);
                            iterUWBAnchor->second[6], iterUWBAnchor->second[7]);
                            // TODO: Add constant condiction of anchor position ! ******** ! //
                        problem.AddResidualBlock(uwbrange_function, loss_function_anchors, p_array[i]);
                        // double tmp[3] = {p_array[i][0],p_array[i][1],p_array[i][2]};
                        // problem.AddResidualBlock(uwbrange_function, loss_function_anchors, tmp);

                        // /* UWB anchor relative position. */
                        // for (int iNext = 1; iterAnchorNext != globalAnchorMap.end(); iterAnchorNext++)
                        // {
                        //     if (norm(iterAnchorNext->second[0] - iterUWBAnchor->second[0])<0.01 &&
                        //         norm(iterAnchorNext->second[1] - iterUWBAnchor->second[1])<0.01 &&
                        //         norm(iterAnchorNext->second[2] - iterUWBAnchor->second[2])<0.01)
                        //     {
                        //         iNext = distance(iterUWBAnchor, iterAnchorNext);
                        //         ceres::CostFunction* uwbrange_function = PError::Create(1);
                        //         problem.AddResidualBlock(uwbrange_function, loss_function_anchors, p_array[i], p_array[i+iNext]);
                        //         break;
                        //     }
                        // }
                    }
                }
            }

            // mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << "\n";

            /* update global pose, with the optimization results. */
            // mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
                                          q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
                iter->second = globalPose; // save the solved global poses
                if(i == length - 1) // Get the final transformation from VIO to UWB
                {
                    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d WUWB_T_body = Eigen::Matrix4d::Identity();
                    double t = iter->first;
                    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4],
                                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
                    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
                    WUWB_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
                                                                        globalPose[5], globalPose[6]).toRotationMatrix();
                    WUWB_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
                    WUWB_T_WVIO = WUWB_T_body * WVIO_T_body.inverse();
                    std::cout << "WUWB_T_WVIO:" << "\n" << WUWB_T_WVIO << "\n";
                }
            }
            updateGlobalPath();

            /* update anchors' positions, with the optimization results. */
            if(newUWBdistance && (globalAnchorMap.size() > 0) && global_use_distance)
            {
                newUWBdistance = false;
                map<double, vector<double>>::iterator iterUWBAnchorMap;
                iterUWBAnchorMap = globalAnchorMap.begin();
                // for (int i = 0; i < length; i++, iter++)
                for (int i = 0; iterUWBAnchorMap != globalAnchorMap.end(); i++, iterUWBAnchorMap++)
                {
                    vector<double> uwbAnchorPosition{iterUWBAnchorMap->second[0], iterUWBAnchorMap->second[1], iterUWBAnchorMap->second[2],
                                                     p_array[i][0], p_array[i][1], p_array[i][2]};
                    iterUWBAnchorMap->second = uwbAnchorPosition; // save the solved global poses
                }

                updateAnchorMap();
            }

            printf("UWB anchor globalAnchorMap size: %ld; globalPoseMap size: %ld; localPoseMap size: %ld \n",
                    globalAnchorMap.size(), globalPoseMap.size(), localPoseMap.size());
            // printf("UWB anchor optimization: %s(%d): \n" ,__FILE__, __LINE__);

            // printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }

        /* Solve anchors' positions using a new problem. */
        if(newUWBdistance && (globalAnchorMap.size() > 0))
        {
            newUWBdistance = false;

            /* Create a new problem to be solved */
            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function_anchors;
            loss_function_anchors = new ceres::HuberLoss(1.0);

            /* Add parameters for the optimization problem, the cost function */
            mPoseMap.lock();
            /* Add local poses in localPoseMap to factor graph */
            int length = localPoseMap.size();
            double p_array[length][3]; // UWB anchors' position variables to be estimated

            // UWBDistanceMap
            map<double, vector<double>>::iterator iterAnchor;

            iterAnchor = globalAnchorMap.begin();
            for (int i = 0; iterAnchor != globalAnchorMap.end(); i++, iterAnchor++) // TODO
            {
                p_array[i][0] = iterAnchor->second[3];
                p_array[i][1] = iterAnchor->second[4];
                p_array[i][2] = iterAnchor->second[5];
                problem.AddParameterBlock(p_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterUWBrange; // , iterGlobal;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            // for (iterGlobal = globalPoseMap.begin(); iterGlobal != globalPoseMap.end(); iterGlobal++, i++)
            {
                double t = iterVIO->first;
                map<double, vector<double>>::iterator iterGlobal, iterUWBAnchor, iterAnchorNext;
                iterGlobal = globalPoseMap.find(t);
                iterUWBAnchor = globalAnchorMap.find(t);
                iterAnchorNext = iterUWBAnchor;
                iterAnchorNext++;
                // iterUWBrange = UWBDistanceMap.find(t); // Find UWB poses with the same t
                // if (iterUWBrange != UWBDistanceMap.end())
                // TODO: globalPoseMap may not have same length and may not with same position.

                /* UWB cost functions, and the mearsurements. */
                if (iterUWBAnchor != globalAnchorMap.end() && iterGlobal != globalPoseMap.end())
                {
                    ceres::CostFunction* uwbrange_function = DError::Create(
                        // DError::Create(iterUWBrange->second[3], iterUWBrange->second[4]);
                        // iterVIO->second[0], iterVIO->second[1], iterVIO->second[2], iterUWBrange->second[3], iterUWBrange->second[4]);
                        iterGlobal->second[0], iterGlobal->second[1], iterGlobal->second[2],
                        // iterUWBrange->second[3], iterUWBrange->second[4]);
                        iterUWBAnchor->second[6], iterUWBAnchor->second[7]);
                        // TODO: Add constant condiction of anchor position ! ******** ! //
                    problem.AddResidualBlock(uwbrange_function, loss_function_anchors, p_array[i]);
                    // double tmp[3] = {p_array[i][0],p_array[i][1],p_array[i][2]};
                    // problem.AddResidualBlock(uwbrange_function, loss_function_anchors, tmp);

                    // /* UWB anchor relative position. */
                    // for (int iNext = 1; iterAnchorNext != globalAnchorMap.end(); iterAnchorNext++)
                    // {
                    //     if (norm(iterAnchorNext->second[0] - iterUWBAnchor->second[0])<0.01 &&
                    //         norm(iterAnchorNext->second[1] - iterUWBAnchor->second[1])<0.01 &&
                    //         norm(iterAnchorNext->second[2] - iterUWBAnchor->second[2])<0.01)
                    //     {
                    //         iNext = distance(iterUWBAnchor, iterAnchorNext);
                    //         // cout<<"distance: " << distance(iterUWBAnchor, iterAnchorNext)<<
                    //         //     ": " << norm(iterAnchorNext->second[0] - iterUWBAnchor->second[0]) <<
                    //         //     ", " << norm(iterAnchorNext->second[1] - iterUWBAnchor->second[1]) <<
                    //         //     ", " << norm(iterAnchorNext->second[2] - iterUWBAnchor->second[2]) << endl;
                    //         ceres::CostFunction* uwbrange_function = PError::Create(1);
                    //         problem.AddResidualBlock(uwbrange_function, loss_function_anchors, p_array[i], p_array[i+iNext]);
                    //         break;
                    //     }
                    // }
                }
            }

            // mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << "\n";

            /* update anchors' positions, with the optimization results. */
            map<double, vector<double>>::iterator iterUWBAnchorMap;
            iterUWBAnchorMap = globalAnchorMap.begin();
            // for (int i = 0; i < length; i++, iter++)
            for (int i = 0; iterUWBAnchorMap != globalAnchorMap.end(); i++, iterUWBAnchorMap++)
            {
                vector<double> uwbAnchorPosition{iterUWBAnchorMap->second[0], iterUWBAnchorMap->second[1], iterUWBAnchorMap->second[2],
                                                    p_array[i][0], p_array[i][1], p_array[i][2]};
                iterUWBAnchorMap->second = uwbAnchorPosition; // save the solved global poses
            }
            updateAnchorMap();

            mPoseMap.unlock();
        }

        /*  Update rate for optimization calculation (2000; can be smaller if the parameters are manifolds and fast enough). */
        std::chrono::milliseconds dura(5000); // Should be bigger as the distance is not a manifolds and ill-condiction in z.
        std::this_thread::sleep_for(dura);
    }
    return;
}

void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}

void GlobalOptimization::updateAnchorMap()
{
    /* Update coresponding anchors estimation map with the newest Map. */
    int anchor_i = 0;
    for (anchor_i = 0; anchor_i < last_UWB_anchorsPs.cols(); anchor_i++)
    {
        // globalAnchorMap.clear(); // Works for a non-pointer map
        map<double, vector<double>>::reverse_iterator iter;
        for (iter = globalAnchorMap.rbegin(); iter != globalAnchorMap.rend(); iter++)
        {
            if (norm(last_UWB_anchorsPs(0,anchor_i) - iter->second[0])<0.01 &&
                norm(last_UWB_anchorsPs(1,anchor_i) - iter->second[1])<0.01 &&
                norm(last_UWB_anchorsPs(2,anchor_i) - iter->second[2])<0.01)
            {
                last_UWB_anchorsPs(3,anchor_i)= iter->second[3];
                last_UWB_anchorsPs(4,anchor_i)= iter->second[4];
                last_UWB_anchorsPs(5,anchor_i)= iter->second[5];
                // printf("UWB anchor map: %f, %f, %f: \n" ,iter->second[3],
                //             iter->second[4],
                //             iter->second[5]);
                break;
            }
        }
    }
    cout << "last UWB anchors map:" << "\n" << last_UWB_anchorsPs << "\n";
}