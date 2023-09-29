/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <yaml-cpp/yaml.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <thread>
#include <numeric>
#include <queue>


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <measurement_msgs/Range.h>

#include <ra_sam/range_factor_w_params.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <ra_sam/odometry_integrator.h>
#include <std_msgs/Bool.h>


namespace sam
{
    struct State
    {
        State()
        {
            stamp = -1;
            vertex_id = -1;
            nav_state = NavState();
            pose_cov.fill(0);
            lin_vel_cov.setIdentity();
            ang_vel_cov.setIdentity();
        }
        double stamp;
        uint64_t vertex_id;
        NavState nav_state;
        imuBias::ConstantBias imu_bias;
        Eigen::Matrix<double, 6, 6> pose_cov;
        Eigen::Matrix<double, 3, 3> lin_vel_cov;
        Eigen::Matrix<double, 3, 3> ang_vel_cov;
    };
    //
    struct AnchorInfo
    {
        AnchorInfo()
        {
            id = -1;
            name = "";
            pos.fill(0);
            pos_cov.setIdentity();
            noise_params = std::make_shared<NoiseParamsGaussian1D>();
        }
        int id;
        std::string name;
        Eigen::Vector3d pos;
        Eigen::Matrix3d pos_cov;
        NoiseParamsGaussian1DPtr noise_params;
    };

    //
    template<class T>
    inline void clamp(T& v, const T& lo, const T& hi )
    {
        assert( !(hi < lo) );
        v = ((v < lo) ? lo : (hi < v) ? hi : v);
    }

}