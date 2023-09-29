/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <ra_sam/ra_sam.h>

namespace sam
{
    bool RaSam::loadConfig()
    {
        using namespace YAML;
        //
        printf("SAM config[%s]\n", sam_config_file.c_str());
        sam_config = LoadFile(sam_config_file);
        //
        printf("Robot config[%s]\n", robot_config_file.c_str());
        robot_config = LoadFile(robot_config_file);
        //
        verbose = sam_config["verbose"].as<bool>();
        printf("[%s]: Verbose:[%s] \n", process_name.c_str(), verbose ? "TRUE" : "FALSE");
        //
        smoother_traj_len = sam_config["smoother_traj_length"].as<uint>();
        printf("[%s]: Smoother trajectory length:[%d].\n", process_name.c_str(), smoother_traj_len);
        //
        auto smoother_internal_iterations = sam_config["smoother_internal_iterations"].as<int>();
        printf("[%s]: Smoother internal iterations:[%d].\n", process_name.c_str(),
            sam_config["smoother_internal_iterations"].as<int>());
        //
        imu_sample_factor = 1;
        if(sam_config["downsample"]["imu"])
        {
            int sample_factor = sam_config["downsample"]["imu"].as<int>();
            if(sample_factor > 1)
                imu_sample_factor = sample_factor;
        }
        printf("[%s]: IMU Sample factor:[%d].\n", process_name.c_str(), imu_sample_factor);

        return true;
    }
    //
    bool RaSam::loadUWBConfig()
    {
        //
        uint anc_id = 0;
        for(auto anc : robot_config["uwb"]["anchor_list"].as<std::vector<std::string>>())
        {
            anchor_data[anc] = AnchorInfo();
            anchor_data[anc].pos = Eigen::Vector3d(
                (robot_config["uwb"][anc]["pos"].as<std::vector<double>>()).data());
            anchor_data[anc].noise_params->mu[0] = 
                robot_config["uwb"][anc]["bias"].as<double>();
            anchor_data[anc].noise_params->mu_cov[0] = 
                robot_config["uwb"][anc]["cov"].as<double>();
            anchor_data[anc].noise_params->cov[0] =
                robot_config["uwb"][anc]["cov"].as<double>();
            //
            anchor_vertex_key[anc] = Symbol('j', anc_id++);
        }
        //
        est_anchor_bias = sam_config["est_anchor_bias"].as<bool>();
        printf("[%s]: Estimate anchor biases:[%d].\n", process_name.c_str(), est_anchor_bias);
        //
        return true;
    }
    //
    bool RaSam::loadSensorExtrinsics()
    {
        //
        auto p_ = robot_config["uwb"]["mobile_node"]["pos"].as<std::vector<double>>();
        body_uwb_offset = Eigen::Vector3d(p_[0], p_[1], p_[2]);
        printf("[%s]: UWB in Body offset: pos:[%.3f, %.3f, %.3f].\n", process_name.c_str(),
            body_uwb_offset[0], body_uwb_offset[1], body_uwb_offset[2]);
        //
        body_odom_sensor_ori_offset = Eigen::Quaterniond::Identity();
        body_odom_sensor_pos_offset.fill(0);
        //
        if(robot_config["sensors"]["imu"].as<bool>())
        {
            // load IMU to body offset
            auto p_ = robot_config["imu"]["pos"].as<std::vector<double>>();
            auto o_ = robot_config["imu"]["ori"].as<std::vector<double>>();
            body_odom_sensor_ori_offset = Eigen::Quaterniond(o_[3], o_[0], o_[1], o_[2]), 
            body_odom_sensor_pos_offset = Eigen::Vector3d(p_[0], p_[1], p_[2]);
            //
            printf("[%s]: IMU in Body offset: pos:[%.3f, %.3f, %.3f] ori(wxyz):[%.3f, %.3f, %.3f, %.3f].\n",
                process_name.c_str(), body_odom_sensor_pos_offset[0], body_odom_sensor_pos_offset[1],
                body_odom_sensor_pos_offset[2], body_odom_sensor_ori_offset.w(), body_odom_sensor_ori_offset.x(),
                body_odom_sensor_ori_offset.y(), body_odom_sensor_ori_offset.z());
        }
        // The way VIO is used currently does not require offset since a difference between
        // two VIO estimates is used for dead-reckoning.
        if(robot_config["sensors"]["vio"].as<bool>())
        {
            // Currently it is expected that VIO outputs data in body frame
            // Hence we don't this conversion
            auto p_ = robot_config["vio"]["pos"].as<std::vector<double>>();
            auto o_ = robot_config["vio"]["ori"].as<std::vector<double>>();
            body_odom_sensor_ori_offset = Eigen::Quaterniond(1.0, 0., 0., 0.), 
            body_odom_sensor_pos_offset = Eigen::Vector3d(p_[0], p_[1], p_[2]);
            //
            printf("[%s]: VIO in Body offset: pos:[%.3f, %.3f, %.3f] ori(wxyz):[%.3f, %.3f, %.3f, %.3f].\n",
                process_name.c_str(), body_odom_sensor_pos_offset[0], body_odom_sensor_pos_offset[1],
                body_odom_sensor_pos_offset[2], body_odom_sensor_ori_offset.w(), body_odom_sensor_ori_offset.x(),
                body_odom_sensor_ori_offset.y(), body_odom_sensor_ori_offset.z());
        }
        //
        auto odom_sensor_uwb_pos_offset = body_uwb_offset - body_odom_sensor_pos_offset;
        printf("Odom to UWB offset:[%.3f, %.3f, %.3f].\n", odom_sensor_uwb_pos_offset[0],
            odom_sensor_uwb_pos_offset[1], odom_sensor_uwb_pos_offset[2]);

        return true;
    }
    //
    bool RaSam::setupInterfaces()
    {
		if(robot_config["sensors"]["range"].as<bool>())
        {
            std::string range_topic = robot_config["sensor_topics"]["range"].as<std::string>();
            printf("[%s]: Creating callback for Range data:[%s].\n", process_name.c_str(),
                    range_topic.c_str());
            subscribers["range"] = p_nh->subscribe<measurement_msgs::Range>(range_topic, 10, &RaSam::rangeCB, this,
            ros::TransportHints().tcpNoDelay());
        }
        //
		if(robot_config["sensors"]["vio"].as<bool>())
        {
            std::string vio_topic = robot_config["sensor_topics"]["vio"].as<std::string>();
            printf("[%s]: Creating callback for VIO data:[%s].\n", process_name.c_str(),
                    vio_topic.c_str());
            subscribers["vio"] = p_nh->subscribe<nav_msgs::Odometry>(vio_topic, 10, &RaSam::vioCB, this,
            ros::TransportHints().tcpNoDelay());
        }
        //
		if(robot_config["sensors"]["imu"].as<bool>())
        {
            std::string imu_topic = robot_config["sensor_topics"]["imu"].as<std::string>();
            printf("[%s]: Creating callback for IMU data:[%s].\n", process_name.c_str(),
                    imu_topic.c_str());
            subscribers["imu"] = p_nh->subscribe<sensor_msgs::Imu>(imu_topic, 200, &RaSam::imuCB, this,
            ros::TransportHints().tcpNoDelay());
        }
        //
        publishers["odom"] = p_nh->advertise<nav_msgs::Odometry>("ra_sam_odom", 10);
        //
        return true;
    }

    bool RaSam::isMeasRelative(const measurement::MeasurementBasePtr& meas)
    {
        return (meas->type == measurement::IMU || 
                meas->type == measurement::ODOMETRY);
    }
    // TODO: Should accomodate absolute Pose or Position measurements ?
    bool RaSam::isMeasAbsolute(const measurement::MeasurementBasePtr& meas)
    {
        return meas->type == measurement::RANGE;
    }

    void RaSam::imuCB(const sensor_msgs::Imu::ConstPtr& _msg)
    {
        if(verbose)
            printf("[%s]: IMU cb: seq:[%d] stamp:[%f].\n", process_name.c_str(), _msg->header.seq,
                _msg->header.stamp.toSec());
        if(imu_sample_factor > 1 && ((_msg->header.seq % imu_sample_factor) != 0))
            return;

        measurement::ImuPtr imu = std::make_shared<measurement::Imu>();
        measurement::fromROS(*_msg, *imu);
        // Rotate from IMU frame to body frame
        imu->angular_velocity = body_odom_sensor_ori_offset * imu->angular_velocity;
        imu->linear_acceleration = body_odom_sensor_ori_offset * imu->linear_acceleration;
        // printf("Ang:[%.3f, %.3f, %.3f] Acc:[%.3f, %.3f, %.3f].\n", imu->angular_velocity[0],
        //     imu->angular_velocity[0], imu->angular_velocity[0], imu->linear_acceleration[0],
        //     imu->linear_acceleration[1], imu->linear_acceleration[2]);
        //
        rel_data_queue.push(std::dynamic_pointer_cast<measurement::MeasurementBase>(imu));
    }

    void RaSam::vioCB(const nav_msgs::Odometry::ConstPtr& _msg)
    {
        if(!prev_vio_odom)
        {
            prev_vio_odom = std::make_shared<measurement::Odometry>();
            // This will copy header and covariances
            measurement::fromROS(*_msg, *(prev_vio_odom.get()));
            return;
        }
        // //
        measurement::Odometry delta_odom, curr_vio_odom;
        measurement::fromROS(*_msg, curr_vio_odom);
        measurement::fromROS(*_msg, delta_odom);
        // printf("Abs: pos:[%.3f, %.3f, %.3f] ori:[%.3f, %.3f, %.3f, w:%.3f].\n",
        //     curr_vio_odom.position[0], curr_vio_odom.position[1],
        //     curr_vio_odom.position[2], curr_vio_odom.orientation.x(),
        //     curr_vio_odom.orientation.y(), curr_vio_odom.orientation.z(),
        //     curr_vio_odom.orientation.w());
        Eigen::Quaterniond q_inv = prev_vio_odom->orientation.conjugate();
        delta_odom.position = q_inv * (curr_vio_odom.position - prev_vio_odom->position);
        delta_odom.orientation = q_inv * curr_vio_odom.orientation;
        delta_odom.orientation.normalize();
        // //
        rel_data_queue.push(std::make_shared<measurement::Odometry>(delta_odom));
        prev_vio_odom = std::make_shared<measurement::Odometry>(curr_vio_odom);
        // if(verbose)
            // printf("[%d]: VIO cb:[%f].\n", cnt++, delta_odom.header.stamp);
            // printf("Diff: pos:[%.3f, %.3f, %.3f] ori:[%.3f, %.3f, %.3f, w:%.3f].\n",
            // delta_odom.position[0], delta_odom.position[1],
            // delta_odom.position[2], delta_odom.orientation.x(),
            // delta_odom.orientation.y(), delta_odom.orientation.z(),
            // delta_odom.orientation.w());
        //
    }
    
    void RaSam::rangeCB(const measurement_msgs::Range::ConstPtr& _msg)
    {
        measurement::Range range;
        // parse message
        measurement::fromROS(*_msg, range);
        //
        // if(verbose)
        //     printf("[%s]: Range cb:[%f].\n", process_name.c_str(), range.header.stamp);
        // add mobile and anchor positions
        std::vector<double> m_pos = 
            robot_config["uwb"][_msg->mobile]["pos"].as<std::vector<double>>(); 
        std::vector<double> a_pos = 
            robot_config["uwb"][_msg->anchors[0]]["pos"].as<std::vector<double>>(); 
        range.anchor_pos.resize((int)_msg->anchors.size());
        range.covariance.resize((int)_msg->anchors.size());
        //
        range.mobile_pos = body_uwb_offset - body_odom_sensor_pos_offset;
        (range.anchor_pos[0] << a_pos[0], a_pos[1], a_pos[2]).finished();
        // copy covariance from config
        range.covariance[0] = robot_config["uwb"][_msg->anchors[0]]["cov"].as<double>();
        // printf("Adding range:[%s]<->[%s] pos:[%.3f, %.3f, %.3f]<->[%.3f, %.3f, %.3f]\n",
        //     range.anchors[0].c_str(), range.mobile.c_str(), range.anchor_pos[0][0], 
        //     range.anchor_pos[0][1], range.anchor_pos[0][2], range.mobile_pos[0],
        //     range.mobile_pos[1], range.mobile_pos[2]);
        measurement::MeasurementBasePtr range_ptr = 
            std::make_shared<measurement::Range>(range);
        abs_data_queue.push(range_ptr);
    }
    //
    State RaSam::getInitialState()
    {
        auto p_ = robot_config["initial_state"]["pos"].as<std::vector<double>>();
        auto q_ = robot_config["initial_state"]["ori"].as<std::vector<double>>();
        auto v_ = robot_config["initial_state"]["vel"].as<std::vector<double>>();
        auto np = robot_config["initial_state"]["pos_cov"].as<std::vector<double>>();
        auto nq = robot_config["initial_state"]["ori_cov"].as<std::vector<double>>();
        auto nv = robot_config["initial_state"]["vel_cov"].as<std::vector<double>>();
        //
        State init_state;
        init_state.stamp = 0.0;
        init_state.vertex_id = 0;
        init_state.nav_state = NavState(Pose3(
            Rot3(Eigen::Quaterniond(q_[3], q_[0], q_[1], q_[2])), // orientation
            Vector3(p_[0], p_[1], p_[2])), // position
            Vector3(v_[0], v_[1], v_[2])); // velocity
        //
        init_state.pose_cov.fill(0);
        init_state.lin_vel_cov.fill(0);
        for(int i = 0; i < 3; i++)
        {
            init_state.pose_cov(i,i) = np[i];
            init_state.pose_cov(i+3,i+3) = nq[i];
            init_state.lin_vel_cov(i,i) = nv[i];
        }
        
        printf("Initial state: pos:[%.3f, %.3f, %.3f] ori(wxyz):[%.3f, %.3f, %.3f, %.3f] "
            "vel:[%.3f, %.3f, %.3f] \n", init_state.nav_state.pose().x(),
            init_state.nav_state.pose().y(), init_state.nav_state.pose().z(),
            init_state.nav_state.pose().rotation().quaternion()[0],
            init_state.nav_state.pose().rotation().quaternion()[1],
            init_state.nav_state.pose().rotation().quaternion()[2],
            init_state.nav_state.pose().rotation().quaternion()[3],
            init_state.nav_state.velocity()[0],
            init_state.nav_state.velocity()[1],
            init_state.nav_state.velocity()[2]);
        std::cout << "Initial Pose covariance:\n" << init_state.pose_cov << std::endl; 
        std::cout << "Initial Twist covariance:\n" << init_state.lin_vel_cov << std::endl;
        //
        return init_state;
    }

    std::shared_ptr<PreintegrationType> RaSam::createImuPreintegrator()
    {
        printf("[%s] Loading IMU noise parameters.\n", process_name.c_str());
        // load prior noise
        auto acc = robot_config["imu"]["accel_cov"].as<std::vector<double>>();
        auto gyr = robot_config["imu"]["gyro_cov"].as<std::vector<double>>();
        auto ba = robot_config["imu"]["accel_bias_cov"].as<std::vector<double>>();
        auto bw = robot_config["imu"]["gyro_bias_cov"].as<std::vector<double>>();
        //
        printf("[%s]: Noise accel:[%.5f,%.5f,%.5f] gyro:[%.5f,%.5f,%.5f].\n", process_name.c_str(), acc[0],
            acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
        printf("[%s]: Noise accel bias:[%.5f,%.5f,%.5f] gyro bias:[%.5f,%.5f,%.5f].\n", process_name.c_str(),
            ba[0], ba[1], ba[2], bw[0], bw[1], bw[2]);
        // load imu noise parameters
        Eigen::DiagonalMatrix<double, 3> measured_acc_cov(acc[0], acc[1], acc[2]);
        Eigen::DiagonalMatrix<double, 3> measured_omega_cov(gyr[0], gyr[1], gyr[2]);
        Eigen::DiagonalMatrix<double, 3> bias_acc_cov(ba[0], ba[1], ba[2]);
        Eigen::DiagonalMatrix<double, 3> bias_omega_cov(bw[0], bw[1], bw[2]);
        Matrix33 integration_error_cov = I_3x3 * 1e-8;  // error committed in integrating position from velocities
        Matrix66 bias_acc_omega_int = I_6x6 * 1e-5;  // error in the bias used for preintegration
        // TODO: Make this a config parameter
        double acc_g = 9.8065;
        printf("[%s]: Acceleration due to garvity:[%f].\n", process_name.c_str(), acc_g);
        auto p = PreintegratedCombinedMeasurements::Params::MakeSharedU(acc_g);
        // PreintegrationBase params:
        p->accelerometerCovariance = measured_acc_cov;  // acc white noise in continuous
        p->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
        // should be using 2nd order integration
        // PreintegratedRotation params:
        p->gyroscopeCovariance = measured_omega_cov;  // gyro white noise in continuous
        // PreintegrationCombinedMeasurements params:
        p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
        p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
        p->biasAccOmegaInt = bias_acc_omega_int;
        // load prior bias
        acc.clear();
        gyr.clear();
        acc = robot_config["imu"]["accel_bias"].as<std::vector<double>>();
        gyr = robot_config["imu"]["gyro_bias"].as<std::vector<double>>();
        imuBias::ConstantBias prior_bias(Vector3(acc[0], acc[1], acc[2]), Vector3(gyr[0],
            gyr[1], gyr[2]));
        printf("[%s]:bias accel:[%.5f,%.5f,%.5f] gyro:[%.5f,%.5f,%.5f].\n", process_name.c_str(), acc[0],
            acc[1], acc[2], gyr[0], gyr[1], gyr[2]);

        std::shared_ptr<PreintegrationType> preintegrated = nullptr;
        preintegrated = std::make_shared<PreintegratedCombinedMeasurements>(p, prior_bias);
        assert(preintegrated);

        return preintegrated;
    }

    std::pair<imuBias::ConstantBias, noiseModel::Diagonal::shared_ptr> RaSam::imuBiasPrior()
    {
        auto ba = robot_config["imu"]["accel_bias_cov"].as<std::vector<double>>();
        auto bw = robot_config["imu"]["gyro_bias_cov"].as<std::vector<double>>();
        noiseModel::Diagonal::shared_ptr prior_bias_noise = noiseModel::Diagonal::Variances(
                (Vector(6) << ba[0],ba[1],ba[1], bw[0], bw[1], bw[2]).finished());

        auto acc = robot_config["imu"]["accel_bias"].as<std::vector<double>>();
        auto gyr = robot_config["imu"]["gyro_bias"].as<std::vector<double>>();
        //
        imuBias::ConstantBias prior_bias(
            Vector3(acc[0], acc[1], acc[2]), Vector3(gyr[0], gyr[1], gyr[2]));
        //
        return std::pair<imuBias::ConstantBias, noiseModel::Diagonal::shared_ptr>
            (prior_bias, prior_bias_noise);
    }

    Pose3 RaSam::odomToSAMPose(const measurement::Odometry& odom)
    {
        const Eigen::Quaterniond& q = odom.orientation;
        const Eigen::Vector3d& t = odom.position;
        return Pose3(Rot3(q.w(), q.x(), q.y(), q.z()),
            Vector3(t[0], t[1], t[2]));
    }

    measurement::Odometry RaSam::samPoseToOdom(const Pose3& pose)
    {
        measurement::Odometry odom;
        odom.orientation = pose.rotation().toQuaternion();
        odom.position = pose.translation();
        return odom;
    }

    geometry_msgs::TwistWithCovariance RaSam::getTwistMessage(const State& posterior_state)
    {
        geometry_msgs::TwistWithCovariance twist_msg;
        Matrix6 twist_cov;
        twist_cov = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
        //
        if(robot_config["sensors"]["imu"].as<bool>())
        {
            auto rel_meas = std::dynamic_pointer_cast<measurement::Imu>(latest_rel_meas);
            // Rt is the world to body (not IMU/VIO) rotation
            auto Rt = posterior_state.nav_state.pose().rotation().inverse().matrix();
            // Transform velocity into body frame as most control
            // algorithms expect this
            Vector3 vel_body_frame = Rt * posterior_state.nav_state.velocity();
            // The velocity covariance from estimator is the covariance of
            // velocity of IMU/VIO in world frame. Hence it needs to be
            // transformed into body frame.
            twist_cov.block<3,3>(0,0) = Rt * posterior_state.lin_vel_cov * Rt.transpose();
            twist_msg.twist.linear.x = vel_body_frame[0];
            twist_msg.twist.linear.y = vel_body_frame[1];
            twist_msg.twist.linear.z = vel_body_frame[2];
            twist_msg.twist.angular.x = rel_meas->angular_velocity[0];
            twist_msg.twist.angular.y = rel_meas->angular_velocity[1];
            twist_msg.twist.angular.z = rel_meas->angular_velocity[2];
        }
        //
        if(robot_config["sensors"]["vio"].as<bool>())
        {
            auto rel_meas = std::dynamic_pointer_cast<measurement::Odometry>(latest_rel_meas);
            twist_msg.twist.linear.x = rel_meas->linear[0];
            twist_msg.twist.linear.y = rel_meas->linear[1];
            twist_msg.twist.linear.z = rel_meas->linear[2];
            twist_msg.twist.angular.x = rel_meas->angular[0];
            twist_msg.twist.angular.y = rel_meas->angular[1];
            twist_msg.twist.angular.z = rel_meas->angular[2];
        }
        //
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
            (twist_msg.covariance.data(), 6, 6) = twist_cov;
        //
        return twist_msg;
    }

    void RaSam::publishDataToRos(const State& posterior_state)
    {
        const Pose3 pose = posterior_state.nav_state.pose();
        auto q = pose.rotation().toQuaternion();
        //
        if(robot_config["publish_odom"].as<bool>())
        {
            // When using VIO, it is expected the odometry is rotated to align with
            // body frame. With IMU, the measurements are rotated to align with body
            // frame. Hence we need to only compensate for position offset.
            nav_msgs::Odometry odom_msg;
            odom_msg.header.frame_id = robot_config["world_frame"].as<std::string>();
            odom_msg.child_frame_id = robot_config["robot_frame"].as<std::string>();
            odom_msg.header.stamp = ros::Time(posterior_state.stamp);
            odom_msg.pose.pose.position.x = pose.x() - body_odom_sensor_pos_offset[0];
            odom_msg.pose.pose.position.y = pose.y() - body_odom_sensor_pos_offset[1];
            odom_msg.pose.pose.position.z = pose.z() - body_odom_sensor_pos_offset[2];
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();
            // The pose covariance from estimator is the covariance of
            // pose of IMU/VIO in bpdu frame.
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
                (odom_msg.pose.covariance.data(), 6, 6) = posterior_state.pose_cov;
            // populate twist
            odom_msg.twist = getTwistMessage(posterior_state);
            publishers["odom"].publish(odom_msg);
        }
        //
        if(robot_config["broadcast_tf"].as<bool>())
        {
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time(posterior_state.stamp);
            transform.header.frame_id = robot_config["world_frame"].as<std::string>();
            transform.child_frame_id = robot_config["robot_frame"].as<std::string>();
            transform.transform.translation.x = pose.x();
            transform.transform.translation.y = pose.y();
            transform.transform.translation.z = pose.z();
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();
            tf_broadcaster.sendTransform(transform);
        }
    }
    // // TODO: This can be done more efficiently by 
    // // storing only the latest IMU
    // measurement::ImuPtr RaSam::getImu(double stamp)
    // {
    //     while(!imu_data_queue.empty())
    //     {
    //         auto meas = imu_data_queue.top();
    //         if(meas->header.stamp < stamp)
    //         {
    //             imu_data_queue.pop();
    //         }
    //         else
    //         {
    //             // TODO: Figure this out
    //             if(fabs(stamp - meas->header.stamp) < 0.001)
    //             {
    //                 imu_data_queue.pop();
    //                 return std::dynamic_pointer_cast<measurement::Imu>(meas);
    //             }
    //         }
    //     }
    //     return nullptr;
    // }
}
