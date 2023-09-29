/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include "ra_sam/ra_sam.h"

namespace sam
{
    RaSam::RaSam(const std::string _sam_config, const std::string _robot_config) :
    sam_config_file(_sam_config),
    robot_config_file(_robot_config)
    {
        p_nh = std::make_shared<ros::NodeHandle>("~");
        //
        process_name = "ra_sam";
        //
        if(!loadConfig())
        {
            printf("Failed to load SAM config:%s\n", sam_config_file.c_str());
            printf("Failed to load robot config:%s\n", robot_config_file.c_str());
        }
        else
        {
            loadUWBConfig();
            //
            loadSensorExtrinsics();
            // Set up callbacks for both online and offline mode
            setupInterfaces();
            // Initializations related to the estimator
            setupEstimator();
            //
            resetEstimator();
        }
    }

    RaSam::~RaSam()
    {
    
    }

    void RaSam::setupEstimator()
    {
        // //
        if(robot_config["sensors"]["vio"].as<bool>())
        {
            odom_integrator = std::make_shared<OdomIntegrator>();
            odom_integrator->reset();
        }
        //
        if(robot_config["sensors"]["imu"].as<bool>())
        {
            imu_integrator = createImuPreintegrator();
        }
        //
        double smoother_lag = sam_config["smoother_lag"].as<double>();
        printf("[%s]: Creating fixed lag smoother with lag:[%f].\n",
            process_name.c_str(), smoother_lag);

        smoother = std::make_unique<IncrementalFixedLagSmoother>(smoother_lag);
    }

    void RaSam::resetEstimator()
    {
        factor_graph.resize(0);
        prior_values.clear();
        posterior_values.clear();
        timestamp_key_map.clear();
        vertex_index.store(0);
        state_initialized = false;
        graph_updated = false;
        latest_abs_meas = std::make_shared<measurement::MeasurementBase>();
        latest_rel_meas = std::make_shared<measurement::MeasurementBase>();
        latest_abs_meas->stamp = -1;
        latest_rel_meas->stamp = -1;
        prev_meas = nullptr;
        abs_data_queue = measurement::MeasurementQueue();
        rel_data_queue = measurement::MeasurementQueue();
        imu_data_queue = measurement::MeasurementQueue();
    }

    void RaSam::setState(const State& state)
    {
        printf("[%s]: Setting state: vertex_idx:[%ld]. pos:[%.3f, %.3f, %.3f].\n",
            process_name.c_str(), vertex_index.load(), state.nav_state.pose().x(),
            state.nav_state.pose().y(), state.nav_state.pose().z());
        //
        vertex_index.store(0);
        //
        Vector3 init_vel = state.nav_state.velocity();
        Pose3 init_pose = state.nav_state.pose();
        // add initial vertex
        Symbol pose_vid('x', vertex_index.load());
        // TODO: Handle this better
        timestamp_key_map[pose_vid] = state.stamp;
        // add a prior factor on initial vertex
        noiseModel::Diagonal::shared_ptr init_pose_cov =
            noiseModel::Diagonal::Variances(state.pose_cov.diagonal());
        //
        prior_values.insert(pose_vid, init_pose);
        PriorFactor<Pose3> pose_prior_factor(pose_vid, init_pose, init_pose_cov);
        factor_graph.add(pose_prior_factor);
        // TODO: Clean this up a bit
        //
        if(robot_config["sensors"]["imu"].as<bool>())
        {
            Symbol vel_vid('v', vertex_index.load());
            Symbol imu_bias_vid('b', vertex_index.load());
            noiseModel::Diagonal::shared_ptr init_vel_cov = 
                noiseModel::Diagonal::Variances((Vector(3) << state.lin_vel_cov(0,0), 
                state.lin_vel_cov(0,0),  state.lin_vel_cov(2,2)).finished());
            PriorFactor<Vector3> vel_prior_factor(vel_vid, init_vel, init_vel_cov);
            prior_values.insert(vel_vid, init_vel);
            factor_graph.add(vel_prior_factor);

            auto imu_bias_prior = imuBiasPrior();
            PriorFactor<imuBias::ConstantBias> imu_bias_factor = PriorFactor<imuBias::ConstantBias>
                (imu_bias_vid, imu_bias_prior.first, imu_bias_prior.second);

            timestamp_key_map[vel_vid] = state.stamp;
            timestamp_key_map[imu_bias_vid] = state.stamp;
            prior_values.insert(imu_bias_vid, imu_bias_factor.prior());
            factor_graph.add(imu_bias_factor);
        }
        //
        vertex_index++;
        //
        state_initialized = true;
    }

    void RaSam::runEstimator()
    {
        // Note the first spin is called
        // after setting initial state
        setState(getInitialState());
        //
        // thread_map["anc_calib_thread"] = std::make_unique<std::thread>(&RaSam::runAnchorCalib, this);
        //
        // For increment smoothing and mapping, anchor params need to be added only once
        // and will be maintained in the graph until completion
        //
        while(ros::ok())
        {
            // Fetch pending data from all callbacks
            ros::spinOnce();
            //
            const ::time_point<::steady_clock> iter_begin = ::steady_clock::now(); 
            //
            solution_available = false;
            //
            if(!state_initialized)
                setState(last_posterior_state);
            //
            if(buildGraph())
            {
                bool should_solve = true;
                // Solve only if the graph was updated
                // with new/additional data
                should_solve &= (graph_updated);
                // Solve only when 'x' new vertices
                // have been added to the graph. This
                // avoids solving a potential large
                // graph after every small increment.
                should_solve &= (vertex_index % smoother_traj_len == 0);
                //
                if(should_solve)
                {
                    // TODO: Setup the parameter values for solver
                    if(solveGraph())
                    {
                        // printf("[%s]: Size of factor graph:[%ld].\n", process_name.c_str(),
                        //     smoother->getDelta().size());
                        solution_available = true;
                        graph_updated = false;
                        updatePosteriorState();
                        resetSmootherBuffers();
                    }
                }
            } // buildGraph
            //
            const ::time_point<::steady_clock> iter_end = ::steady_clock::now();
            // If running online, run filter at 400Hz
            auto iter_duration = ::duration_cast<::microseconds>
                (iter_end - iter_begin).count();
            //
            if(iter_duration < 1000)
                std::this_thread::sleep_for(::duration<double, std::micro>
                    (1000 - iter_duration));
            //
            if(iter_duration > 200000)
                ROS_INFO_THROTTLE(1.0, "[%s]: Failed to meet update rate. Updated took:[%.3f]s.\n",
                    process_name.c_str(), (double)iter_duration/1000000);
        }
    }

    void RaSam::resetSmootherBuffers()
    {
        // This is only applicable to smoother
        timestamp_key_map.clear();
        prior_values.clear();
        factor_graph.resize(0);
    }

    bool RaSam::buildGraph()
    {
        // Observability is hard without absolute measurements
        // start processing only measurements are available
		if(abs_data_queue.empty())
        {
            if(verbose)
                printf("[%s]: No absolute measurements.\n", process_name.c_str());
            return false;
        }
        // if there are no control inputs, no prediction can be done
        // so do nothing
        if(rel_data_queue.empty())
        {
            if(verbose)
                printf("[%s]: No relative measurements.\n", process_name.c_str());
            return false;
        }
        //
        double abs_meas_stamp = abs_data_queue.top()->stamp;
        double rel_meas_stamp = rel_data_queue.top()->stamp;
        //
        if(verbose)
        {
            printf("[%s]: Queues top: abs:[%.3f] rel:[%.3f].\n", process_name.c_str(),
                abs_meas_stamp, rel_meas_stamp);
        }
        //
        // At the outset(or after a reset), if an absolute measurement
        // is received before odometry, there aren't enough constraints
        // to determine the system, hence, reject the abs measurement.
        if(latest_rel_meas->stamp < 0)
        {
            if(abs_meas_stamp < rel_meas_stamp)
            {
                printf("WARNING: Initial setup. Dropping abs meas. abs stamp:[%f] rel stamp:[%f].\n",
                    abs_meas_stamp, rel_meas_stamp);
                abs_data_queue.pop();
                return false;
            }
            // Use the first message to "initialize odometry" with a very
            // small dt=0.0001. We want to do this after old (stale) absolute
            // measurements are removed
            else
            {
                latest_rel_meas = rel_data_queue.top();
                updateOdometry(latest_rel_meas, 0.0001);
                rel_data_queue.pop();
                return false;
            }
        }

        if(latest_abs_meas->stamp > 0)
        {
            if(abs_meas_stamp < latest_abs_meas->stamp)
            {
                if(verbose)
                {
                    printf("WARNING: Out-of-sequence abs. meas:stamp:[%f] curr stamp:[%f].\n",
                        abs_meas_stamp, latest_abs_meas->stamp);
                }
                abs_data_queue.pop();
                return false;
            }
            //
            if(abs_meas_stamp < latest_rel_meas->stamp)
            {
                if(verbose)
                {
                    printf("WARNING: Out-of-sequence rel. meas.:stamp:[%f] curr stamp:[%f].\n",
                        rel_meas_stamp, latest_rel_meas->stamp);
                }
                rel_data_queue.pop();
                return false;
            }
        }
        //
        while(!rel_data_queue.empty())
        {
            // fetch all the relevant control inputs up until the measurement time
            auto last_ctrl = rel_data_queue.top();
            // if the control input is ahead of absolute measurement (plus some tolerance)
            // then don't do predict step (dead reckoning)
            if(last_ctrl->stamp > abs_meas_stamp + 1e-8)
                break;
            //
            if(verbose)
            {
                printf("[%s]: Adding rel meas:[%.3f] till abs meas:[%.3f].\n", process_name.c_str(),
                    last_ctrl->stamp, abs_meas_stamp);
            }
            // it has been observed that consecutive IMU messages with same timestamps and similar
            // contents can be recieved. For now we drop them.
            double dt = last_ctrl->stamp - latest_rel_meas->stamp;
            //
            if(dt < 1e-4)
            {
                if(verbose)
                    printf("WARNING: Potential duplicate meas: stamp:[%f] last rel stamp:[%f].\n",
                        rel_meas_stamp, latest_rel_meas->stamp);
                rel_data_queue.pop();
                continue;
            }
            //
            updateOdometry(last_ctrl, dt);
            prev_meas = last_ctrl;
            latest_rel_meas = last_ctrl;
            rel_data_queue.pop();
        }
        // If there are no more odometry messages, there could be two cases:
        // 1. The absolute measurement is too far in the future. In this case
        //  wait for more odometry data to become available.  (if case)
        // 2. The absolute measurement is between two relative measurements.
        // In this case either use the latest or do interpolation. (else case)
        if(rel_data_queue.empty())
        {
            double meas_dt = fabs(abs_meas_stamp - latest_rel_meas->stamp);
            // TODO: Use sensor dt to do this check
            if(meas_dt > 1e-3)
            {
                if(verbose)
                    printf("[%s]: Abs meas far in future. Waiting for more odometry.\n",
                        process_name.c_str());
                return false;
            }
        }
        else
        {
            auto next_ctrl_stamp = rel_data_queue.top()->stamp;
            double ctrl_dt = fabs(next_ctrl_stamp - latest_rel_meas->stamp);
            double meas_dt = fabs(abs_meas_stamp - latest_rel_meas->stamp);
            if(meas_dt > ctrl_dt)
            {
                printf("[%s]: Inconsistent timeline of sensor data.\n", process_name.c_str());
            }
            // TODO: linear iterpolation for IMU and pose measurements
        }
        //
        // In two consecutive range/tdoa measurements are received,
        // the problem will be ill-conditioned and can result in
        // solver failing. Optionally drop such measurements
        if(sam_config["drop_consec_abs_meas"].as<bool>() &&
            prev_meas != nullptr)
        {
            if(isMeasAbsolute(prev_meas))
            {
                if(verbose)
                    printf("WARNING: Consecutive abs meas. Dropping.\n");
                //
                abs_data_queue.pop();
                return false;
            }
        }
        // If the measurement is absolute:
        // 1. First, add a between factor between previous vertex
        //    and current vertex. This also creates a new vertex
        addBetweenFactor();
        // 2. Then add the absolute measurement factor at the
        //    current vertex
        // What happens if we keep on rejecting measurements
        // and the estimate diverges ? Is this a concern in
        // smoothing mode.
        auto abs_meas = abs_data_queue.top();
        if(addMeasurementFactor(abs_meas))
        {
            // record correction time
            latest_abs_meas = abs_meas;
            prev_meas = abs_meas;
            // Book-keeping
            abs_data_queue.pop();
        }
        else
        {
            printf("[%s]: Dropping measurement:[%s] at [%f].\n", process_name.c_str(),
                abs_meas->getTypeAsString().c_str(), abs_meas->stamp);
            abs_data_queue.pop();
        }
        //
        graph_updated = true;
        //
        if(vertex_index == 1)
            handleInitialTimeJump();
        //
        vertex_index++;
        //
        if(verbose && graph_updated)
        {
            printf("Latest abs:[%f] latest rel:[%f].\n \n", 
                latest_abs_meas->stamp, latest_rel_meas->stamp);
            // printf("[%s]: Increment vertex index:[%d]\n",
            //     process_name.c_str(), vertex_index.load());
        }
        //
        return true;
    }

    bool RaSam::updateOdometry(const measurement::MeasurementBasePtr& meas, const double dt)
    {
        if(robot_config["sensors"]["vio"].as<bool>())
        {
            auto odom_meas = std::dynamic_pointer_cast<measurement::Odometry>(meas);
            odom_integrator->update(*(odom_meas.get()));
        }
        else if(robot_config["sensors"]["imu"].as<bool>())
        {
            auto imu_meas = std::dynamic_pointer_cast<measurement::Imu>(meas);
            // printf("[%s]: Latest rel stamp:[%d][%f] curr stamp:[%d][%f] dt:[%f] small:[%s].\n",
            //     process_name.c_str(), latest_rel_meas->header.seq, latest_rel_meas->stamp,  
            //     meas->header.seq, meas->stamp, dt, (fabs(dt) < 1e-4 ? "TRUE" : "FALSE") );
            // pre-integrate measurements
            imu_integrator->integrateMeasurement(imu_meas->linear_acceleration, 
                imu_meas->angular_velocity, dt);
            imu_data_queue.push(meas);
        }

        return true;
    }
    //
    bool RaSam::addBetweenFactor()
    {
        if(robot_config["sensors"]["vio"].as<bool>())
        {
            //
            Symbol prev_pose_key('x', vertex_index.load()-1);
            Symbol curr_pose_key('x', vertex_index.load());
            // get cumulative odometry
            auto cum_odom = odomToSAMPose(odom_integrator->get());
            Pose3 prev_state;
            // get prior for current vertex using dead reckoning
            if(smoother->valueExists(prev_pose_key))
                prev_state = smoother->calculateEstimate<Pose3>(prev_pose_key);
            else
                prev_state = prior_values.at<Pose3>(prev_pose_key);
            //
            auto prev_odom = samPoseToOdom(prev_state);
            const Pose3 prior = odomToSAMPose(odom_integrator->predict(prev_odom));
            // add prior for current vertex
            prior_values.insert(curr_pose_key, prior);
            timestamp_key_map[curr_pose_key] = latest_rel_meas->stamp;
            // TODO: Extract covariance from odom integrator
            auto cov = robot_config["vio"]["cov"].as<std::vector<double>>();
            noiseModel::Diagonal::shared_ptr prior_cov = noiseModel::Diagonal::Variances(
                (Vector(6) << cov[0], cov[1], cov[1], cov[3], cov[4], cov[5]).finished());
            //
            // Add a between-factor using odometry measurements
            BetweenFactor<Pose3> between_factor(prev_pose_key, curr_pose_key,
                cum_odom, prior_cov);
            factor_graph.add(between_factor);

            odom_integrator->reset();
        }
        else if(robot_config["sensors"]["imu"].as<bool>())
        {
            // previous node
            Key prev_pose_key = Symbol('x', vertex_index.load()-1);
            Key prev_vel_key = Symbol('v', vertex_index.load()-1);
            Key prev_bias_key = Symbol('b', vertex_index.load()-1);
            // current node
            Key curr_pose_key = Symbol('x', vertex_index.load());
            Key curr_vel_key = Symbol('v', vertex_index.load());
            Key curr_bias_key = Symbol('b', vertex_index.load());
            // TODO: Pointer ?
            NavState prev_state;
            imuBias::ConstantBias prev_imu_bias;
            // this implies there is inferred state available for previous vertex
            if(smoother->valueExists(prev_pose_key))
            {
                prev_state = NavState(smoother->calculateEstimate<Pose3>(prev_pose_key),
                    smoother->calculateEstimate<Vector3>(prev_vel_key));
                prev_imu_bias = smoother->calculateEstimate<imuBias::ConstantBias>(prev_bias_key);
            }
            else
            {
                prev_state = NavState(prior_values.at<Pose3>(prev_pose_key),
                        prior_values.at<Vector3>(prev_vel_key));
                prev_imu_bias = prior_values.at<imuBias::ConstantBias>(prev_bias_key);
            }
            //
            NavState prop_state = imu_integrator->predict(prev_state, prev_imu_bias);
            prior_values.insert(curr_pose_key, prop_state.pose());
            prior_values.insert(curr_vel_key, prop_state.v());
            prior_values.insert(curr_bias_key, prev_imu_bias);
            timestamp_key_map[curr_pose_key] = latest_rel_meas->stamp;
            timestamp_key_map[curr_vel_key] = latest_rel_meas->stamp;
            timestamp_key_map[curr_bias_key] = latest_rel_meas->stamp;
            //accumulate all imu measurements into a single delta-pose factor
            auto preint_imu_combined =
                dynamic_cast<const PreintegratedCombinedMeasurements&>(*imu_integrator);
            CombinedImuFactor imu_factor(prev_pose_key, prev_vel_key,
                    curr_pose_key, curr_vel_key, prev_bias_key,
                    curr_bias_key, preint_imu_combined);
            //
            factor_graph.add(imu_factor);
            //
            imu_integrator->resetIntegrationAndSetBias(prev_imu_bias);
        }
        //
        return true;
    }

    void RaSam::handleInitialTimeJump()
    {
        // This is pretty much a hack. Figure out a better way 
        // to handle time jumps
        Symbol prev_pose_key('x', vertex_index.load()-1);
        Symbol curr_pose_key('x', vertex_index.load());
        auto prev_timestamp = timestamp_key_map[prev_pose_key];
        auto curr_timestamp = timestamp_key_map[curr_pose_key];
        double smoother_lag = sam_config["smoother_lag"].as<double>();
        printf("prev: [%f] curr:[%f].\n", prev_timestamp, curr_timestamp);

        // TODO: Check validity of update rates
        if(curr_timestamp - prev_timestamp > smoother_lag/2.0)
        {
            printf("[%s]: Initial time jump. Resetting.\n", process_name.c_str());
            if(robot_config["sensors"]["vio"].as<bool>())
            {
                double delta = 1.0/(robot_config["vio"]["update_rate"].as<double>());
                clamp<double>(delta, 1e-3, smoother_lag/2.0); 
                timestamp_key_map[prev_pose_key] = curr_timestamp - delta;
            }
            if(robot_config["sensors"]["imu"].as<bool>())
            {
                double delta = 1.0/(robot_config["imu"]["update_rate"].as<double>());
                clamp<double>(delta, 1e-3, smoother_lag/2.0); 
                timestamp_key_map[prev_pose_key] = curr_timestamp - delta;
                timestamp_key_map[Symbol('v', vertex_index.load()-1)] = curr_timestamp - delta;
                timestamp_key_map[Symbol('b', vertex_index.load()-1)] = curr_timestamp - delta;
            }
        }
        printf("Updated: prev: [%f] curr:[%f].\n", timestamp_key_map[prev_pose_key],
            timestamp_key_map[curr_pose_key]);
    }

    bool RaSam::addMeasurementFactor(const measurement::MeasurementBasePtr meas)
    {
        switch(meas->type)
        {
            case measurement::RANGE:
            {
                measurement::RangePtr range_meas = std::dynamic_pointer_cast<measurement::Range>(meas);
                //
                std::string anchor_name = range_meas->anchors[0];
                //
                updateAnchorBiasStateOnGraph(range_meas);
                // this is a shared pointer to anchor noise parameters
                auto anc_noise_params = anchor_data[anchor_name].noise_params;
                // Vanilla noise model
                noiseModel::Diagonal::shared_ptr meas_noise = 
                    noiseModel::Diagonal::Variances(Vector1(anc_noise_params->cov(0,0)));
                //
                auto robust_cost = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(3.0), meas_noise);
                //
                Key curr_vertex_id = Symbol('x', vertex_index.load());
                //
                Key anc_bias_id = anchor_vertex_key[range_meas->anchors[0]];
                // add Range factor at current vertex
                auto range_factor = boost::make_shared<RangeFactorNoiseParams>
                    (curr_vertex_id, anc_bias_id, robust_cost, range_meas, anc_noise_params);
                //
                // Will the prior pose suffice?
                Pose3 curr_pose = prior_values.at<Pose3>(curr_vertex_id);
                // Simple outlier rejection based on euclidean distance threshold
                Eigen::VectorXd residual = range_factor->evaluateError(curr_pose, anc_noise_params->mu);
                //
                if(isMeasurementValid(curr_pose, range_meas, residual))
                    factor_graph.add(range_factor);
                // Store range factors for calibration of anchor noise parameters
                range_factors.push_back(range_factor);
                //
                return true;
            }
            default:
            {
                printf("[%s]: Unkown measurement:[%s].\n", process_name.c_str(),
                    meas->getTypeAsString().c_str());
                return false;
            }
        }
    }

    bool RaSam::solveGraph()
    {
        try
        {
            smoother->update(factor_graph, prior_values, timestamp_key_map);
            // Clear containers for the next iteration
            // Optinal for higher accuracy
            for(int it = 0; it < sam_config["smoother_internal_iterations"].as<int>(); it++)
                smoother->update();
        }
        catch(const std::exception& e)
        {
            printf("[%s]: Exception occured in solving graph.[%s].\n",
                process_name.c_str(), e.what());
            setupEstimator();
            resetEstimator();
            return false;
        }

        return true;
    }
    //
    bool RaSam::isMeasurementValid(const Pose3& curr_pose, const measurement::MeasurementBasePtr& meas,
        const Eigen::VectorXd& residual)
    {
        switch(meas->type)
        {
            case measurement::RANGE :
            {
                auto range_meas = std::dynamic_pointer_cast<measurement::Range>(meas);
                // printf("[%s]: Adding range meas:[%s]: Anchor Key:[%ld] Pose key:[%ld].\n",
                //     process_name.c_str(), anchor_name.c_str(), anchor_vertex_key[anchor_name], 
                //     curr_vertex_id);
                // // TODO: Do chi sq check as well.
                if(sam_config["uwb"]["range_res_thr"])
                {
                    if(fabs(residual[0]) > sam_config["uwb"]["range_res_thr"].as<double>())
                    {
                        printf("[%s]: Range outlier:[%s]<->[%s]:[%.3f](m) res:[%.3f] thr:[%.3f].\n",
                            process_name.c_str(), range_meas->anchors[0].c_str(), range_meas->mobile.c_str(),
                            range_meas->data[0], residual[0], sam_config["uwb"]["range_res_thr"].as<double>());
                        return false;
                    }
                }
                //
                return true;
            }
            default:
            {
                printf("[%s]: Measurement valid check not implemented for:[%s].\n",
                    process_name.c_str(), meas->getTypeAsString().c_str());
                return true;
            }
        }
    }

    void RaSam::updatePosteriorState()
    {
        // At the initial state, the traj is too short to get a smooth
        // estimate
        if(vertex_index < smoother_traj_len)
            return;
        try
        {
            // look back into the past to get a smoothed estimate
            const int idx = vertex_index - smoother_traj_len;
            Key key(Symbol('x', idx));
            //
            State posterior_state;
            posterior_state.stamp = timestamp_key_map[key];
            // TODO: Check before access
            if(robot_config["sensors"]["imu"].as<bool>())
            {
                // If using smoother
                auto est_pose = smoother->calculateEstimate<Pose3>(Symbol('x', idx));
                auto est_vel = smoother->calculateEstimate<Vector3>(Symbol('v', idx));
                auto est_bias = smoother->calculateEstimate<imuBias::ConstantBias>(
                    Symbol('b', idx));
                posterior_state.nav_state = NavState(est_pose, est_vel);
                posterior_state.imu_bias = est_bias;
                posterior_state.vertex_id = idx;
                posterior_state.pose_cov = smoother->marginalCovariance(Symbol('x', idx));
                posterior_state.lin_vel_cov = smoother->marginalCovariance(Symbol('v', idx));
                // posterior_state.ang_vel_cov =
            }
            //
            if(robot_config["sensors"]["vio"].as<bool>())
            {
                auto est_pose = smoother->calculateEstimate<Pose3>(Symbol('x', idx));
                posterior_state.pose_cov = smoother->marginalCovariance(Symbol('x', idx));
                auto odom_msg = std::dynamic_pointer_cast<measurement::Odometry>(latest_rel_meas);
                posterior_state.nav_state = NavState(est_pose, odom_msg->linear);
            }

            posterior_values.insert(Symbol('x', idx), posterior_state.nav_state.pose());
            posterior_history.push_back(posterior_state);
            // printf("Timestamps: latest rel meas:[%f], latest abs meas:[%f] posterior:[%f].\n",
            //     latest_rel_meas->stamp, latest_abs_meas->stamp, posterior_state.stamp);
            // TODO: Move to different method
            publishDataToRos(posterior_state);
        }
        catch(const std::exception& e)
        {
            std::cerr <<  "\033[1;31m" << e.what()  << "\033[0m\n";
        }
        //
        try
        {
            if(est_anchor_bias)
            {
                for(auto anchor_key : anchor_vertex_key)
                {
                    if(smoother->valueExists(anchor_key.second))
                    {
                        Vector1 bias = smoother->calculateEstimate<Vector1>(anchor_key.second);
                        Vector1 bias_cov = smoother->marginalCovariance(anchor_key.second);
                        anchor_data[anchor_key.first].noise_params->mu[0] = bias[0];
                        anchor_data[anchor_key.first].noise_params->mu_cov[0] = bias_cov[0];
                        if((vertex_index.load() % 100) == 0)
                            printf("[%s]: bias:[%.3f]:\n", anchor_key.first.c_str(), bias[0]);
                    }
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void RaSam::updateAnchorBiasStateOnGraph(const measurement::RangePtr range_meas)
    {
        std::string anchor_name = range_meas->anchors[0];
        Key anc_key = anchor_vertex_key[anchor_name];
        bool add_to_graph = false;

        add_to_graph = !smoother->valueExists(anc_key) && !prior_values.exists(anc_key);
        timestamp_key_map[anc_key] = range_meas->stamp;

        if(add_to_graph)
        {
            // We add prior factor for each bias variable to provide some
            // regularization
            const auto noise_params = anchor_data[anchor_name].noise_params;
            // anchor data holds the last best known value. We use it to
            // initialize the variable value. 
            prior_values.insert(anc_key, noise_params->mu);
            // For the prior factor we use the initial values
            // insert a prior factor
            double bias_cov = 0.01;
            if(sam_config["uwb"]["bias_cov"])
                bias_cov = sam_config["uwb"]["bias_cov"].as<double>();

            noiseModel::Diagonal::shared_ptr init_cov =
                noiseModel::Diagonal::Variances(Vector1(bias_cov));
            //
            Vector1 prior_mean(robot_config["uwb"][anchor_name]["bias"].as<double>());
            PriorFactor<Vector1> bias_prior_factor(anc_key, prior_mean, init_cov);
            factor_graph.add(bias_prior_factor);
            //
            printf("[%s]: Adding anchor:[%s] bias:[%.3f] cov:[%.3f].\n", process_name.c_str(),
                anchor_name.c_str(), prior_mean[0], bias_cov);
        }
    }
}

