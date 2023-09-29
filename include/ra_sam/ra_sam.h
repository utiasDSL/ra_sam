/* ----------------------------------------------------------------------------

 * Range-aided smoothing and mapping 
 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <ra_sam/sam_defs.h>

using namespace gtsam;
using namespace std::chrono;

namespace sam
{
    class RaSam
    {
        public:
        //
        RaSam(const std::string _sam_config_file,
            const std::string _robot_config_file);
        //
        ~RaSam();
        /* * @brief */
        void runEstimator();
        //
        private:
        std::string sam_config_file;
        std::string robot_config_file;
        std::string process_name;
        //
        YAML::Node sam_config;
        YAML::Node robot_config;
        // 
        std::shared_ptr<ros::NodeHandle> p_nh;
        std::map<std::string, ros::Subscriber> subscribers;
        std::map<std::string, ros::Publisher> publishers;
        std::map<std::string, ros::Timer> timers;
        tf2_ros::TransformBroadcaster tf_broadcaster;
        //
        bool verbose;
        bool state_initialized;
        bool solution_available;
        bool graph_updated;
        bool est_anchor_bias;
        //
        uint smoother_traj_len;
        std::atomic<uint64_t> vertex_index;
        // sensor data
        measurement::MeasurementQueue abs_data_queue;
        measurement::MeasurementQueue rel_data_queue;
        measurement::MeasurementQueue imu_data_queue;
        measurement::MeasurementBasePtr prev_meas;
        // offset between VIO/IMU input and body center in body center
        Eigen::Vector3d body_odom_sensor_pos_offset;
        Eigen::Quaterniond body_odom_sensor_ori_offset;
        // offset between UWB input and body center in body center
        Point3 body_uwb_offset;
        //
        measurement::MeasurementBasePtr latest_abs_meas;
        measurement::MeasurementBasePtr latest_rel_meas;
        //
        uint imu_sample_factor;
        NonlinearFactorGraph factor_graph;
        std::unique_ptr<IncrementalFixedLagSmoother> smoother;
        FixedLagSmoother::KeyTimestampMap timestamp_key_map;
        //
        Values prior_values;
        Values posterior_values;
        State last_posterior_state;
        std::vector<State> posterior_history;
        //
        OdomIntegratorPtr odom_integrator = nullptr;
        std::shared_ptr<PreintegrationType> imu_integrator = nullptr;
        //
        Eigen::Matrix<double, 6, 6> def_pose_cov;
        measurement::OdometryPtr prev_vio_odom;
        //
        bool loadConfig();
        bool loadSensorExtrinsics();
        bool loadUWBConfig();
        bool setupInterfaces();
        void imuCB(const sensor_msgs::Imu::ConstPtr&);
        void vioCB(const nav_msgs::Odometry::ConstPtr&);
        void rangeCB(const measurement_msgs::Range::ConstPtr&);
        bool isMeasAbsolute(const measurement::MeasurementBasePtr& meas);
        bool isMeasRelative(const measurement::MeasurementBasePtr& meas);
        void setupEstimator();
        void resetEstimator();
        void resetSmootherBuffers();
        //
        State getInitialState();
        std::shared_ptr<PreintegrationType> createImuPreintegrator();
        std::pair<imuBias::ConstantBias, noiseModel::Diagonal::shared_ptr> imuBiasPrior();

        void setState(const State&);
        bool buildGraph();
        void handleInitialTimeJump();
        bool solveGraph();
        bool doBatchUpdate();
        void updatePosteriorState();
        bool updateOdometry(const measurement::MeasurementBasePtr& meas, const double);
        bool addBetweenFactor();
        bool addMeasurementFactor(const measurement::MeasurementBasePtr meas);
        bool isMeasurementValid(const Pose3& curr_pose, const measurement::MeasurementBasePtr& meas,
            const Eigen::VectorXd& residual);
        Pose3 odomToSAMPose(const measurement::Odometry&);
        measurement::Odometry samPoseToOdom(const Pose3&);
        geometry_msgs::TwistWithCovariance getTwistMessage(const State& );
        void publishDataToRos(const State& state);
        //
        std::vector<boost::shared_ptr<RangeFactorNoiseParams>> range_factors;
        std::map<std::string, AnchorInfo> anchor_data;
        std::map<std::string, Key> anchor_vertex_key;
        void updateAnchorBiasStateOnGraph(const measurement::RangePtr);
    };
}
