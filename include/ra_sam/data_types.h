
/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>


namespace measurement
{
	enum TYPE
	{
		IMU=0,
		ODOMETRY = 1,
		RANGE=2,
		UNKNOWN = 3,
	};
    //
    class MeasurementBase
    {
        public:
        MeasurementBase()
        {
            m_distance = -1;
        }

        ~MeasurementBase()
        {

        }
        // header
        double stamp;
        std::string frameId;
        // status of measurement
        int status;
        // mahalanobis distance
        float m_distance;
        //
        uint8_t type;

        // We want earlier times to have greater priority
        bool operator()(const std::shared_ptr<MeasurementBase> &a, const std::shared_ptr<MeasurementBase> &b)
        {
            return (*this)(*(a.get()), *(b.get()));
        }

        bool operator()(const MeasurementBase &a, const MeasurementBase &b)
        {
            return a.stamp > b.stamp;
        }

		virtual std::string getTypeAsString()
		{
			return "UNKNOWN";
		}

    };

    typedef std::shared_ptr<MeasurementBase> MeasurementBasePtr;
    typedef std::shared_ptr<const MeasurementBase> MeasurementBaseConstPtr;
    // priority queue for holding measurments
    typedef std::priority_queue<MeasurementBasePtr, 
        std::vector<MeasurementBasePtr>, 
        MeasurementBase> MeasurementQueue;	
    // deque for holding processed measurements
    typedef std::deque<MeasurementBasePtr> MeasurementDequeue;

    class Range : public MeasurementBase
    {
        public:
        Range()
        {
            data.clear();
            anchors.clear();
            mobile_pos.fill(0);
            type = RANGE;
        }

        ~Range()
        {

        }

        std::string mobile;
        Eigen::Vector3d mobile_pos;
        std::vector<std::string> anchors;
        std::vector<Eigen::Vector3d> anchor_pos;
        std::vector<double> data;
        std::vector<double> covariance;

		std::string getTypeAsString() override
		{
			return "RANGE";
		}

    };
    //
	static void fromROS(const measurement_msgs::Range& in, measurement::Range& out)
	{
		out.stamp = in.header.stamp.toSec();
		out.frameId = in.header.frame_id;
		out.mobile = in.mobile;

		for(int i = 0; i < in.anchors.size(); i++)
		{
			out.anchors.push_back(in.anchors.at(i));
			out.data.push_back(in.data.at(i));
			out.covariance.push_back(in.covariance.at(i));
		}
	}
    //
    typedef std::shared_ptr<Range> RangePtr;
    typedef std::shared_ptr<const Range> RangeConstPtr;
    //
    class Imu : public MeasurementBase
    {
        public:
        //
        Imu() : MeasurementBase()	
        {
            type = IMU;
        }

        ~Imu()
        {

        }
        // linear acceleration data
        Eigen::Vector3d linear_acceleration;
        // angular velocity data
        Eigen::Vector3d angular_velocity;
        // covariance for linear acceleration
        Eigen::Matrix3d accel_covariance;
        // covariance for angular velocity
        Eigen::Matrix3d gyro_covariance;
        //
		std::string getTypeAsString() override
		{
			return "IMU";
		}

    };
    static void fromROS(const sensor_msgs::Imu& in, measurement::Imu& out)
	{
		out.stamp = in.header.stamp.toSec();
		out.frameId = in.header.frame_id;
        out.linear_acceleration[0] = in.linear_acceleration.x;
        out.linear_acceleration[1] = in.linear_acceleration.y;
        out.linear_acceleration[2] = in.linear_acceleration.z;
        out.angular_velocity[0] = in.angular_velocity.x;
        out.angular_velocity[1] = in.angular_velocity.y;
        out.angular_velocity[2] = in.angular_velocity.z;
	}
    //
    typedef std::shared_ptr<Imu> ImuPtr;
    typedef std::shared_ptr<const Imu> ImuConstPtr; 
    //
	class Odometry : public MeasurementBase
	{
	public:
		Odometry() : MeasurementBase()
		{
			type = ODOMETRY;
		}

		~Odometry()
		{

		}

		std::string child_frame_id = "";
		Eigen::Vector3d linear;
        Eigen::Vector3d angular;
		Eigen::Vector3d position;
        Eigen::Quaternion<double, Eigen::DontAlign> orientation;
        Eigen::Matrix<double, 6, 6, Eigen::DontAlign> pose_covariance;
        Eigen::Matrix<double, 6, 6, Eigen::DontAlign> twist_covariance;
		std::string getTypeAsString() override
		{
			return "ODOMETRY";
		}
		//
		static Odometry Identity()
		{
			Odometry odom;
			odom.position.fill(0);
			odom.orientation.setIdentity();
			odom.linear.fill(0);
			odom.angular.fill(0);
			return odom;
		}
		//
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	};

	typedef std::shared_ptr<Odometry> OdometryPtr;
	typedef std::shared_ptr<const Odometry> OdometryConstPtr;

	static void fromROS(const nav_msgs::Odometry& in, measurement::Odometry& out)
	{
		out.child_frame_id = in.child_frame_id;
		out.stamp = in.header.stamp.toSec();
		out.frameId = in.header.frame_id;
        out.position[0] = in.pose.pose.position.x;
        out.position[1] = in.pose.pose.position.y;
        out.position[2] = in.pose.pose.position.z;
		out.orientation.x() = in.pose.pose.orientation.x;
		out.orientation.y() = in.pose.pose.orientation.y;
		out.orientation.z() = in.pose.pose.orientation.z;
		out.orientation.w() = in.pose.pose.orientation.w;

		out.pose_covariance = Eigen::Map<const Eigen::Matrix<double, 6, 6,
			Eigen::RowMajor>>(&(in.pose.covariance[0]), 6, 6);
        out.linear[0] = in.twist.twist.linear.x;
        out.linear[1] = in.twist.twist.linear.y;
        out.linear[2] = in.twist.twist.linear.z;
        out.angular[0] = in.twist.twist.angular.x;
        out.angular[1] = in.twist.twist.angular.y;
        out.angular[2] = in.twist.twist.angular.z;
		out.twist_covariance = Eigen::Map<const Eigen::Matrix<double, 6, 6,
			Eigen::RowMajor>>(&(in.twist.covariance[0]), 6, 6);
	}
}
