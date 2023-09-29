/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <ra_sam/data_types.h>
#include <Eigen/Dense>

namespace sam
{
    //
    class OdomIntegrator
    {
        public:
        //
        OdomIntegrator()
        {
            cum_odom = measurement::Odometry::Identity();
            covariance = Eigen::Matrix<double, 6, 6>::Identity();
        }
        //
        ~OdomIntegrator()
        {

        }
        //
        void reset()
        {
            cum_odom = measurement::Odometry::Identity();
            covariance = Eigen::Matrix<double, 6, 6>::Identity();
        }
        //
        void update(const measurement::Odometry& meas)
        {
            // accumulate delta positions
            cum_odom.position = cum_odom.position + cum_odom.orientation * meas.position;
            cum_odom.orientation = cum_odom.orientation * meas.orientation;
            // TODO: propogate covariance
        }
        /*
         * @brief calculates the final pose with 
         * accumulated odom given a starting pose
         */
        const measurement::Odometry predict(const measurement::Odometry start_odom)
        {
            measurement::Odometry final_odom;
            // TODO: Optionally predict covariance
            final_odom.position = start_odom.position + 
                start_odom.orientation * cum_odom.position;
            final_odom.orientation = start_odom.orientation * 
                cum_odom.orientation;
            //
            return final_odom;
        }
        //
        const measurement::Odometry& get()
        {
            return cum_odom;
        }

        private:
        //
        measurement::Odometry cum_odom;
        Eigen::Matrix<double, 6, 6, Eigen::DontAlign> covariance;
    };

    typedef std::shared_ptr<OdomIntegrator> OdomIntegratorPtr;
}
