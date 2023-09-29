/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <ra_sam/noise_params.h>
#include <ra_sam/data_types.h>

namespace sam
{
    using namespace gtsam;
    class RangeFactorNoiseParams : public NoiseModelFactor2<Pose3, Vector1>
    {
        public:
        RangeFactorNoiseParams(Key j, Key bias_key, const SharedNoiseModel& model,
            measurement::RangePtr _meas, NoiseParamsGaussian1DPtr _params) :
            key(j), NoiseModelFactor2<Pose3, Vector1>(model, j, bias_key),
            meas(_meas), noise_params(_params) 
        {}

        void print(const std::string& s,
            const KeyFormatter& keyFormatter) const {
            std::cout << s << "RangeFactor: pose:" << keyFormatter(this->key) << "\n";
            std::cout << "\t" << meas->anchors[0] << "<->" << meas->mobile << "\n";
            std::cout << "\t mobile pos:" << meas->mobile_pos[0] << ",";
            std::cout << meas->mobile_pos[1] << "," << meas->mobile_pos[2] << "\n";
            std::cout << "\t anchor pos:" << meas->anchor_pos[0][0] << ",";
            std::cout << ","<< meas->anchor_pos[0][1] << "," << meas->anchor_pos[0][2] << "\n";
            std::cout << "\t range:" << meas->data[0] << "\n";
            std::cout << "\t noise params: mean:" << noise_params->mu << ", cov" << noise_params->cov << std::endl;  
            this->noiseModel_->print("  noise model: ");
        }

        Vector evaluateError(const Pose3& p, const Vector1& bias,
            boost::optional<Matrix&> H1 = boost::none,
            boost::optional<Matrix&> H2 = boost::none) const
        {
            using namespace gtsam;
            Matrix3 R = p.rotation().matrix();
            Point3 l = meas->mobile_pos;
            Point3 q = p * l - meas->anchor_pos[0];
            double pred_range = q.norm();
            if(H1)
            {
                Matrix13 D_h_q;
                D_h_q << q[0]/pred_range, q[1]/pred_range, q[2]/pred_range;
                Matrix36 D_q_local;
                D_q_local << -R * skewSymmetric(l), R;
                
                (*H1) =  D_h_q * D_q_local;
            }
            if(H2)
            {
                Matrix11 D_h_b;
                D_h_b << 1.0;
                (*H2) = D_h_b;
            }

            return Vector1(pred_range + bias[0] - meas->data[0]);
        }

        const Key getKey()
        {
            return key;
        }
        //
        const measurement::RangePtr getMeas()
        {
            return meas;
        }

        private:
        // measurement
        measurement::RangePtr meas;
        //
        NoiseParamsGaussian1DPtr noise_params;
        // key
        Key key;
    };
}