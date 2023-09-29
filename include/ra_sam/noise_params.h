/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>

namespace sam
{
    template<typename T, int N>
    class NoiseParamsBase
    {
        public:
        //
        NoiseParamsBase()
        {
            dim = N;
        }
        //
        inline uint8_t getDim()
        {
            return dim;
        }
        //
        private:
        //
        uint8_t dim = 0;

    };
    
    template<typename T, int N>
    class NoiseParamsGaussian : public NoiseParamsBase<T, N>
    {
        public:
        //
        NoiseParamsGaussian()
        {
            mu.resize(N, 1);
            mu.fill(0.);
            //
            mu_cov.resize(N, N);
            mu_cov.setIdentity();
            //
            cov.resize(N, N);
            cov.setIdentity();
        }
        // mean
        Eigen::Matrix<T, N, 1> mu;
        // covariance of estimation mean
        Eigen::Matrix<T, N, N> mu_cov;
        // covariance
        Eigen::Matrix<T, N, N> cov;
        //        
        private:
    };

    //  
    template<typename T, int N>
    using NoiseParamsPtr = std::shared_ptr<NoiseParamsBase<T, N>>;
    //
    template<typename T, int N>
    using NoiseParamsGaussianPtr = std::shared_ptr<NoiseParamsGaussian<T, N>>;
    //
    using NoiseParamsGaussian1D = NoiseParamsGaussian<double, 1>;
    //
    using NoiseParamsGaussian1DPtr = std::shared_ptr<NoiseParamsGaussian1D>;
}
