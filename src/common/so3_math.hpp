#pragma once

#include <boost/math/tools/precision.hpp>
#include <cassert>
#include <cmath>
#include <utility>

#include "common/eigen_types.h"

namespace lightning::math {

/**
 * Calculate cosine and sinc of sqrt(x2).
 * @param x2 the squared angle must be non-negative
 * @return a pair containing cos and sinc of sqrt(x2)
 */
template <class scalar>
inline std::pair<scalar, scalar> cos_sinc_sqrt(const scalar& x2) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    static scalar const taylor_0_bound = boost::math::tools::epsilon<scalar>();
    static scalar const taylor_2_bound = sqrt(taylor_0_bound);
    static scalar const taylor_n_bound = sqrt(taylor_2_bound);

    assert(x2 >= 0 && "argument must be non-negative");

    // FIXME check if bigger bounds are possible
    if (x2 >= taylor_n_bound) {
        // slow fall-back solution
        scalar x = sqrt(x2);
        return std::make_pair(cos(x), sin(x) / x);  // x is greater than 0.
    }

    // FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more stable, theoretically cos and sinc can
    // be calculated in parallel using SSE2 mulpd/addpd)
    // TODO Find optimal coefficients using Remez algorithm
    static scalar const inv[] = {1 / 3., 1 / 4., 1 / 5., 1 / 6., 1 / 7., 1 / 8., 1 / 9.};
    scalar cosi = 1., sinc = 1;
    scalar term = -1 / 2. * x2;
    for (int i = 0; i < 3; ++i) {
        cosi += term;
        term *= inv[2 * i];
        sinc += term;
        term *= -inv[2 * i + 1] * x2;
    }

    return std::make_pair(cosi, sinc);
}

inline SO3 exp(const Vec3d& vec, const double& scale = 1) {
    double norm2 = vec.squaredNorm();
    std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
    double mult = cos_sinc.second * scale;
    Vec3d result = mult * vec;
    return SO3(Quatd(cos_sinc.first, result[0], result[1], result[2]));
}

/**
 * SO3 Jl()/JacobianL()
 * @param v
 * @return
 */
inline Eigen::Matrix<double, 3, 3> A_matrix(const Vec3d& v) {
    Eigen::Matrix<double, 3, 3> res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squaredNorm);
    if (norm < 1e-5) {
        res = Eigen::Matrix<double, 3, 3>::Identity();
    } else {
        res = Eigen::Matrix<double, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * SO3::hat(v) +
              (1 - std::sin(norm) / norm) / squaredNorm * SO3::hat(v) * SO3::hat(v);
    }
    return res;
}

}  // namespace lightning::math
