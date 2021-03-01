#ifndef OB_CONFIG_HPP
#define OB_CONFIG_HPP

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <memory>
#include <iostream>



namespace OB
{
    // We could also use 16 bits to save space?
    using FeatureId = uint32_t;

    using Real = double;
    using Point = Eigen::Matrix<Real, 3, 1>;
    using Vector = Point;
    using Matrix3 = Eigen::Matrix<Real,3,3>;
    using DiagonalMatrix3 = Eigen::DiagonalMatrix<Real,3>;
    using Plane = Eigen::Hyperplane<Real,3>;

    // Affine looks like slow...
    constexpr enum Eigen::TransformTraits Isometry = Eigen::Isometry;
    //constexpr enum Eigen::TransformTraits Isometry = Eigen::Affine;

    // can't get it to work with CompactAffine
    constexpr enum Eigen::TransformTraits DefaultTransformTrait = Eigen::Affine;
    using Transform = Eigen::Transform<Real,3,DefaultTransformTrait>; //size:128 with double

    using Translation = Eigen::Translation<Real,3>;
    using Quaternion = Eigen::Quaternion<Real>;
    using AngleAxis = Eigen::AngleAxis<Real>;
    using ParametrizedLine = Eigen::ParametrizedLine<Real,3>;

    constexpr Real PI = EIGEN_PI;

    template<typename ScalarType>
    bool is_zero(ScalarType value, ScalarType prec = std::numeric_limits< float >::epsilon())
    {
        return Eigen::internal::isMuchSmallerThan(value,
                                                  static_cast<ScalarType>(1.0),
                                                  prec);
    }


}


#endif
