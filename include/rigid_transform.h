#ifndef TRANSFORM_RIGID_TRANSFORM_H_
#define TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "port.h"

namespace transform {

template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  const Vector& translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

}  // namespace transform

#endif  // TRANSFORM_RIGID_TRANSFORM_H_
