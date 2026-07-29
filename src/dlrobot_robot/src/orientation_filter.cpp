#include "dlrobot_robot/orientation_filter.hpp"

#include <cmath>

namespace dlrobot_robot
{

Quaternion OrientationFilter::Update(
  float angular_x, float angular_y, float angular_z,
  float acceleration_x, float acceleration_y, float acceleration_z)
{
  const float acceleration_norm = std::sqrt(
    acceleration_x * acceleration_x +
    acceleration_y * acceleration_y +
    acceleration_z * acceleration_z);

  if (acceleration_norm > 0.0F) {
    acceleration_x /= acceleration_norm;
    acceleration_y /= acceleration_norm;
    acceleration_z /= acceleration_norm;

    const float half_vx = q1_ * q3_ - q0_ * q2_;
    const float half_vy = q0_ * q1_ + q2_ * q3_;
    const float half_vz = q0_ * q0_ - 0.5F + q3_ * q3_;

    const float half_ex = acceleration_y * half_vz - acceleration_z * half_vy;
    const float half_ey = acceleration_z * half_vx - acceleration_x * half_vz;
    const float half_ez = acceleration_x * half_vy - acceleration_y * half_vx;

    if (kTwoKi > 0.0F) {
      integral_x_ += kTwoKi * half_ex / kSamplingFrequency;
      integral_y_ += kTwoKi * half_ey / kSamplingFrequency;
      integral_z_ += kTwoKi * half_ez / kSamplingFrequency;
      angular_x += integral_x_;
      angular_y += integral_y_;
      angular_z += integral_z_;
    } else {
      integral_x_ = 0.0F;
      integral_y_ = 0.0F;
      integral_z_ = 0.0F;
    }

    angular_x += kTwoKp * half_ex;
    angular_y += kTwoKp * half_ey;
    angular_z += kTwoKp * half_ez;
  }

  angular_x *= 0.5F / kSamplingFrequency;
  angular_y *= 0.5F / kSamplingFrequency;
  angular_z *= 0.5F / kSamplingFrequency;

  const float previous_q0 = q0_;
  const float previous_q1 = q1_;
  const float previous_q2 = q2_;
  q0_ += -previous_q1 * angular_x - previous_q2 * angular_y - q3_ * angular_z;
  q1_ += previous_q0 * angular_x + previous_q2 * angular_z - q3_ * angular_y;
  q2_ += previous_q0 * angular_y - previous_q1 * angular_z + q3_ * angular_x;
  q3_ += previous_q0 * angular_z + previous_q1 * angular_y - previous_q2 * angular_x;

  const float quaternion_norm = std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  if (quaternion_norm > 0.0F) {
    q0_ /= quaternion_norm;
    q1_ /= quaternion_norm;
    q2_ /= quaternion_norm;
    q3_ /= quaternion_norm;
  }

  return Quaternion{
    static_cast<double>(q1_),
    static_cast<double>(q2_),
    static_cast<double>(q3_),
    static_cast<double>(q0_)};
}

}  // namespace dlrobot_robot
