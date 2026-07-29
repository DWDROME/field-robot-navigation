#ifndef DLROBOT_ROBOT__ORIENTATION_FILTER_HPP_
#define DLROBOT_ROBOT__ORIENTATION_FILTER_HPP_

namespace dlrobot_robot
{

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

class OrientationFilter
{
public:
  Quaternion Update(
    float angular_x, float angular_y, float angular_z,
    float acceleration_x, float acceleration_y, float acceleration_z);

private:
  static constexpr float kSamplingFrequency = 20.0F;
  static constexpr float kTwoKp = 1.0F;
  static constexpr float kTwoKi = 0.0F;

  float q0_{1.0F};
  float q1_{0.0F};
  float q2_{0.0F};
  float q3_{0.0F};
  float integral_x_{0.0F};
  float integral_y_{0.0F};
  float integral_z_{0.0F};
};

}  // namespace dlrobot_robot

#endif  // DLROBOT_ROBOT__ORIENTATION_FILTER_HPP_
