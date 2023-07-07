#ifndef MATH_HPP
#define MATH_HPP

#include <cmath>
#include <valarray>

namespace mt {
// It return the norm of a vector
template <typename T>
T vec_norm(std::valarray<T> vec) {
  return std::sqrt(std::pow(vec, {2., 2.}).sum());
}

// It computes the angle of the vector
template <typename T>
T compute_angle(std::valarray<T> const& vec) {
  // assert(vec.size() == 2);
  double angle{0.};
  if (vec[1] == 0. && vec[0] < 0.) {
    angle = -90.;
  } else if (vec[1] == 0. && vec[0] > 0.) {
    angle = 90.;
  } else if (vec[1] == 0. && vec[0] == 0.) {
    angle = 0.;
  } else if (vec[0] == 0. && vec[1] > 0.) {
    angle = 0.;
  } else if (vec[0] == 0. && vec[1] < 0.) {
    angle = 180.;
  } else {
    angle = std::atan(vec[0] / vec[1]) / M_PI * 180;
    (vec[1] < 0. && vec[0] < 0.) ? angle -= 180. : angle;
    (vec[1] < 0. && vec[0] > 0.) ? angle += 180. : angle;
  }
  return angle;
}
}  // namespace mt
#endif