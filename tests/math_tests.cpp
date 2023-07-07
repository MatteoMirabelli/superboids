#include "../simulation/boid.hpp"
#include "../doctest.h"
#include "../simulation/flock.hpp"
#include "../simulation/obstacles.hpp"
#include "../simulation/predator.hpp"

TEST_CASE("Testing vec_norm function") {
  std::valarray<double> vec_1{1, 4};
  std::valarray<double> vec_2{2, 5};
  std::valarray<double> vec_3{0, 0};
  std::valarray<double> vec_4{-1, -4};

  double norm1 = mt::vec_norm<double>(vec_1);
  double norm2 = mt::vec_norm<double>(vec_2);
  double norm3 = mt::vec_norm<double>(vec_3);
  double norm4 = mt::vec_norm<double>(vec_4);

  CHECK(norm1 == doctest::Approx(4.1231056));
  CHECK(norm2 == doctest::Approx(5.385164807));
  CHECK(norm3 == 0);
  CHECK(norm4 == doctest::Approx(4.1231056));
}

TEST_CASE("Testing boid_dist function") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  SUBCASE("Testing the boid_dist function") {
    bd::Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    bd::Boid bd_2(4, 4, 1, -3, 120., 1920, 1080, 4, 1);
    bd::Boid bd_3(0, 0, 3, 5, 120., 1920, 1080, 4, 1);
    bd::Boid bd_4(0, 0, 6, -1, 120., 1920, 1080, 4, 1);
    bd::Boid bd_5(1, 8, 0, 0, 120., 1920, 1080, 4, 1);

    double d12 = bd::boid_dist(bd_1, bd_2);
    double d13 = bd::boid_dist(bd_1, bd_3);
    double d34 = bd::boid_dist(bd_3, bd_4);
    double d15 = bd::boid_dist(bd_1, bd_5);

    CHECK(d12 == doctest::Approx(2.828427));
    CHECK(d13 == doctest::Approx(2.828427));
    CHECK(d34 == 0);
    CHECK(d15 == doctest::Approx(6.082762));
  }

  SUBCASE("Testing the boid_dist function with boids with other boids") {
    bd::Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    bd::Boid bd_2(4, 4, 1, -3, 120., 1920, 1080, 4, 1);
    bd::Boid bd_3(0, 0, 3, 5, 120., 1920, 1080, 4, 1);
    bd::Boid bd_4(0, 0, 6, -1, 120., 1920, 1080, 4, 1);

    double d12 = bd::boid_dist(bd_1, bd_2);
    double d21 = bd::boid_dist(bd_2, bd_1);
    double d13 = bd::boid_dist(bd_1, bd_3);
    double d31 = bd::boid_dist(bd_3, bd_1);
    double d34 = bd::boid_dist(bd_3, bd_4);
    double d43 = bd::boid_dist(bd_4, bd_3);

    CHECK(d12 == doctest::Approx(2.828427));
    CHECK(d21 == doctest::Approx(2.828427));
    CHECK(d13 == doctest::Approx(2.828427));
    CHECK(d31 == doctest::Approx(2.828427));
    CHECK(d34 == 0);
    CHECK(d43 == 0);
  }

  SUBCASE("Testing the boid_dist function with just one boid") {
    bd::Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    double d11 = bd::boid_dist(bd_1, bd_1);

    CHECK(d11 == 0);
  }
}

TEST_CASE("Testing the compute_angle function") {
  std::valarray<double> vec_1{1, 4};
  std::valarray<double> vec_2{1, -4};
  std::valarray<double> vec_3{-1, -4};
  std::valarray<double> vec_4{-1., -6};
  std::valarray<double> vec_5{0., 0.};
  std::valarray<double> vec_6{0., 6.};
  std::valarray<double> vec_7{0., -4.};
  std::valarray<double> vec_8{-3., 2.};
  std::valarray<double> vec_9{4., 1};

  double angle_1 = mt::compute_angle<double>(vec_1);
  double angle_2 = mt::compute_angle<double>(vec_2);
  double angle_3 = mt::compute_angle<double>(vec_3);
  double angle_4 = mt::compute_angle<double>(vec_4);
  double angle_5 = mt::compute_angle<double>(vec_5);
  double angle_6 = mt::compute_angle<double>(vec_6);
  double angle_7 = mt::compute_angle<double>(vec_7);
  double angle_8 = mt::compute_angle<double>(vec_8);
  double angle_9 = mt::compute_angle<double>(vec_9);

  CHECK(angle_1 == doctest::Approx(14.036243));
  CHECK(angle_2 == doctest::Approx(165.963756));
  CHECK(angle_3 == doctest::Approx(-165.9637565));
  CHECK(angle_4 == doctest::Approx(-170.53767779));
  CHECK(angle_5 == 0.);
  CHECK(angle_6 == 0.);
  CHECK(angle_7 == 180.);
  CHECK(angle_8 == doctest::Approx(-56.30994327));
  CHECK(angle_9 == doctest::Approx(75.96375653));
}
