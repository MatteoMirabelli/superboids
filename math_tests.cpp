#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

TEST_CASE("Testing vec_norm function") {
  std::valarray<double> vec_1{1, 4};
  std::valarray<double> vec_2{2, 5};
  std::valarray<double> vec_3{0, 0};
  std::valarray<double> vec_4{-1, -4};

  double norm1 = vec_norm(vec_1);
  double norm2 = vec_norm(vec_2);
  double norm3 = vec_norm(vec_3);
  double norm4 = vec_norm(vec_4);

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
    Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 4, 1, -3, 120., 1920, 1080, 4, 1);
    Boid bd_3(0, 0, 3, 5, 120., 1920, 1080, 4, 1);
    Boid bd_4(0, 0, 6, -1, 120., 1920, 1080, 4, 1);
    Boid bd_5(1, 8, 0, 0, 120., 1920, 1080, 4, 1);

    double d12 = boid_dist(bd_1, bd_2);
    double d13 = boid_dist(bd_1, bd_3);
    double d34 = boid_dist(bd_3, bd_4);
    double d15 = boid_dist(bd_1, bd_5);

    CHECK(d12 == doctest::Approx(2.828427));
    CHECK(d13 == doctest::Approx(2.828427));
    CHECK(d34 == 0);
    CHECK(d15 == doctest::Approx(6.082762));
  }

  SUBCASE("Testing the boid_dist function with boids with other boids") {
    Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 4, 1, -3, 120., 1920, 1080, 4, 1);
    Boid bd_3(0, 0, 3, 5, 120., 1920, 1080, 4, 1);
    Boid bd_4(0, 0, 6, -1, 120., 1920, 1080, 4, 1);

    double d12 = boid_dist(bd_1, bd_2);
    double d21 = boid_dist(bd_2, bd_1);
    double d13 = boid_dist(bd_1, bd_3);
    double d31 = boid_dist(bd_3, bd_1);
    double d34 = boid_dist(bd_3, bd_4);
    double d43 = boid_dist(bd_4, bd_3);

    CHECK(d12 == doctest::Approx(2.828427));
    CHECK(d21 == doctest::Approx(2.828427));
    CHECK(d13 == doctest::Approx(2.828427));
    CHECK(d31 == doctest::Approx(2.828427));
    CHECK(d34 == 0);
    CHECK(d43 == 0);
  }

  SUBCASE("Testing the boid_dist function with just one boid") {
    Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    double d11 = boid_dist(bd_1, bd_1);

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

  double angle_1 = compute_angle<double>(vec_1);
  double angle_2 = compute_angle<double>(vec_2);
  double angle_3 = compute_angle<double>(vec_3);
  double angle_4 = compute_angle<double>(vec_4);
  double angle_5 = compute_angle<double>(vec_5);
  double angle_6 = compute_angle<double>(vec_6);
  double angle_7 = compute_angle<double>(vec_7);
  double angle_8 = compute_angle<double>(vec_8);
  double angle_9 = compute_angle<double>(vec_9);

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

TEST_CASE("Testing the is_visible function") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  // The first boid passed is the one which we want to know whether or not is
  // visible BY the second boid passed

  std::valarray<double> space{1920., 1080.};

  SUBCASE("Testing the is_visible function with view_angle 0") {
    Boid b1({1., 2.}, {2., 1.}, 0., space, 4, 1);
    Boid b2({3., 3.}, {2., 1.}, 0., space, 4, 1);

    bool iv12 = is_visible(b2, b1);
    bool iv21 = is_visible(b1, b2);
    
    CHECK(iv12 == true);
    CHECK(iv21 == false);
  }

  SUBCASE("Testing the is_visible function (I)") {
    Boid b1({1., 2.}, {3., 4.}, 150., space, 4, 1);
    Boid b2({4., 3.}, {2., 1.}, 150., space, 4, 1);
    Boid b3({1., 1.}, {1., 1.}, 150., space, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv13 = is_visible(b1, b3);

    CHECK(iv12 == false);
    CHECK(iv13 == true);
  }

  SUBCASE("Testing the is_visible function (II)") {
    Boid b1({1., 2.}, {0., -4.}, 90., space, 4, 1);
    Boid b2({4., 2.}, {0., +1.}, 90., space, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv21 = is_visible(b2, b1);

    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }

  SUBCASE("Testing the is_visible function (III)") {
    Boid b1({1., 1.}, {0., -1.}, 90., space, 4, 1);
    Boid b2({4., 3.}, {3., 2.}, 150., space, 4, 1);
    Boid b3({1., 2.}, {1., 0.}, 150., space, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv21 = is_visible(b2, b1);
    bool iv13 = is_visible(b1, b3);
    bool iv31 = is_visible(b3, b1);
    bool iv23 = is_visible(b2, b3);
    bool iv32 = is_visible(b3, b2);

    CHECK(iv12 == false);
    CHECK(iv21 == false);
    CHECK(iv13 == true);
    CHECK(iv31 == false);
    CHECK(iv23 == true);
    CHECK(iv32 == false);
  }

  SUBCASE("Testing the is_visible function (IV)") {
    Boid b1({154., 112.}, {-1., -1.}, 120., {1920., 1080.}, 10., 2.);
    Boid b2({152., 114.}, {-1., -1.}, 120., {1920., 1080.}, 10., 2.);
   
    bool iv12 = is_visible(b1, b2);
    bool iv21 = is_visible(b2, b1);

    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }

  SUBCASE(
      "Testing the is_visible function with boids with same x_position "
      "configurations") {
    Boid bd1({7., 3.}, {3., -1.}, 120., space, 5., 1.);
    Boid bd2({7., 8.}, {-2., -3.}, 120., space, 5., 1.);
    Boid bd3({7., 6.}, {-1., 0.}, 30., space, 5., 1.);

    bool iv12 = is_visible(bd1, bd2);
    bool iv21 = is_visible(bd2, bd1);
    bool iv13 = is_visible(bd1, bd3);
    bool iv31 = is_visible(bd3, bd1);
    bool iv23 = is_visible(bd2, bd3);
    bool iv32 = is_visible(bd3, bd2);

    CHECK(iv12 == true);
    CHECK(iv21 == true);
    CHECK(iv13 == false);
    CHECK(iv31 == true);
    CHECK(iv23 == false);
    CHECK(iv32 == true);
  }

  SUBCASE(
      "Testing the is_visible function with boids with same y_position "
      "configurations") {
    Boid bd1({15., 10.}, {-3., 0.}, 90., space, 5., 1.);
    Boid bd2({17., 10.}, {0., -3.}, 90., space, 5., 1.);
    Boid bd3({20., 10.}, {3., 0.}, 90., space, 5., 1.);

    bool iv12 = is_visible(bd1, bd2);
    bool iv21 = is_visible(bd2, bd1);
    bool iv23 = is_visible(bd2, bd3);
    bool iv32 = is_visible(bd3, bd2);

    CHECK(iv12 == true);
    CHECK(iv21 == false);
    CHECK(iv23 == false);
    CHECK(iv32 == true);
  }

  SUBCASE("Testing the is_visible function with boid on border of view_angle") {
    Boid bd1({10., 5.}, {0, 1.}, 45., space, 5., 1.);
    Boid bd2({7., 8.}, {-1., 0.}, 135., space, 5., 1.);

    bool iv12 = is_visible(bd1, bd2);
    bool iv21 = is_visible(bd2, bd1);

    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }
}

TEST_CASE("Testing the is_obs_visible") {
  // OBSTACLE CONSTRUCTOR takes: pos{x,y}, size;
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  SUBCASE("Testing the is_obs_visible with one visible obstacle and one not") {
    Obstacle ob1({40., 50.}, 10.);
    Obstacle ob2({52., 50.}, 20.);
    Boid bd({50., 40.}, {-2., -3.}, 110., {1920., 1080.}, 5., 3.);

    bool iv_ob1_bd = is_obs_visible(ob1, bd);
    bool iv_ob2_bd = is_obs_visible(ob2, bd);

    CHECK(iv_ob1_bd == true);
    CHECK(iv_ob2_bd == false);
  }

  SUBCASE(
      "Testing the is_obs_visible with obstacle on the edge of view_angle") {
    Obstacle ob1({40., 50.}, 10.);
    Boid bd({60., 50.}, {0., -3.}, 90., {1920., 1080.}, 5., 3.);

    bool iv_ob1_bd = is_obs_visible(ob1, bd);

    CHECK(iv_ob1_bd == true);
  }
}
