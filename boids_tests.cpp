#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

TEST_CASE(
    "Testing the Boid::update_state method without any conditions on border") {
  SUBCASE("Testing the Boid::update_state method with positive values") {
    std::valarray<double> pos{2., 2.};
    std::valarray<double> vel{2., 2.};
    std::valarray<double> window{1920, 1080};
    double view_angle = 120.;
    std::valarray<double> delta_vel{1., 1.};

    Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == doctest::Approx(3.));
    CHECK(boid.get_vel()[1] == doctest::Approx(3.));
    CHECK(boid.get_pos()[0] == doctest::Approx(5.));
    CHECK(boid.get_pos()[1] == doctest::Approx(5.));
    CHECK(boid.get_angle() == doctest::Approx(45.));
  }

  SUBCASE("Testing the Boid::update_state method with null values") {
    std::valarray<double> pos{2., 2.};
    std::valarray<double> vel{1., 1.};
    std::valarray<double> window{1920, 1080};
    double view_angle = 120.;
    std::valarray<double> delta_vel{0., 0.};

    Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == 1.);
    CHECK(boid.get_vel()[1] == 1.);
    CHECK(boid.get_pos()[0] == 3.);
    CHECK(boid.get_pos()[1] == 3.);
    CHECK(boid.get_angle() == 45.);
  }

  SUBCASE("Testing the Boid::update_state method with negative values") {
    std::valarray<double> pos{3., 10.};
    std::valarray<double> vel{5., -4.};
    std::valarray<double> window{1920, 1080};
    double view_angle = 120.;
    std::valarray<double> delta_vel{0., -1.};

    Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == 5.);
    CHECK(boid.get_vel()[1] == -5.);
    CHECK(boid.get_pos()[0] == 8.);
    CHECK(boid.get_pos()[1] == 5.);
  }
}

TEST_CASE("Testing the Boid::update_state method with borders") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
}

TEST_CASE("Testing the Boid::update_state method with periodic conditions") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  // update_state(double delta_t, std::valarray<double> delta_vel, bool const&
  // brd_bhv, double param_d, double repulsion_factor)

  // REMEMBER: Max speed: 350; Min speed: 70; ( sqrt((vel_x)^2+(vel_y)^2) )

  SUBCASE(
      "Testing the Boid::update_state method with periodic conditions on left "
      "border") {
    std::valarray<double> space{1920, 1080};
    std::valarray<double> init_pos{100., 100.};
    std::valarray<double> init_vel{-75., 75.};
    std::valarray<double> delta_vel{-30., +4.};
    Boid bd(init_pos, init_vel, 120., space, 4., 1.);
    bd.update_state(1., delta_vel, true);

    CHECK(bd.get_vel()[0] == -105);
    CHECK(bd.get_vel()[1] == 79);
    CHECK(bd.get_pos()[0] == 1899);
    CHECK(bd.get_pos()[1] == 179);
  }

  SUBCASE(
      "Testing the Boid::update_state method with periodic conditions on right "
      "border") {
    std::valarray<double> space{1920, 1080};
    std::valarray<double> init_pos{1880., 100.};
    std::valarray<double> init_vel{75., 75.};
    std::valarray<double> delta_vel{30., +4.};
    Boid bd(init_pos, init_vel, 120., space, 4., 1.);
    bd.update_state(1., delta_vel, true);

    CHECK(bd.get_vel()[0] == 105);
    CHECK(bd.get_vel()[1] == 79);
    CHECK(bd.get_pos()[0] == 21);
    CHECK(bd.get_pos()[1] == 179);
  }

  SUBCASE(
      "Testing the Boid::update_state method with periodic conditions on top "
      "border") {
    std::valarray<double> space{1920, 1080};
    std::valarray<double> init_pos{500., 25.};
    std::valarray<double> init_vel{100., 7.};
    std::valarray<double> delta_vel{30., -15.};
    Boid bd(init_pos, init_vel, 120., space, 4., 1.);
    bd.update_state(1., delta_vel, true);

    CHECK(bd.get_vel()[0] == 130);
    CHECK(bd.get_vel()[1] == -8);
    CHECK(bd.get_pos()[0] == 630);
    CHECK(bd.get_pos()[1] == 1059);
  }

  SUBCASE(
      "Testing the Boid::update_state method with periodic conditions on "
      "bottom border") {
    std::valarray<double> space{1920, 1080};
    std::valarray<double> init_pos{500., 1050.};
    std::valarray<double> init_vel{100., 7.};
    std::valarray<double> delta_vel{30., 15.};
    Boid bd(init_pos, init_vel, 120., space, 4., 1.);
    bd.update_state(1., delta_vel, true);

    CHECK(bd.get_vel()[0] == 130);
    CHECK(bd.get_vel()[1] == 22);
    CHECK(bd.get_pos()[0] == 630);
    CHECK(bd.get_pos()[1] == 21);
  }

  SUBCASE(
      "Testing the Boid::update_state method with periodic conditions on top "
      "left corner") {
    std::valarray<double> space{1920, 1080};
    std::valarray<double> init_pos{1880., 100.};
    std::valarray<double> init_vel{20., -70.};
    std::valarray<double> delta_vel{30., -15.};
    Boid bd(init_pos, init_vel, 120., space, 4., 1.);
    bd.update_state(1., delta_vel, true);

    CHECK(bd.get_vel()[0] == 50);
    CHECK(bd.get_vel()[1] == -85);
    CHECK(bd.get_pos()[0] == 21);
    CHECK(bd.get_pos()[1] == 1059);
  }

  SUBCASE(
      "Testing the Boid::update_state method with periodic conditions: boid "
      "exactly on top left corner, no correction needed") {
    std::valarray<double> space{1920, 1080};
    std::valarray<double> init_pos{1880., 100.};
    std::valarray<double> init_vel{20., -70.};
    std::valarray<double> delta_vel{0., -10.};
    Boid bd(init_pos, init_vel, 120., space, 4., 1.);
    bd.update_state(1., delta_vel, true);

    CHECK(bd.get_vel()[0] == 20);
    CHECK(bd.get_vel()[1] == -80);
    CHECK(bd.get_pos()[0] == 1900);
    CHECK(bd.get_pos()[1] == 20);
  }
}

TEST_CASE("Testing the Boid::avoid_obs method") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  // OBSTACLE CONSTRUCTOR takes: pos{x,y}, size

  SUBCASE("Testing the Boid::avoid_obs with no obstacles") {
    Boid bd(8., 6., -2., 2., 120., 1920., 1080., 3., 1.);
    std::vector<Obstacle> obstacles;
    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles within range") {
    // b_param_d_s = 6;
    Boid bd(8., 6., -2., -2., 120., 1920., 1080., 5., 1.);
    Obstacle ob1(4., 4., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    Boid bdx(8., 2., 0., 0., 120., 1920., 1080., 5., 1.);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 6);
    CHECK(delta_vel[1] == 9);
    CHECK(is_visible(bdx, bd) == true);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles not in the "
      "range ") {
    // b_param_d_s = 0;
    // both obstacles in range;
    Boid bd(10., 10., -2., -2., 120., 1920., 1080., 1., 1.);
    Obstacle ob1(4., 4., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two NOT visible obstacles within "
      "5*param_s range ") {
    Boid bd(8., 6., 2., 2., 90., 1920., 1080., 1., 1.);
    Obstacle ob1(4., 4., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles, with just one "
      "within range") {
    // b_param_d_s = 5;
    // ob_1 not in range;
    // ob_2 in range;
    Boid bd(8., 6., 0., -2., 120., 1920., 1080., 5., 1.);
    Obstacle ob1(1., 1., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 6);
  }
}

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

  SUBCASE("Testing the is_visible function") {
    Boid b1({1., 2.}, {3., 4.}, 150., space, 4, 1);
    Boid b2({4., 3.}, {2., 1.}, 150., space, 4, 1);
    Boid b3({1., 1.}, {1., 1.}, 150., space, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv13 = is_visible(b1, b3);
    CHECK(iv12 == false);
    CHECK(iv13 == true);
  }

  SUBCASE("Testing the is_visible function") {
    Boid b1({1., 2.}, {0., -4.}, 90., space, 4, 1);
    Boid b2({4., 2.}, {0., +1.}, 90., space, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv21 = is_visible(b2, b1);
    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }

  SUBCASE("Testing the is_visible function") {
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

  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  SUBCASE(
      "Testing the is_visible function with boids with same x_position "
      "configurations") {
    Boid bd1({7., 3.}, {1., -1.}, 120., space, 5., 1.);
    Boid bd2({7., 8.}, {-2., -3.}, 120., space, 5., 1.);
    Boid bd3({7., 6.}, {-1., 0.}, 30., space, 5., 1.);

    bool iv12 = is_visible(bd1, bd2);
    bool iv21 = is_visible(bd2, bd1);
    bool iv13 = is_visible(bd1, bd3);
    bool iv31 = is_visible(bd3, bd1);
    bool iv23 = is_visible(bd2, bd3);
    bool iv32 = is_visible(bd3, bd2);

    CHECK(iv12 == true);  //
    CHECK(iv21 == true);
    CHECK(iv13 == false);
    CHECK(iv31 == true);  //
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
