#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

TEST_CASE("Testing the Boid::update_state method without border behaviour") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

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

    CHECK(boid.get_vel()[0] == doctest::Approx(1.));
    CHECK(boid.get_vel()[1] == doctest::Approx(1.));
    CHECK(boid.get_pos()[0] == doctest::Approx(3.));
    CHECK(boid.get_pos()[1] == doctest::Approx(3.));
    CHECK(boid.get_angle() == doctest::Approx(45.));
  }

  SUBCASE("Testing the Boid::update_state method with negative values") {
    std::valarray<double> pos{3., 10.};
    std::valarray<double> vel{5., -4.};
    std::valarray<double> window{1920, 1080};
    double view_angle = 120.;
    std::valarray<double> delta_vel{0., -1.};

    Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == doctest::Approx(5.));
    CHECK(boid.get_vel()[1] == doctest::Approx(-5.));
    CHECK(boid.get_pos()[0] == doctest::Approx(8.));
    CHECK(boid.get_pos()[1] == doctest::Approx(5.));
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
    bd.update_state(1., delta_vel, true, 5., 1);

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
    bd.update_state(1., delta_vel, true, 5., 1);

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
    bd.update_state(1., delta_vel, true, 5., 1);

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
    bd.update_state(1., delta_vel, true, 5., 1);

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
    bd.update_state(1., delta_vel, true, 5., 1);

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
    bd.update_state(1., delta_vel, true, 5., 1);

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

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles within range") {
    Boid bd(8., 6., -2., 2., 120., 1920., 1080., 3., 1.);
    Obstacle ob1(4., 4., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles);

    CHECK(delta_vel[0] == -6);
    CHECK(delta_vel[1] == -9);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles not in the "
      "range ") {
    Boid bd(10., 10., -2., 2., 120., 1920., 1080., 1., 1.);
    Obstacle ob1(4., 4., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles);

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

    auto delta_vel = bd.avoid_obs(obstacles);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles, with just one "
      "within range") {
    Boid bd(8., 6., -2., 2., 120., 1920., 1080., 1., 1.);
    Obstacle ob1(1., 1., 1.);
    Obstacle ob2(8., 2., 1.);
    std::vector<Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == -6);
  }
}

TEST_CASE("Testing auxiliary functions") {
  SUBCASE("Testing the vec_norm function") {
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

  SUBCASE("Testing the compute_angle function") {
    std::valarray<double> vec_1{1, 4};
    std::valarray<double> vec_2{1, -4};
    std::valarray<double> vec_3{-1, -4};

    double angle_1 = compute_angle<double>(vec_1);
    double angle_2 = compute_angle<double>(vec_2);
    double angle_3 = compute_angle<double>(vec_3);

    CHECK(angle_1 == doctest::Approx(14.036243));
    CHECK(angle_2 == doctest::Approx(165.963756));
    CHECK(angle_3 == doctest::Approx(194.036243));
  }
}

TEST_CASE("Testing the is_visible function") {
  // The first boid passed is the one which we want to know whether or not is
  // visible BY the second boid passed
  SUBCASE("Testing the is_visible function with view_angle 0") {
    Boid b1(1., 2., 2., 1., 0., 1920, 1080, 4, 1);
    Boid b2(3., 3., 2., 1., 0., 1920, 1080, 4, 1);

    bool iv12 = is_visible(b2, b1);
    bool iv21 = is_visible(b1, b2);
    CHECK(iv12 == true);
    CHECK(iv21 == false);
  }

  SUBCASE("Testing the is_visible function") {
    Boid b1(1., 2., 3., 4., 150., 1920, 1080, 4, 1);
    Boid b2(4., 3., 2., 1., 150., 1920, 1080, 4, 1);
    Boid b3(1., 1., 1., 1., 150., 1920, 1080, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv13 = is_visible(b1, b3);
    CHECK(iv12 == false);
    CHECK(iv13 == true);
  }

  SUBCASE("Testing the is_visible function") {
    Boid b1(1., 2., 0., -4., 90., 1920, 1080, 4, 1);
    Boid b2(4., 2., 0., +1., 90., 1920, 1080, 4, 1);

    bool iv12 = is_visible(b1, b2);
    bool iv21 = is_visible(b2, b1);
    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }

  SUBCASE("Testing the is_visible function") {
    Boid b1(1., 1., 0., -1., 90., 1920, 1080, 4, 1);
    Boid b2(4., 3., 3., 2., 150., 1920, 1080, 4, 1);
    Boid b3(1., 2., 1., 0., 150., 1920, 1080, 4, 1);

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
}

TEST_CASE("Testing the Boid::sort method") {
  SUBCASE("Testing the Flock::sort method with five boids boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(0, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(54, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_2);
    flock.push_back(bd_1);
    flock.push_back(bd_4);
    flock.push_back(bd_3);

    flock.sort();

    CHECK(flock.get_boid(1).get_pos()[0] == 0);
    CHECK(flock.get_boid(2).get_pos()[0] == 1);
    CHECK(flock.get_boid(3).get_pos()[0] == 4);
    CHECK(flock.get_boid(4).get_pos()[0] == 54);
  }

  SUBCASE("Testing the Flock::sort method with boids with same x_position") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(10, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_2);
    flock.push_back(bd_4);
    flock.push_back(bd_3);
    flock.push_back(bd_1);

    flock.sort();

    CHECK(flock.get_boid(1).get_pos()[0] == 1);
    CHECK(flock.get_boid(2).get_pos()[0] == 4);
    CHECK(flock.get_boid(3).get_pos()[0] == 10);
    CHECK(flock.get_boid(4).get_pos()[0] == 10);
    CHECK(flock.get_boid(3).get_pos()[1] == 3);
    CHECK(flock.get_boid(4).get_pos()[1] == 4);
  }

  SUBCASE("Testing the Flock::sort method with just one boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);

    flock.sort();

    CHECK(flock.get_boid(1).get_pos()[0] == 1);
    CHECK(flock.get_boid(1).get_pos()[1] == 4);
  }
}

TEST_CASE("Testing the Flock::update_com method") {
  SUBCASE("Testing the Flock::update_com method with four boids") {
    Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    Boid bd_2(5, 7, 1, -3, 120., 1920, 1080, 4, 1);
    Boid bd_3(6, 8, 4, 3, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 12, -4, 8, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    flock.update_com();

    CHECK(flock.get_com().get_pos()[0] == 5.75);
    CHECK(flock.get_com().get_pos()[1] == 7.25);
    CHECK(flock.get_com().get_vel()[0] == 1.5);
    CHECK(flock.get_com().get_vel()[1] == 3);
  }

  SUBCASE("Testing the Flock::update_com method with just one boid") {
    Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.update_com();

    CHECK(flock.get_com().get_pos()[0] == 2);
    CHECK(flock.get_com().get_pos()[1] == 2);
    CHECK(flock.get_com().get_vel()[0] == 5);
    CHECK(flock.get_com().get_vel()[1] == 4);
  }

  SUBCASE("Testing the Flock::update_com method with two boids") {
    Boid bd_1(0, 0, 0, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);

    flock.update_com();

    CHECK(flock.get_com().get_pos()[0] == 0);
    CHECK(flock.get_com().get_pos()[1] == 0);
    CHECK(flock.get_com().get_vel()[0] == 0);
    CHECK(flock.get_com().get_vel()[1] == 0);
  }

  SUBCASE("Testing the Flock::sort method with two boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_2);
    flock.push_back(bd_1);

    flock.sort();

    CHECK(flock.get_boid(1).get_pos()[0] == 1);
    CHECK(flock.get_boid(2).get_pos()[0] == 4);
  }
}

TEST_CASE("Testing the Flock::get_neighbours method") {
  SUBCASE("Testing the Flock::get_neighbours method with four boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920., 1080., 4., 1.);
    Boid bd_2(3, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(10, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_4);
    flock.push_back(bd_3);

    auto it = flock.begin();
    ++it;
    std::vector<Boid> neighbours = flock.get_neighbours(it);

    CHECK(neighbours.size() == 1);
    CHECK(neighbours[0].get_pos()[0] == 1);
    CHECK(neighbours[0].get_pos()[1] == 4);
  }

  SUBCASE("Testing the Flock::get_neighbours method again with four boids ") {
    Boid bd_1(1., 4., 5., 0., 150., 1920., 1080., 4., 1.);
    Boid bd_2(3, 3, -2, 9, 150., 1920, 1080, 4, 1);
    Boid bd_3(4, 4, 5, 0, 150., 1920, 1080, 4, 1);
    Boid bd_4(6, 7, -2, 9, 150., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 150., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    auto it = flock.begin() + 1;

    std::vector<Boid> neighbours = flock.get_neighbours(it);

    CHECK(neighbours.size() == 2);

    CHECK(neighbours[0].get_pos()[0] == 4);
    CHECK(neighbours[0].get_pos()[1] == 4);
    CHECK(neighbours[1].get_pos()[0] == 1);
    CHECK(neighbours[1].get_pos()[1] == 4);
  }

  SUBCASE(
      "Testing the Flock::get_neighbours method with two boids that don't "
      "see "
      "each other") {
    Boid bd_1(7, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 3, -5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_2);
    flock.push_back(bd_1);

    auto it = flock.begin() + 1;

    std::vector<Boid> neighbours = flock.get_neighbours(it);

    CHECK(neighbours.size() == 0);
  }

  SUBCASE(
      "Testing the Flock::get_neighbours method with four boids, the last "
      "one "
      "doesn't see the others") {
    Boid bd_1(1, 1, 1, 1, 120., 1920, 1080, 4, 1);
    Boid bd_2(3, 3, 1, 1, 120., 1920, 1080, 4, 1);
    Boid bd_3(4, 4, 1, 1, 120., 1920, 1080, 4, 1);
    Boid bd_4(6, 7, 1, 1, 120., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    auto it = flock.begin() + 1;
    auto ut = flock.end() - 1;

    std::vector<Boid> neighbours_1 = flock.get_neighbours(it);
    std::vector<Boid> neighbours_2 = flock.get_neighbours(ut);

    CHECK(neighbours_1.size() == 1);
    CHECK(neighbours_1[0].get_pos()[0] == 4);
    CHECK(neighbours_1[0].get_pos()[1] == 4);

    CHECK(neighbours_2.size() == 0);
  }

  SUBCASE("Testing the Flock::get_neighbours method with just one boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);

    auto it = flock.begin();
    auto neighbours = flock.get_neighbours(it);

    CHECK(neighbours.size() == 0);
  }

  SUBCASE("Testing the Flock::get_neighbours method with Flock::end") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(3, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(4, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(6, 7, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    auto it = flock.end();
    auto neighbours = flock.get_neighbours(it);

    CHECK(neighbours.size() == 0);
  }
}

TEST_CASE("Testing the Flock::vel_correction method without predator") {
  SUBCASE("Testing the Flock::vel_correction method with four boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(3, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(10, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_4);
    flock.push_back(bd_3);

    const auto it = flock.begin();
    const auto dv1 = flock.vel_correction(it);
    CHECK(dv1[0] == -10.);
    CHECK(dv1[1] == 16.);
  }

  SUBCASE("Testing the Flock::vel_correction method again with four boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(3, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(4, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(6, 7, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    const auto it = flock.begin() + 1;
    const auto dv2 = flock.vel_correction(it);
    CHECK(dv2[0] == 13.5);
    CHECK(dv2[1] == -17.);
  }

  SUBCASE(
      "Testing the Flock::vel_correction method with two boids that don't "
      "see "
      "each other") {
    Boid bd_1(7, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 3, -5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_2);
    flock.push_back(bd_1);

    const auto it = flock.begin() + 1;
    const auto dv2 = flock.vel_correction(it);
    CHECK(dv2[0] == 0.);
    CHECK(dv2[1] == 0.);
  }

  SUBCASE(
      "Testing the Flock::vel_correction method with four boids, the last "
      "one "
      "doesn't see the others") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(3, 3, 5, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(4, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(6, 7, 5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    const auto it = flock.begin() + 1;
    const auto ut = flock.end() - 1;
    const auto dv1 = flock.vel_correction(it);
    const auto dv2 = flock.vel_correction(ut);
    CHECK(dv1[0] == -0.5);
    CHECK(dv1[1] == -17.);
    CHECK(dv2[0] == 0.);
    CHECK(dv2[1] == 0.);
  }

  SUBCASE("Testing the Flock::vel_correction method with just one boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);
    Boid com(0, 0, 0, 0, 120., 1920, 1080, 4, 1);

    Flock flock(params, 0, com, 120., {1920, 1080});

    flock.push_back(bd_1);

    const auto it = flock.begin();
    const auto dv = flock.vel_correction(it);
    CHECK(dv[0] == 0.);
    CHECK(dv[1] == 0.);
  }
}

TEST_CASE("Testing the Flock::vel_correction method with predator") {
  SUBCASE(
      "Testing the Flock::vel_correction method with predator with three "
      "boids") {}
}

TEST_CASE("Testing the Flock::Update_stats method") {
  SUBCASE("Testing the Flock::update_stats method with no neighbours") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(10, 6, 2, 0, 120., 1920, 1080, 4, 1);

    Parameters params(3, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.sort();

    flock.update_stats();

    CHECK(flock.get_stats().av_dist == 0);
    CHECK(flock.get_stats().dist_RMS == 0);
    CHECK(flock.get_stats().av_vel == 3.5);
    CHECK(flock.get_stats().vel_RMS == 1.5);
  }

  SUBCASE("Testing the Flock::update_stats method with boids") {
    Boid bd_1(1, 1, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(2, 2, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(3, 3, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(4, 4, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(10, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);
    flock.sort();

    flock.update_stats();

    CHECK(flock.size() == 4);
    CHECK(flock.get_stats().av_dist == doctest::Approx(2.3570226));
    CHECK(flock.get_stats().dist_RMS == doctest::Approx(1.054092562));
    CHECK(flock.get_stats().av_vel == doctest::Approx(7.10977223));
    CHECK(flock.get_stats().vel_RMS == doctest::Approx(2.10977224));
  }

  SUBCASE("Testing the Flock::update_stats method with boids") {
    Boid bd_1(1, 1, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(2, 2, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(3, 3, 5, 0, 120., 1920, 1080, 4, 1);

    Boid bd_4(1, 1, 0, 0, 120., 1920, 1080, 4, 1);
    Boid bd_5(4, 1, 0, 0, 120., 1920, 1080, 4, 1);
    Boid bd_6(2, 3, 0, 0, 120., 1920, 1080, 4, 1);

    Parameters params(10, 4, 1, 2, 3);

    Flock flock_1(params, 0, 120., {1920, 1080});
    Flock flock_2(params, 0, 120., {1920, 1080});

    flock_1.push_back(bd_1);
    flock_1.push_back(bd_2);
    flock_1.push_back(bd_3);

    flock_2.push_back(bd_4);
    flock_2.push_back(bd_5);
    flock_2.push_back(bd_6);

    flock_1.sort();
    flock_2.sort();

    flock_1.update_stats();
    flock_2.update_stats();

    CHECK(flock_1.size() == 3);
    CHECK(flock_1.get_stats().av_dist == doctest::Approx(1.885618));
    CHECK(flock_1.get_stats().dist_RMS == doctest::Approx(0.666666));
    CHECK(flock_1.get_stats().av_vel == doctest::Approx(6.4065148));
    CHECK(flock_1.get_stats().vel_RMS == doctest::Approx(1.98911239));

    CHECK(flock_2.size() == 3);
    CHECK(flock_2.get_stats().av_dist == doctest::Approx(2.688165));
    CHECK(flock_2.get_stats().dist_RMS == doctest::Approx(0.3272645));
    CHECK(flock_2.get_stats().av_vel == 0);
    CHECK(flock_2.get_stats().vel_RMS == 0);
  }
}