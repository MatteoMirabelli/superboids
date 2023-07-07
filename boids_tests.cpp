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

    bd::Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == doctest::Approx(3.));
    CHECK(boid.get_vel()[1] == doctest::Approx(3.));
    CHECK(boid.get_pos()[0] == doctest::Approx(5.));
    CHECK(boid.get_pos()[1] == doctest::Approx(5.));
    CHECK(boid.get_angle() == doctest::Approx(45.));
  }

  SUBCASE(
      "Testing the Boid::update_state method with null vel_coorection values") {
    std::valarray<double> pos{2., 2.};
    std::valarray<double> vel{1., 1.};
    std::valarray<double> window{1920, 1080};
    double view_angle = 120.;
    std::valarray<double> delta_vel{0., 0.};

    bd::Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == 1.);
    CHECK(boid.get_vel()[1] == 1.);
    CHECK(boid.get_pos()[0] == 3.);
    CHECK(boid.get_pos()[1] == 3.);
    CHECK(boid.get_angle() == 45.);
  }

  SUBCASE(
      "Testing the Boid::update_state method with negative values "
      "vel_correction values") {
    std::valarray<double> pos{3., 10.};
    std::valarray<double> vel{5., -4.};
    std::valarray<double> window{1920, 1080};
    double view_angle = 120.;
    std::valarray<double> delta_vel{0., -1.};

    bd::Boid boid(pos, vel, view_angle, window, 4, 1);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == 5.);
    CHECK(boid.get_vel()[1] == -5.);
    CHECK(boid.get_pos()[0] == 8.);
    CHECK(boid.get_pos()[1] == 5.);
  }
}

TEST_CASE("Testing the Boid::update_state method with borders") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1080, 1080}, param_ds_,
  // param_s
  // update_state takes: delta_t, delta_vel {vx, vy}, bhv, border_detection,
  // border_repulsion
  void update_state(double, std::valarray<double>, bool, double, double);
  SUBCASE("Testing the update_state with left border") {
    bd::Boid bd({20., 700}, {-5., 70.}, 120., {1080., 1080.}, 5., 4.);
    std::valarray<double> vel_corr{0., 0.};

    bd.update_state(1., vel_corr, false, 1., 1.);
    CHECK(bd.get_pos()[0] == 15.);
    CHECK(bd.get_pos()[1] == 770.);
    CHECK(bd.get_vel()[0] == doctest::Approx(13.71422));
    CHECK(bd.get_vel()[1] == doctest::Approx(70.));
  }

  SUBCASE(
      "Testing the update_state with borders on the edge of border detecion "
      "area (expected no correction)") {
    bd::Boid bd({40., 700}, {-7., 80.}, 120., {1080., 1080.}, 5., 4.);
    std::valarray<double> vel_corr{2., -1.};

    bd.update_state(1., vel_corr, false, 1., 1.);
    CHECK(bd.get_pos()[0] == 35.);
    CHECK(bd.get_pos()[1] == 779.);
    CHECK(bd.get_vel()[0] == -5);
    CHECK(bd.get_vel()[1] == 79);
  }

  SUBCASE("Testing the update_state with right border ") {
    bd::Boid bd({1045., 700}, {5., 76.}, 120., {1080., 1080.}, 5., 4.);
    std::valarray<double> vel_corr{2., -4.};

    bd.update_state(1., vel_corr, false, 2., 1.);
    CHECK(bd.get_pos()[0] == 1052.);
    CHECK(bd.get_pos()[1] == 772.);
    CHECK(bd.get_vel()[0] == doctest::Approx(-3.334211));
    CHECK(bd.get_vel()[1] == 72.);
  }

  SUBCASE("Testing the update_state with top border ") {
    bd::Boid bd({600., 40}, {75., 6.}, 120., {1080., 1080.}, 5., 4.);
    std::valarray<double> vel_corr{-4., -4.};

    bd.update_state(1., vel_corr, false, 3., 1.);
    CHECK(bd.get_pos()[0] == 671.);
    CHECK(bd.get_pos()[1] == 42.);
    CHECK(bd.get_vel()[0] == 71.);
    CHECK(bd.get_vel()[1] == doctest::Approx(8.764586));
  }

  SUBCASE("Testing the update_state with bottom border ") {
    bd::Boid bd({100., 1040}, {75., 6.}, 120., {1920., 1080.}, 5., 4.);
    std::valarray<double> vel_corr{-4., -4.};

    bd.update_state(1., vel_corr, false, 3., 1.);
    CHECK(bd.get_pos()[0] == 171.);
    CHECK(bd.get_pos()[1] == 1042.);
    CHECK(bd.get_vel()[0] == 71.);
    CHECK(bd.get_vel()[1] == doctest::Approx(-5.476648));
  }
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
    bd::Boid bd(init_pos, init_vel, 120., space, 4., 1.);
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
    bd::Boid bd(init_pos, init_vel, 120., space, 4., 1.);
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
    bd::Boid bd(init_pos, init_vel, 120., space, 4., 1.);
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
    bd::Boid bd(init_pos, init_vel, 120., space, 4., 1.);
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
    bd::Boid bd(init_pos, init_vel, 120., space, 4., 1.);
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
    bd::Boid bd(init_pos, init_vel, 120., space, 4., 1.);
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

  // avoid_obs takes:  obstacles, obstacle_repulsion,  obstacle_detection;

  SUBCASE("Testing the Boid::avoid_obs with no obstacles") {
    bd::Boid bd(8., 6., -2., 2., 120., 1920., 1080., 3., 1.);
    std::vector<ob::Obstacle> obstacles;
    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles within range") {
    // b_param_d_s = 6;
    bd::Boid bd(8., 6., -2., -2., 120., 1920., 1080., 6., 2.);
    ob::Obstacle ob1(4., 4., 1.);
    ob::Obstacle ob2(8., 2., 1.);
    std::vector<ob::Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 2.);

    CHECK(delta_vel[0] == doctest::Approx(2.82843));
    CHECK(delta_vel[1] == doctest::Approx(8.48528));
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two visible obstacles not in the "
      "range ") {
    // b_param_d_s = 0;
    // both obstacles in range;
    bd::Boid bd(10., 10., -2., -2., 120., 1920., 1080., 1., 1.);
    ob::Obstacle ob1(4., 4., 1.);
    ob::Obstacle ob2(8., 2., 1.);
    std::vector<ob::Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0);
  }

  SUBCASE(
      "Testing the Boid::avoid_obs with two NOT visible obstacles within "
      "5*param_s range ") {
    bd::Boid bd(8., 6., 2., 2., 90., 1920., 1080., 1., 1.);
    ob::Obstacle ob1(4., 4., 1.);
    ob::Obstacle ob2(8., 2., 1.);
    std::vector<ob::Obstacle> obstacles;
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
    bd::Boid bd(8., 6., 0., -2., 120., 1920., 1080., 5., 1.);
    ob::Obstacle ob1(1., 1., 1.);
    ob::Obstacle ob2(8., 2., 1.);
    std::vector<ob::Obstacle> obstacles;
    obstacles.push_back(ob1);
    obstacles.push_back(ob2);

    auto delta_vel = bd.avoid_obs(obstacles, 1., 1.5);

    CHECK(delta_vel[0] == 0);
    CHECK(delta_vel[1] == 0.75);
  }
}

TEST_CASE("Testing the get_vector_neighbours function") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c
  // FLOCK CONSTRUCTOR takes:
  // params, boid number, view_angle, window_space{1920, 1080};

  SUBCASE("Testing the get_vector_neighbours function with no neighbours") {
    fk::Parameters params(6., 4., 2., 2., 3.);
    fk::Flock flock(params, 0., 120., {1920., 1080.});

    bd::Boid bd({15., 10.}, {4., 7.}, 120., {1920., 1080}, 4., 2.);
    flock.push_back(bd);
    auto it = flock.begin();

    auto neighbours =
        bd::get_vector_neighbours(flock.get_flock(), it, flock.get_params().d);

    CHECK(neighbours.size() == 0);
  }

  SUBCASE("Testing the get_vector_neighbours function with three boids") {
    fk::Parameters params(6., 4., 2, 2, 3);
    fk::Flock flock(params, 0., 120., {1920., 1080.});

    bd::Boid bd1({15., 10.}, {-4., 0.}, 90., {1920., 1080}, 4., 2.);
    bd::Boid bd2({17., 10}, {0., -3.}, 90., {1920., 1080}, 4., 2.);
    bd::Boid bd3({20., 10.}, {4., 0.}, 90., {1920., 1080.}, 4., 2.);

    flock.push_back(bd1);
    flock.push_back(bd2);
    flock.push_back(bd3);
    auto it = flock.begin();

    auto neighbours1 =
        bd::get_vector_neighbours(flock.get_flock(), it, flock.get_params().d);
    ++it;
    auto neighbours2 =
        bd::get_vector_neighbours(flock.get_flock(), it, flock.get_params().d);

    CHECK(neighbours1.size() == 0);
    CHECK(neighbours2.size() == 2);
    CHECK(flock.size() == 3);
  }

  SUBCASE("Testing the get_vector_neighbours function with end") {
    fk::Parameters params(4, 4, 2, 2, 3);
    fk::Flock flock(params, 0., 120., {1920., 1080.});

    bd::Boid bd1({15., 10.}, {-4., 0.}, 90., {1920., 1080}, 4, 2);
    bd::Boid bd2({17., 10}, {0., -3.}, 90., {1920., 1080}, 4., 2);
    bd::Boid bd3({20., 10.}, {4., 0.}, 90., {1920., 1080.}, 4., 2.);

    flock.push_back(bd1);
    flock.push_back(bd2);
    flock.push_back(bd3);
    auto it = flock.end();

    auto neighbours =
        bd::get_vector_neighbours(flock.get_flock(), it, flock.get_params().d);

    CHECK(neighbours.size() == 0);
  }

  SUBCASE(
      "Testing the get_vector_neighbours function with boids out of range") {
    fk::Parameters params(4, 4, 2., 2, 3);
    fk::Flock flock(params, 0., 120., {1920., 1080.});

    bd::Boid bd1({15., 10.}, {2., 2.}, 90., {1920., 1080}, 4, 2);
    bd::Boid bd2({19., 14}, {2., 2.}, 90., {1920., 1080}, 4., 2);
    bd::Boid bd3({21., 16.}, {2., 2.}, 90., {1920., 1080.}, 4., 2.);

    flock.push_back(bd1);
    flock.push_back(bd2);
    flock.push_back(bd3);
    auto it = flock.begin();

    auto neighbours1 =
        bd::get_vector_neighbours(flock.get_flock(), it, flock.get_params().d);
    auto neighbours2 = bd::get_vector_neighbours(flock.get_flock(), it + 1,
                                                 flock.get_params().d);

    auto neighbours3 = bd::get_vector_neighbours(flock.get_flock(), it + 2,
                                                 flock.get_params().d);

    CHECK(neighbours1.size() == 0);
    CHECK(neighbours2.size() == 1);
    CHECK(neighbours2[0].get_pos()[0] == 21);
    CHECK(neighbours2[0].get_pos()[1] == 16.);
    CHECK(neighbours3.size() == 0);
  }
}

TEST_CASE("Testing the is_visible function") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  // The first boid passed is the one which we want to know whether or not is
  // visible BY the second boid passed

  std::valarray<double> space{1920., 1080.};

  SUBCASE("Testing the is_visible function with view_angle 0") {
    bd::Boid b1({1., 2.}, {2., 1.}, 0., space, 4, 1);
    bd::Boid b2({3., 3.}, {2., 1.}, 0., space, 4, 1);

    bool iv12 = bd::is_visible(b2, b1);
    bool iv21 = bd::is_visible(b1, b2);

    CHECK(iv12 == true);
    CHECK(iv21 == false);
  }

  SUBCASE("Testing the is_visible function (I)") {
    bd::Boid b1({1., 2.}, {3., 4.}, 150., space, 4, 1);
    bd::Boid b2({4., 3.}, {2., 1.}, 150., space, 4, 1);
    bd::Boid b3({1., 1.}, {1., 1.}, 150., space, 4, 1);

    bool iv12 = bd::is_visible(b1, b2);
    bool iv13 = bd::is_visible(b1, b3);

    CHECK(iv12 == false);
    CHECK(iv13 == true);
  }

  SUBCASE("Testing the is_visible function (II)") {
    bd::Boid b1({1., 2.}, {0., -4.}, 90., space, 4, 1);
    bd::Boid b2({4., 2.}, {0., +1.}, 90., space, 4, 1);

    bool iv12 = bd::is_visible(b1, b2);
    bool iv21 = bd::is_visible(b2, b1);

    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }

  SUBCASE("Testing the is_visible function (III)") {
    bd::Boid b1({1., 1.}, {0., -1.}, 90., space, 4, 1);
    bd::Boid b2({4., 3.}, {3., 2.}, 150., space, 4, 1);
    bd::Boid b3({1., 2.}, {1., 0.}, 150., space, 4, 1);

    bool iv12 = bd::is_visible(b1, b2);
    bool iv21 = bd::is_visible(b2, b1);
    bool iv13 = bd::is_visible(b1, b3);
    bool iv31 = bd::is_visible(b3, b1);
    bool iv23 = bd::is_visible(b2, b3);
    bool iv32 = bd::is_visible(b3, b2);

    CHECK(iv12 == false);
    CHECK(iv21 == false);
    CHECK(iv13 == true);
    CHECK(iv31 == false);
    CHECK(iv23 == true);
    CHECK(iv32 == false);
  }

  SUBCASE("Testing the is_visible function (IV)") {
    bd::Boid b1({154., 112.}, {-1., -1.}, 120., {1920., 1080.}, 10., 2.);
    bd::Boid b2({152., 114.}, {-1., -1.}, 120., {1920., 1080.}, 10., 2.);

    bool iv12 = bd::is_visible(b1, b2);
    bool iv21 = bd::is_visible(b2, b1);

    CHECK(iv12 == true);
    CHECK(iv21 == true);
  }

  SUBCASE(
      "Testing the is_visible function with boids with same x_position "
      "configurations") {
    bd::Boid bd1({7., 3.}, {3., -1.}, 120., space, 5., 1.);
    bd::Boid bd2({7., 8.}, {-2., -3.}, 120., space, 5., 1.);
    bd::Boid bd3({7., 6.}, {-1., 0.}, 30., space, 5., 1.);

    bool iv12 = bd::is_visible(bd1, bd2);
    bool iv21 = bd::is_visible(bd2, bd1);
    bool iv13 = bd::is_visible(bd1, bd3);
    bool iv31 = bd::is_visible(bd3, bd1);
    bool iv23 = bd::is_visible(bd2, bd3);
    bool iv32 = bd::is_visible(bd3, bd2);

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
    bd::Boid bd1({15., 10.}, {-3., 0.}, 90., space, 5., 1.);
    bd::Boid bd2({17., 10.}, {0., -3.}, 90., space, 5., 1.);
    bd::Boid bd3({20., 10.}, {3., 0.}, 90., space, 5., 1.);

    bool iv12 = bd::is_visible(bd1, bd2);
    bool iv21 = bd::is_visible(bd2, bd1);
    bool iv23 = bd::is_visible(bd2, bd3);
    bool iv32 = bd::is_visible(bd3, bd2);

    CHECK(iv12 == true);
    CHECK(iv21 == false);
    CHECK(iv23 == false);
    CHECK(iv32 == true);
  }

  SUBCASE("Testing the is_visible function with boid on border of view_angle") {
    bd::Boid bd1({10., 5.}, {0, 1.}, 45., space, 5., 1.);
    bd::Boid bd2({7., 8.}, {-1., 0.}, 135., space, 5., 1.);

    bool iv12 = bd::is_visible(bd1, bd2);
    bool iv21 = bd::is_visible(bd2, bd1);

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
    ob::Obstacle ob1({40., 50.}, 10.);
    ob::Obstacle ob2({52., 50.}, 20.);
    bd::Boid bd({50., 40.}, {-2., -3.}, 110., {1920., 1080.}, 5., 3.);

    bool iv_ob1_bd = bd::is_obs_visible(ob1, bd);
    bool iv_ob2_bd = bd::is_obs_visible(ob2, bd);

    CHECK(iv_ob1_bd == true);
    CHECK(iv_ob2_bd == false);
  }

  SUBCASE(
      "Testing the is_obs_visible with obstacle on the edge of view_angle") {
    ob::Obstacle ob1({40., 50.}, 10.);
    bd::Boid bd({60., 50.}, {0., -3.}, 90., {1920., 1080.}, 5., 3.);

    bool iv_ob1_bd = bd::is_obs_visible(ob1, bd);

    CHECK(iv_ob1_bd == true);
  }
}