#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

TEST_CASE("Testing the Flock::sort method") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c

  SUBCASE("Testing the Flock::sort method with five boids boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(4, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(0, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(54, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

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

    Flock flock(params, 0, 120., {1920, 1080});

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

  SUBCASE("Testing the Flock::sort method with just one boid") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);

    flock.sort();

    CHECK(flock.get_boid(1).get_pos()[0] == 1);
    CHECK(flock.get_boid(1).get_pos()[1] == 4);
  }
}

TEST_CASE("Testing the Flock::update_com method") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c

  SUBCASE("Testing the Flock::update_com method with four boids") {
    Boid bd_1(2, 2, 5, 4, 120., 1920, 1080, 4, 1);
    Boid bd_2(5, 7, 1, -3, 120., 1920, 1080, 4, 1);
    Boid bd_3(6, 8, 4, 3, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 12, -4, 8, 120., 1920, 1080, 4, 1);

    Parameters params(4, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

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

    Flock flock(params, 0, 120., {1920, 1080});

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

    Flock flock(params, 0, 120., {1920, 1080});

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

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_2);
    flock.push_back(bd_1);

    flock.sort();

    CHECK(flock.get_boid(1).get_pos()[0] == 1);
    CHECK(flock.get_boid(2).get_pos()[0] == 4);
  }
}

TEST_CASE("Testing the Flock::get_neighbours method") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c

  SUBCASE("Testing the Flock::get_neighbours method with four boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920., 1080., 4., 1.);
    Boid bd_2(3, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(10, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

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

    Flock flock(params, 0, 120., {1920, 1080});

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
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c

  SUBCASE("Testing the Flock::vel_correction method with four boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_2(3, 3, -2, 9, 120., 1920, 1080, 4, 1);
    Boid bd_3(10, 4, 5, 0, 120., 1920, 1080, 4, 1);
    Boid bd_4(10, 3, -2, 9, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

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
    const auto dv4 = flock.vel_correction(ut);
    CHECK(dv1[0] == -0.5);
    CHECK(dv1[1] == -17.);
    CHECK(dv4[0] == 0.);
    CHECK(dv4[1] == 0.);
  }

  SUBCASE("Testing the Flock::vel_correction method with just one boids") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);

    Parameters params(2.5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);

    const auto it = flock.begin();
    const auto dv = flock.vel_correction(it);
    CHECK(dv[0] == 0.);
    CHECK(dv[1] == 0.);
  }
}

TEST_CASE("Testing the Flock::vel_correction method with predator") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s

  // PREDATOR CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s, p_range, p_hunger

  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c

  SUBCASE(
      "Testing the Flock::vel_correction method with predator with four "
      "boids and one predator") {
    Boid bd_1(1., 4., 5., 0., 120., 1920., 1080., 4., 1.);
    Boid bd_2(3., 3., 5., 9., 120., 1920., 1080., 4., 1.);
    Boid bd_3(4., 4., 5., 0., 120., 1920., 1080., 4., 1.);
    Boid bd_4(6, 7, 5, 0, 120., 1920, 1080, 4, 1);

    Predator pd(1., 1., 2., 2., 140., 1920., 1080., 4., 1., 5., 1.);

    Parameters params(5, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    std::vector<Predator> preds;
    preds.push_back(pd);

    auto it = flock.begin() + 1;
    auto et = flock.end() - 1;

    auto dv2 = flock.vel_correction(it, preds, 2., 2);
    auto dv4 = flock.vel_correction(et, preds, 2., 2.);
    CHECK(dv2[0] == 3.5);
    CHECK(dv2[1] == -13);
    CHECK(dv4[0] == 10);
    CHECK(dv4[1] == 12);
  }

  SUBCASE(
      "Testing the Flock::vel_correction method with four boids and Predator "
      "out of range for three of them") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);  // pd out of range
    Boid bd_2(3, 3, 5, 9, 120., 1920, 1080, 4, 1);  // pd out of range
    Boid bd_3(4, 4, 5, 0, 120., 1920, 1080, 4, 1);  // pd out of range
    Boid bd_4(6, 7, 5, 0, 120., 1920, 1080, 4, 1);  // pd NOT out of range

    Predator pd(8., 14., -5., -5., 140., 1920., 1080., 4., 1., 5., 1.);

    Parameters params(4, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});
    std::vector<Predator> preds;
    preds.push_back(pd);

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    const auto it = flock.begin() + 1;
    const auto ut = flock.end() - 1;
    const auto dv1 = flock.vel_correction(it, preds, 2., 2.);
    const auto dv4 = flock.vel_correction(ut, preds, 2., 2.);
    CHECK(dv1[0] == -0.5);  // vel_correction without predator
    CHECK(dv1[1] == -17.);  // vel_correction without predator
    CHECK(dv4[0] == -4);    // vel_correction with predator
    CHECK(dv4[1] == -14.);  // vel_correction with predator
  }

  SUBCASE(
      "Testing the Flock::vel_correction method with one boid and two "
      "predators, one of which "
      "out of range") {
    Boid bd_1(1, 4, 5, 0, 120., 1920, 1080, 4, 1);

    Predator pd1(4., 8., -5., -5., 140., 1920., 1080., 4., 1., 5., 1.);
    Predator pd2(10., 8., -5., -5., 140., 1920., 1080., 4., 1., 5., 1.);

    Parameters params(4, 4, 1, 2, 3);

    Flock flock(params, 0, 120., {1920, 1080});
    std::vector<Predator> preds;
    preds.push_back(pd1);
    preds.push_back(pd2);

    flock.push_back(bd_1);

    const auto it = flock.begin();
    const auto dv1 = flock.vel_correction(it, preds, 2., 2.);
    CHECK(dv1[0] == -6);
    CHECK(dv1[1] == -8.);
  }
}

TEST_CASE("Testing the Flock::avoid_pred method") {}

TEST_CASE("Testing the Flock::Update_stats method") {
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // PARAMS are f_params.d, f_params.d_s, f_params.s, f_params.a, f_params.c

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