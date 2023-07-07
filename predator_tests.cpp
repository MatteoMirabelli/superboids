#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

TEST_CASE("Testing the Predator::Predate method") {
  // PREDATOR CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s, p_range, p_hunger

  SUBCASE("Testing the Predator::Predate method with two preys") {
    Predator pr(2., 2., 5., 5., 50., 10., 2., 1920., 1080., 5., 2.);

    Boid bd1(5., 3., 1., 1., 120., 1920., 1080., 10., 2.);
    Boid bd2(3., 6., 1., 1., 120., 1920., 1080., 10., 2.);

    std::vector<Boid> preys{bd1, bd2};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == 16);
    CHECK(vel_correction[1] == 9);
  }

  SUBCASE("Testing the Predator::Predate method with two preys") {
    Predator pr({154., 112.}, {-1., -1.}, 120., 10., 2., {1920., 1080.}, 10.,
                2.);

    Boid bd1({156., 108.}, {1., -2.}, 120., {1920., 1080.}, 5., 4.);
    Boid bd2({148., 113.}, {2., -1}, 120., {1920., 1080.}, 5., 4.);

    std::vector<Boid> preys{bd1, bd2};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == 4);
    CHECK(vel_correction[1] == -19);
  }

  SUBCASE("Testing the Predator::Predate method with two preys") {
    Predator pr({152., 114.}, {-1., -1.}, 120., 10., 2., {1920., 1080.}, 10.,
                2.);

    Boid bd1({156., 108.}, {1., -2.}, 120., {1920., 1080.}, 5., 4.);
    Boid bd2({148., 113.}, {2., -1}, 120., {1920., 1080.}, 5., 4.);

    std::vector<Boid> preys{bd1, bd2};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == -16);
    CHECK(vel_correction[1] == -11);
  }

  SUBCASE("Testing the Predator::Predate method with no preys") {
    Predator pr(2., 2., 5., 5., 50., 10., 2., 1920., 1080., 5., 2.);

    std::vector<Boid> preys{};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == 0);
    CHECK(vel_correction[1] == 0);
  }
}

TEST_CASE("Testing the get_vector_neighbours") {
  Predator pd1({154., 112.}, {-1., -1.}, 120., 10., 2., {1920., 1080.}, 10.,
               2.);
  Predator pd2({152., 114.}, {-1., -1.}, 120., 10., 2., {1920., 1080.}, 10.,
               2.);

  std::vector<Predator> preds;
  preds.push_back(pd1);
  preds.push_back(pd2);
  auto it = preds.begin();
  auto et = it + 1;
  std::vector<Predator> neighbours1 = get_vector_neighbours(preds, it, 10.);
  ++it;
  std::vector<Predator> neighbours2 = get_vector_neighbours(preds, it, 10.);

  CHECK(neighbours1.size() == 1);
  CHECK(neighbours1[0].get_pos()[0] == 152.);
  CHECK(neighbours1[0].get_pos()[1] == 114.);
  CHECK(neighbours2.size() == 1);
  CHECK(neighbours2[0].get_pos()[0] == 154.);
  CHECK(neighbours2[0].get_pos()[1] == 112.);
}

TEST_CASE("Testing the update_predator_state with two predators") {
  // PREDATOR CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle,  param_ds_,
  // param_s, window_space{1920, 1080}, p_range, p_hunger
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // OBSTACLE CONSTRUCTOR takes: pos{x,y}, size;
  // update_predator_states takes: predators, delta_t, bhv, preys, obstacles,
  // pred_pred_repulsion, pred_obs_detection, pred_obstacle_separation,
  // pred_brd_detection, pred_brd_repulsion

  Predator pd1({154., 112.}, {-1., -1.}, 120., 20., 2., {1920., 1080.}, 20.,
               2.);
  Predator pd2({152., 114.}, {-1., -1.}, 120., 20., 2., {1920., 1080.}, 20.,
               2.);

  Boid bd1({156., 108.}, {1., -2.}, 120., {1920., 1080.}, 5., 4.);
  Boid bd2({148., 113.}, {2., -1}, 120., {1920., 1080.}, 5., 4.);

  Obstacle ob1({145., 110.}, 1);
  Obstacle ob2({150., 105.}, 1.);

  std::vector<Predator> preds;
  preds.push_back(pd1);
  preds.push_back(pd2);
  std::pair<Boid, int> pair1({bd1, 0});
  std::pair<Boid, int> pair2({bd1, 1});
  std::pair<Boid, int> pair3({bd2, 0});
  std::pair<Boid, int> pair4({bd2, 1});
  std::vector<std::pair<Boid, int>> preys;
  preys.push_back(pair1);
  preys.push_back(pair2);
  preys.push_back(pair3);
  preys.push_back(pair4);

  std::vector<Obstacle> g_obstacles;
  g_obstacles.push_back(ob1);
  g_obstacles.push_back(ob2);

  update_predators_state(preds, 1., true, preys, g_obstacles, 4., 2., 1., 1.,
                         1.);

  CHECK(preds[0].get_pos()[0] == doctest::Approx(120.818));
  CHECK(preds[0].get_pos()[1] == doctest::Approx(119.021));
  CHECK(preds[0].get_vel()[0] == doctest::Approx(-69.1096));
  CHECK(preds[0].get_vel()[1] == doctest::Approx(11.1291));

  CHECK(preds[1].get_pos()[0] == doctest::Approx(174.021));
  CHECK(preds[1].get_pos()[1] == doctest::Approx(77.8183));
  CHECK(preds[1].get_vel()[0] == doctest::Approx(35.3791));
  CHECK(preds[1].get_vel()[1] == doctest::Approx(-60.4013));
}

TEST_CASE("Testing the random_predators generation") {
  // PREDATOR CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s, p_range, p_hunger

  // RANDOM_PREDATORS takes: obs, number of predators, window_space{1920, 1080},
  // view_angle, param_ds, param_s. p_range, p_hunger

  SUBCASE("Testing the random_predators generation with two predators") {
    std::vector<Obstacle> obs;
    Obstacle ob1({150., 700.}, 60.);
    Obstacle ob2({450., 400.}, 70.);
    obs.push_back(ob1);
    obs.push_back(ob2);

    auto preds =
        random_predators(obs, 3., {1920., 1080.}, 130., 5., 3., 10., 5.);

    CHECK(preds.size() == 3);

    bool overlap_pd1 =
        (preds[0].get_pos()[0] < 20. || preds[0].get_pos()[0] > 1900 ||
         preds[0].get_pos()[1] < 20. || preds[0].get_pos()[1] > 1060 ||
         vec_norm<double>(preds[0].get_pos() - ob1.get_pos()) < 63 ||
         vec_norm<double>(preds[0].get_pos() - ob2.get_pos()) < 73);

    bool overlap_pd2 =
        (preds[1].get_pos()[0] < 20. || preds[1].get_pos()[0] > 1900 ||
         preds[1].get_pos()[1] < 20. || preds[1].get_pos()[1] > 1060 ||
         vec_norm<double>(preds[1].get_pos() - ob1.get_pos()) < 63 ||
         vec_norm<double>(preds[1].get_pos() - ob2.get_pos()) < 73);

    bool overlap_pd3 =
        (preds[2].get_pos()[0] < 20. || preds[2].get_pos()[0] > 1900 ||
         preds[2].get_pos()[1] < 20. || preds[2].get_pos()[1] > 1060 ||
         vec_norm<double>(preds[2].get_pos() - ob1.get_pos()) < 63 ||
         vec_norm<double>(preds[2].get_pos() - ob2.get_pos()) < 73);

    CHECK(overlap_pd1 == false);
    CHECK(overlap_pd1 == false);
    CHECK(overlap_pd2 == false);

    CHECK(preds[0].get_view_angle() == 130.);
    CHECK(preds[0].get_par_ds() == 5.);
    CHECK(preds[0].get_par_s() == 3.);
    CHECK(preds[0].get_range() == 10.);
    CHECK(preds[0].get_hunger() == 5.);

    CHECK(preds[1].get_view_angle() == 130.);
    CHECK(preds[1].get_par_ds() == 5.);
    CHECK(preds[1].get_par_s() == 3.);
    CHECK(preds[1].get_range() == 10.);
    CHECK(preds[1].get_hunger() == 5.);

    CHECK(preds[2].get_view_angle() == 130.);
    CHECK(preds[2].get_par_ds() == 5.);
    CHECK(preds[2].get_par_s() == 3.);
    CHECK(preds[2].get_range() == 10.);
    CHECK(preds[2].get_hunger() == 5.);
  }

  SUBCASE(
      "Testing the random_predators generation with three predators and no "
      "obstacles") {
    std::vector<Obstacle> obs;
    auto preds =
        random_predators(obs, 3., {1920., 1080.}, 130., 5., 3., 10., 5.);

    CHECK(preds.size() == 3);

    bool out_of_borders_pd1 =
        (preds[0].get_pos()[0] < 20. || preds[0].get_pos()[0] > 1900 ||
         preds[0].get_pos()[1] < 20. || preds[0].get_pos()[1] > 1060);

    bool out_of_borders_pd2 =
        (preds[1].get_pos()[0] < 20. || preds[1].get_pos()[0] > 1900 ||
         preds[1].get_pos()[1] < 20. || preds[1].get_pos()[1] > 1060);

    bool out_of_borders_pd3 =
        (preds[2].get_pos()[0] < 20. || preds[2].get_pos()[0] > 1900 ||
         preds[2].get_pos()[1] < 20. || preds[2].get_pos()[1] > 1060);

    CHECK(out_of_borders_pd1 == false);
    CHECK(out_of_borders_pd2 == false);
    CHECK(out_of_borders_pd3 == false);

    CHECK(preds[0].get_view_angle() == 130.);
    CHECK(preds[0].get_par_ds() == 5.);
    CHECK(preds[0].get_par_s() == 3.);
    CHECK(preds[0].get_range() == 10.);
    CHECK(preds[0].get_hunger() == 5.);

    CHECK(preds[1].get_view_angle() == 130.);
    CHECK(preds[1].get_par_ds() == 5.);
    CHECK(preds[1].get_par_s() == 3.);
    CHECK(preds[1].get_range() == 10.);
    CHECK(preds[1].get_hunger() == 5.);

    CHECK(preds[2].get_view_angle() == 130.);
    CHECK(preds[2].get_par_ds() == 5.);
    CHECK(preds[2].get_par_s() == 3.);
    CHECK(preds[2].get_range() == 10.);
    CHECK(preds[2].get_hunger() == 5.);
  }

  SUBCASE("Testing the random_predators generation with zero predators") {
    std::vector<Obstacle> obs1;
    std::vector<Obstacle> obs2;

    auto preds_with_obstacles_1 =
        random_predators(obs1, 0., {1920., 1080.}, 130., 5., 3., 10., 5.);

    Obstacle ob1({150., 700.}, 60.);
    Obstacle ob2({450., 400.}, 70.);
    obs2.push_back(ob1);
    obs2.push_back(ob2);

    auto preds_with_obstacles_2 =
        random_predators(obs2, 0., {1920., 1080.}, 130., 5., 3., 10., 5.);

    CHECK(preds_with_obstacles_1.size() == 0);
    CHECK(preds_with_obstacles_2.size() == 0);
  }
}