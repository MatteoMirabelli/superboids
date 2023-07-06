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

  SUBCASE("Testing the Predator::Predate method with no preys") {
    Predator pr(2., 2., 5., 5., 50., 10., 2., 1920., 1080., 5., 2.);

    std::vector<Boid> preys{};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == 0);
    CHECK(vel_correction[1] == 0);
  }
}

TEST_CASE("Testing the update_predator_state with two predators") {
  // PREDATOR CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle,  param_ds_,
  // param_s, window_space{1920, 1080}, p_range, p_hunger
  // BOID CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s
  // OBSTACLE CONSTRUCTOR takes: pos{x,y}, size;

  Predator pd1({156., 108.}, {-1., -1.}, 120., 10., 2., {1920., 1080.}, 10.,
               2.);
  Predator pd2({152., 114.}, {-1., -1.}, 120., 10., 2., {1920., 1080.}, 10.,
               2.);

  Boid bd1({156., 108.}, {1., -2.}, 120., {1920., 1080.}, 5., 4.);
  Boid bd2({148., 113.}, {2., -1}, 120., {1920., 1080.}, 5., 4.);

  Obstacle ob1({145., 110.}, 1);
  Obstacle ob2({150., 105.}, 1.);

  std::vector<Predator> preds;
  preds.push_back(pd1);
  preds.push_back(pd2);

  std::vector<std::pair<Boid, int>> preys{
      {bd1, 1}, {bd1, 2}, {bd2, 1}, {bd2, 2}};
  std::vector<Obstacle> g_obstacles;
  g_obstacles.push_back(ob1);
  g_obstacles.push_back(ob2);

  update_predators_state(preds, 1., true, preys, g_obstacles, 4., 1., 1., 1.,
                         1.);

  CHECK(pd1.get_pos()[0] == 201);
  CHECK(pd1.get_pos()[1] == 91);
  CHECK(pd1.get_vel()[0] == 45.);
  CHECK(pd1.get_vel()[1] == -18);

  CHECK(pd2.get_pos()[0] == 118);
  CHECK(pd2.get_pos()[1] == 158);
  CHECK(pd1.get_vel()[0] == -15);
  CHECK(pd1.get_vel()[1] == 44);
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
