//
// Created by luis on 10.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO
#define REWARD_DIM 4

#include "gtest/gtest.h"

#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/random_generator.h"
#include "test/crossing_state.hpp"

std::mt19937  mcts::RandomGenerator::random_generator_;
ObjectiveVec MctsParameters::LOWER_BOUND = Eigen::Vector4f(-1000.0f, -1000.0f, 0.0f, -1000.0f);
ObjectiveVec MctsParameters::UPPER_BOUND = Eigen::Vector4f(0.0f, 0.0f, 100.0f, 0.0f);

double MctsParameters::DISCOUNT_FACTOR = 0.9;
double MctsParameters::EXPLORATION_CONSTANT = 0.7;

double MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC = 1;
double MctsParameters::MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC = 1000;

TEST(CrossingTest, general) {
  RandomGenerator::random_generator_ = std::mt19937(1000);
  Mcts<CrossingState, UctStatistic, UctStatistic, RandomHeuristic> mcts;
  auto state = std::make_shared<CrossingState>();
  std::vector<Reward> rewards(1, Reward::Zero());
  JointAction jt(2, (int) Actions::FORWARD);
  //jt.resize(1);
  std::vector<int> pos_history;
  pos_history.clear();
  pos_history.emplace_back(state->get_ego_pos());
  while (!state->is_terminal()) {
    mcts.search(*state, 50000, 1000);
    jt[0] = mcts.returnBestAction();
    state = state->execute(jt, rewards);
    pos_history.emplace_back(state->get_ego_pos());
  }
  std::cout << "Positions:" << std::endl;
  for (auto p : pos_history) {
    std::cout << p << ", ";
  }
  std::cout << std::endl;
  std::cout << "END" << std::endl;
}