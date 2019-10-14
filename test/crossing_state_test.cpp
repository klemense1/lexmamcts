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
ObjectiveVec MctsParameters::LOWER_BOUND = Eigen::Vector4f(-1000.0f, -1000.0f, -100.0f, -1000.0f);
ObjectiveVec MctsParameters::UPPER_BOUND = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);

double MctsParameters::DISCOUNT_FACTOR = 0.9;
double MctsParameters::EXPLORATION_CONSTANT = 1.0;

double MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC = 1;
double MctsParameters::MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC = 1000;

TEST(CrossingTest, general) {
  RandomGenerator::random_generator_ = std::mt19937(1000);
  Mcts<CrossingState, UctStatistic, UctStatistic, RandomHeuristic> mcts;

  // SETUP RULES
  std::vector<EvaluatorRuleLTL> automata;
  // Finally arrive at goal (Liveness)
  automata.emplace_back("F ego_goal_reached", -100.f, RewardPriority::GOAL);
  // Do not collide with others (Safety)
  automata.emplace_back("G !collision", -1000.f, RewardPriority::SAFETY);
  // Arrive before others (Guarantee)
  // Currently not possible because ego can't drive faster than others
  // TODO: Add more actions for ego
  //automata.emplace_back("!other_goal_reached U ego_goal_reached", -1000.f, RewardPriority::GOAL);


  auto state = std::make_shared<CrossingState>(automata);

  std::vector<Reward> rewards(1, Reward::Zero());
  JointAction jt(2, (int) Actions::FORWARD);
  std::vector<int> pos_history;

  pos_history.emplace_back(state->get_ego_pos());
  while (!state->is_terminal()) {
    mcts.search(*state, 50000, 10000);
    jt[0] = mcts.returnBestAction();
    state = state->execute(jt, rewards);
    std::cout << rewards[0] << std::endl;
    pos_history.emplace_back(state->get_ego_pos());
  }

  std::cout << "Ego positions:" << std::endl;
  for (auto p : pos_history) {
    std::cout << p << ", ";
  }

  EXPECT_TRUE(state->ego_goal_reached());
}

TEST(CrossingTest, LexicographicOrder) {
  std::vector<Eigen::Vector2i> v;
  v.emplace_back(1, 2);
  v.emplace_back(2, 1);
  v.emplace_back(2, 3);
  v.emplace_back(2, 2);
  v.emplace_back(-5, 3);
  auto max = std::max_element(v.begin(), v.end(),
                              [](Eigen::Vector2i &a,
                                 Eigen::Vector2i &b) -> bool {
                                return std::lexicographical_compare(a.begin(),
                                                                    a.end(),
                                                                    b.begin(),
                                                                    b.end());
                              });
  ASSERT_EQ(*max, Eigen::Vector2i(2, 3));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}