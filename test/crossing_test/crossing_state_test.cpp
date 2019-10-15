//
// Created by luis on 10.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO

#include "gtest/gtest.h"
#include "common.hpp"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/random_generator.h"

#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/evaluator_label_collision.hpp"
#include "test/crossing_test/evaluator_label_goal_reached.hpp"
#include "test/crossing_test/evaluator_label_hold_at_xing.hpp"
#include "test/crossing_test/evaluator_label_other_near.hpp"

std::mt19937  mcts::RandomGenerator::random_generator_;
ObjectiveVec MctsParameters::LOWER_BOUND = Eigen::Vector4f(-1000.0f, -1000.0f, -100.0f, -1000.0f);
ObjectiveVec MctsParameters::UPPER_BOUND = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);

double MctsParameters::DISCOUNT_FACTOR = 0.9;
double MctsParameters::EXPLORATION_CONSTANT = 1.0;

double MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC = 1;
double MctsParameters::MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC = 1000;

class CrossingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    RandomGenerator::random_generator_ = std::mt19937(1000);


    // SETUP LABEL EVALUATORS
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision",
                                                                            CrossingState::crossing_point));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelGoalReached>("ego_goal_reached",
                                                                              CrossingState::ego_goal_reached_position));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelHoldAtXing>("at_hp_xing",
                                                                             CrossingState::crossing_point));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
    // SETUP RULES
    // Finally arrive at goal (Liveness)
    automata.emplace_back("F ego_goal_reached", -100.f, RewardPriority::GOAL);
    // Do not collide with others (Safety)
    automata.emplace_back("G !collision", -1000.f, RewardPriority::SAFETY);
    // Arrive before others (Guarantee)
    // Currently not possible because ego can't drive faster than others
    // TODO: Add more actions for ego
    //automata.emplace_back("!other_goal_reached U ego_goal_reached", -1000.f, RewardPriority::GOAL);
    automata.emplace_back("G((at_hp_xing & other_near) -> (X at_hp_xing))", -500.0f, RewardPriority::SAFETY);

    state = std::make_shared<CrossingState>(automata, label_evaluators);
    rewards = std::vector<Reward>(1, Reward::Zero());
    jt = JointAction(2, (int) Actions::FORWARD);
  }
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
  std::vector<EvaluatorRuleLTL> automata;
  std::vector<Reward> rewards;
  JointAction jt;
  std::vector<int> pos_history;
  Mcts<CrossingState, UctStatistic, UctStatistic, RandomHeuristic> mcts;
  std::shared_ptr<CrossingState> state;
};

TEST_F(CrossingTest, general) {
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

TEST_F(CrossingTest, giveWay) {
  std::vector<AgentState> other_agent_states(1);
  other_agent_states[0].x_pos = 8;
  other_agent_states[0].last_action = Actions::FORWARD;
  state = std::make_shared<CrossingState>(other_agent_states,
                                          AgentState(7, Actions::FORWARD),
                                          false,
                                          automata,
                                          label_evaluators);
  state = state->execute(jt, rewards);
  state = state->execute(jt, rewards);
  state = state->execute(jt, rewards);
  state = state->execute(jt, rewards);
  ASSERT_EQ(rewards[0](0), -500);
  state->execute(jt, rewards);
}

TEST_F(CrossingTest, LexicographicOrder) {
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