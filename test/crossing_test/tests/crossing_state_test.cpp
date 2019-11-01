//
// Created by luis on 10.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO

#include "gtest/gtest.h"
#include "test/crossing_test/common.hpp"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/random_generator.h"

#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/evaluator_label_collision.hpp"
#include "test/crossing_test/evaluator_label_goal_reached.hpp"
#include "test/crossing_test/evaluator_label_hold_at_xing.hpp"
#include "test/crossing_test/evaluator_label_other_near.hpp"
#include "test/crossing_test/crossing_state_episode_runner.h"
#include "test/crossing_test/evaluator_label_speed.hpp"

class CrossingTestF : protected CrossingTest, public ::testing::Test {
 public:
  CrossingTestF() : CrossingTest() {};
};

std::vector<Reward> get_optimal_reward(std::shared_ptr<CrossingState> state) {
  JointAction jt(state->get_agent_idx().size(), aconv(Actions::FORWARD));
  std::vector<Reward> rewards(state->get_agent_idx().size(), Reward::Zero());
  std::vector<Reward> accu_reward(state->get_agent_idx().size(), Reward::Zero());

  jt[state->ego_agent_idx] = aconv(Actions::WAIT);
  for (int i = 0; i < 2; ++i) {
    state = state->execute(jt, rewards);
    accu_reward += rewards;
    LOG(INFO) << state->sprintf();
  }

  jt[state->ego_agent_idx] = aconv(Actions::FORWARD);
  for (; !state->is_terminal(); state = state->execute(jt, rewards)) {
    LOG(INFO) << state->sprintf();
    accu_reward += rewards;
  }

  accu_reward += state->get_final_reward();
  LOG(INFO) << "Optimal rewards:";
  LOG(INFO) << "Ego:" << accu_reward.at(0).transpose();
  for (size_t otr_idx = 1; otr_idx < state->num_other_agents + 1; ++otr_idx) {
    LOG(INFO) << "Agent " << otr_idx << ":" << accu_reward.at(otr_idx).transpose();
  }
  return accu_reward;
}

TEST_F(CrossingTestF, general) {
  const int MAX_STEPS = 40;
  int steps = 0;
  std::vector<Reward> optimal_reward = get_optimal_reward(state->clone());
  std::vector<Reward> accu_reward(state->get_agent_idx().size(), Reward::Zero());
  pos_history.emplace_back(state->get_ego_pos());
  pos_history_other.emplace_back(state->get_agent_states()[1].x_pos);
  while (!state->is_terminal() && steps < MAX_STEPS) {
    mcts.search(*state, 50000, 1000);
    //jt[0] = mcts.returnBestAction()[0];
    jt = mcts.returnBestAction();
    LOG(INFO) << "Performing action:" << jt;
    state = state->execute(jt, rewards);
    accu_reward += rewards;
    state->reset_depth();
    pos_history.emplace_back(state->get_ego_pos());
    pos_history_other.emplace_back(state->get_agent_states()[1].x_pos);
    ++steps;
  }
  LOG(INFO) << "Accumulated rewards:";
  LOG(INFO) << "Ego:" << accu_reward.at(0).transpose();
  for (size_t otr_idx = 1; otr_idx < state->num_other_agents + 1; ++otr_idx) {
    LOG(INFO) << "Agent " << otr_idx << ":" << accu_reward.at(otr_idx).transpose();
  }
  //Should not hit the maximum # of steps
  EXPECT_LT(steps, MAX_STEPS);
  EXPECT_TRUE(state->ego_goal_reached());
  std::vector<bool> result;
  std::transform(accu_reward.begin(),
                 accu_reward.end(),
                 optimal_reward.begin(),
                 std::back_inserter(result),
                 [](Reward const &a, Reward const &b) {
                   return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
                 });
  EXPECT_TRUE(std::all_of(result.begin(), result.end(), [](bool i) { return i; }));
}

TEST_F(CrossingTestF, belief) {
  const int MAX_STEPS = 40;
  int steps = 0;
  JointAction other_jt(2);

  label_evaluators.emplace_back(std::make_shared<EvaluatorLabelSpeed>("speeding"));
  automata[1].emplace_back("G !speeding", -20.0f, RewardPriority::SAFETY, 0.9);
  std::shared_ptr<CrossingState> ego_state = std::make_shared<CrossingState>(automata, label_evaluators);

  pos_history.emplace_back(ego_state->get_ego_pos());
  pos_history_other.emplace_back(state->get_ego_pos());

  while (!state->is_terminal() && steps < MAX_STEPS) {

    // Ego search
    mcts.search(*ego_state, 50000, 1000);
    jt[0] = mcts.returnBestAction()[0];
    other_jt[1] = jt[0];

    // Other search
    mcts.search(*state, 50000, 1000);
    jt[1] = mcts.returnBestAction()[0]; //Note that other is now in the ego perspective
    other_jt[0] = jt[1];

    LOG(INFO) << "Performing action:" << jt;
    ego_state = ego_state->execute(jt, rewards);

    state = state->execute(other_jt, rewards);

    ego_state->update_rule_belief();
    ego_state->reset_violations();

    pos_history.emplace_back(ego_state->get_ego_pos());
    pos_history_other.emplace_back(state->get_ego_pos());
    ++steps;
  }
  //Should not hit the maximum # of steps
  EXPECT_LT(steps, MAX_STEPS);
  EXPECT_TRUE(state->ego_goal_reached());
}

TEST_F(CrossingTestF, giveWay) {
  std::vector<AgentState> agent_states(2);
  agent_states[0].x_pos = 7;
  agent_states[0].last_action = Actions::FORWARD;
  agent_states[1].x_pos = 8;
  agent_states[1].last_action = Actions::FORWARD;
  automata[0].emplace_back("G((at_hp_xing & other_near) -> (X at_hp_xing))", -500.0f, RewardPriority::SAFETY);
  state = std::make_shared<CrossingState>(agent_states, false, automata, label_evaluators);
  state = state->execute(jt, rewards);
  state = state->execute(jt, rewards);
  state = state->execute(jt, rewards);
  state = state->execute(jt, rewards);
  ASSERT_EQ(rewards[0](0), -500);
  state->execute(jt, rewards);
}

TEST_F(CrossingTestF, LexicographicOrder) {
  std::vector<Eigen::Vector2i> v;
  v.emplace_back(1, 2);
  v.emplace_back(2, 1);
  v.emplace_back(2, 3);
  v.emplace_back(2, 2);
  v.emplace_back(-5, 3);
  auto max = std::max_element(v.begin(), v.end(), [](Eigen::Vector2i &a, Eigen::Vector2i &b) -> bool {
    return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
  });
  ASSERT_EQ(*max, Eigen::Vector2i(2, 3));
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = "/tmp/log";
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = 1;
  ::testing::GTEST_FLAG(filter) = "CrossingTestF.general";
  return RUN_ALL_TESTS();
}
