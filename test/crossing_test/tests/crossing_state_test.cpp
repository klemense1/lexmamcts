//
// Created by luis on 10.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO

#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "test/crossing_test/common.hpp"
#include "mcts/mcts.h"

#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/common.h"
#include "mcts/statistics/thres_uct_statistic.h"

typedef ThresUCTStatistic Stat;
typedef RandomHeuristic HeuristicType;

class CrossingTestF : public CrossingTestEnv<Stat, HeuristicType>, public ::testing::Test {
 public:
  CrossingTestF() : CrossingTestEnv<Stat, HeuristicType>() {};
};

TEST(CrossingTest, general) {
  CrossingStateParameter p = make_default_crossing_state_parameters();
  p.depth_prio = static_cast<int>(RewardPriority::GOAL);
  p.speed_deviation_prio = static_cast<int>(RewardPriority::GOAL);
  p.acceleration_prio = static_cast<int>(RewardPriority::GOAL);
  p.potential_prio = static_cast<int>(RewardPriority::GOAL);
  p.depth_weight = 0;
  p.speed_deviation_weight = 0;
  p.acceleration_weight = 0;
  p.potential_weight = 1;
  CrossingTestEnv<> test_env(make_default_mcts_parameters(),
                             p,
                             BaseTestEnv::make_default_automata(p.num_other_agents + 1),
                             BaseTestEnv::make_default_labels(p));
  const int MAX_STEPS = 40;
  int steps = 0;
  CrossingTestEnv<> optimal_env(test_env);
  get_optimal_reward(&optimal_env);
  std::vector<Reward> optimal_reward = optimal_env.rewards;
  std::vector<Reward> accu_reward(test_env.state->get_agent_idx().size(), Reward::Zero());
  test_env.pos_history.emplace_back(test_env.state->get_ego_pos());
  test_env.pos_history_other.emplace_back(test_env.state->get_agent_states()[1].x_pos);
  while (!test_env.state->is_terminal() && steps < MAX_STEPS) {
    test_env.mcts.search(*test_env.state, 50000, 10000);
    JointAction jt = test_env.mcts.returnBestAction();
    test_env.set_jt(jt);
    LOG(INFO) << "Performing action:" << test_env.get_jt();
    test_env.state = test_env.state->execute(test_env.get_jt(), test_env.rewards);
    accu_reward += test_env.rewards;
    test_env.pos_history.emplace_back(test_env.state->get_ego_pos());
    test_env.pos_history_other.emplace_back(test_env.state->get_agent_states()[1].x_pos);
    ++steps;
  }
  accu_reward += test_env.state->get_final_reward();
  LOG(INFO) << "Accumulated rewards:";
  LOG(INFO) << "Ego:" << accu_reward.at(0).transpose();
  for (size_t otr_idx = 1; otr_idx < test_env.state->get_agent_idx().size(); ++otr_idx) {
    LOG(INFO) << "Agent " << otr_idx << ":" << accu_reward.at(otr_idx).transpose();
  }
  //Should not hit the maximum # of steps
  EXPECT_LT(steps, MAX_STEPS);
  EXPECT_TRUE(test_env.state->ego_goal_reached());
  Reward row_sum_accu = rewards_to_mat(accu_reward).rowwise().sum();
  Reward row_sum_opti = rewards_to_mat(optimal_reward).rowwise().sum();
  // We should not be better than our constructed optimum
  // Plausibility check for weights
  EXPECT_TRUE(
      std::lexicographical_compare(row_sum_accu.begin(), row_sum_accu.end(), row_sum_opti.begin(), row_sum_opti.end())
          || row_sum_accu == row_sum_opti);
}

TEST_F(CrossingTestF, belief) {
  const int MAX_STEPS = 40;
  int steps = 0;
  JointAction other_jt(2);

  automata_[0].at(Rule::NO_SPEEDING) = EvaluatorRuleLTL("G !speeding", -1.0f, RewardPriority::LEGAL_RULE_B, 0.9);
  state = std::make_shared<CrossingState>(get_automata_vec(), label_evaluators_, crossing_state_parameter_);

  JointAction jt = get_jt();
  jt[1] = aconv(Actions::FASTFORWARD);
  set_jt(jt);

  auto succ_state = state->execute(jt, rewards);
  succ_state->update_rule_belief();
  succ_state->reset_violations();
  // TODO: Check for lowered belief
}

TEST_F(CrossingTestF, giveWay) {
  std::vector<AgentState> agent_states(2);
  agent_states[0].x_pos = 7;
  agent_states[0].last_action = Actions::FORWARD;
  agent_states[1].x_pos = 8;
  agent_states[1].last_action = Actions::FORWARD;
  automata_[0].at(Rule::GIVE_WAY) =
      EvaluatorRuleLTL("G((at_hp_xing & other_near) -> (X at_hp_xing))", -500.0f, RewardPriority::SAFETY);
  state = std::make_shared<CrossingState>(agent_states,
                                          false,
                                          get_automata_vec(),
                                          label_evaluators_,
                                          crossing_state_parameter_,
                                          0);
  state = state->execute(get_jt(), rewards);
  state = state->execute(get_jt(), rewards);
  state = state->execute(get_jt(), rewards);
  state = state->execute(get_jt(), rewards);
  ASSERT_EQ(rewards[0](0), -500);
  state->execute(get_jt(), rewards);
}

TEST(CrossingTest, LexicographicOrder) {
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
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
