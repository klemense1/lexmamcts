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

class CrossingTestF : protected CrossingTest, public ::testing::Test {
 public:
    CrossingTestF() : CrossingTest() {};
};

TEST_F(CrossingTestF, general) {
    const int MAX_STEPS = 40;
    int steps = 0;
    pos_history.emplace_back(state->get_ego_pos());
    while (!state->is_terminal() && steps < MAX_STEPS) {
        mcts.search(*state,50000, 1000);
        //jt[0] = mcts.returnBestAction()[0];
        jt = mcts.returnBestAction();
        LOG(INFO) << "Performing action:" << jt;
        state = state->execute(jt, rewards);
        pos_history.emplace_back(state->get_ego_pos());
        ++steps;
    }

    LOG(INFO) << "Ego positions:" << pos_history;

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
    state = std::make_shared<CrossingState>(agent_states,
                                            false,
                                            automata,
                                            label_evaluators);
    state = state->execute(jt, rewards);
    state = state->execute(jt, rewards);
    state = state->execute(jt, rewards);
    state = state->execute(jt, rewards);
    ASSERT_EQ(rewards[0](0),-500);
    state->execute(jt, rewards);
}

TEST_F(CrossingTestF, LexicographicOrder) {
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
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = "/tmp/log";
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_alsologtostderr = 1;
  ::testing::GTEST_FLAG(filter) = "CrossingTestF.general";
  return RUN_ALL_TESTS();
}