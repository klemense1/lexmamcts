//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <vector>

#include "test/crossing_test/crossing_state_episode_runner.h"
#include "test/crossing_test/tests/common.h"

using std::vector;
using Eigen::MatrixXf;

template<class T>
void run_test(T &test_f, size_t num_iter) {
  const int MAX_STEPS = 40;
  int steps = 0;
  std::vector<Reward> step_reward(test_f.rewards);
  test_f.pos_history.emplace_back(test_f.state->get_ego_pos());
  test_f.pos_history_other.emplace_back(test_f.state->get_agent_states()[1].x_pos);
  while (!test_f.state->is_terminal() && steps < MAX_STEPS) {
    test_f.mcts.search(*test_f.state, 50000, num_iter);
    test_f.jt = test_f.mcts.returnBestAction();
    test_f.state = test_f.state->execute(test_f.jt, step_reward);
    test_f.rewards += step_reward;
    test_f.state->reset_depth();
    test_f.pos_history.emplace_back(test_f.state->get_ego_pos());
    test_f.pos_history_other.emplace_back(test_f.state->get_agent_states()[1].x_pos);
    ++steps;
  }
}

MatrixXf rewards_to_mat(vector<Reward> const &rewards) {
  MatrixXf mat(Reward::RowsAtCompileTime, rewards.size());
  for (size_t i = 0; i < rewards.size(); ++i) {
    mat.col(i) = rewards[i];
  }
  return mat;
}

void compare_result(vector<Reward> const &optimal, vector<Reward> const &candidate) {
  LOG(WARNING) << (rewards_to_mat(optimal) - rewards_to_mat(candidate)).cwiseAbs();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = "/tmp/log";
  FLAGS_alsologtostderr = 1;
  CrossingTest<UctStatistic<>> optimal;
  size_t num_agents = 2;

  vector<Reward> opti_reward(num_agents, Reward::Zero());
  opti_reward = get_optimal_reward(optimal.state);

  vector<size_t> const num_iter = {10, 100, 1000, 5000, 10000};
  for (size_t i = 0; i < num_iter.size(); ++i) {
    CrossingTest<UctStatistic<>> uct;
    CrossingTest<ParetoUCTStatistic> pareto;
    CrossingTest<SlackUCTStatistic> slack;

    run_test(uct, num_iter[i]);
    run_test(pareto, num_iter[i]);
    run_test(slack, num_iter[i]);

    LOG(WARNING) << "Absolute Errors:";

    LOG(WARNING) << "Optimal vs UCT:";
    compare_result(opti_reward, uct.rewards);

    LOG(WARNING) << "Optimal vs Slack:";
    compare_result(opti_reward, slack.rewards);

    LOG(WARNING) << "Optimal vs Pareto:";
    compare_result(opti_reward, pareto.rewards);
  }
  return 0;
}

