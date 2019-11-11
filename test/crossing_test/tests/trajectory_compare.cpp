//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <vector>
#include <iostream>

#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/common.h"
#include "mcts/statistics/e_greedy_uct_statistic.h"
#include "mcts/statistics/max_uct_statistic.h"
#include "mcts/heuristics/semi_random_heuristic.h"
#include "mcts/statistics/thres_uct_statistic.h"

using std::vector;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;
using mcts::JointReward;

enum StatStrategy {
  UCT = 0, SLACK, SCALARIZATION, MAX, HEURISTIC, THRESHOLD, NUM
};

template<class T>
void run_test(T &test_f, size_t num_iter) {
  const int MAX_STEPS = 40;
  int steps = 0;
  std::vector<Reward> step_reward(test_f.rewards);
  test_f.pos_history.emplace_back(test_f.state->get_ego_pos());
  test_f.pos_history_other.emplace_back(test_f.state->get_agent_states()[1].x_pos);
  while (!test_f.state->is_terminal() && steps < MAX_STEPS) {
    test_f.mcts.search(*test_f.state, 50000, num_iter);
    test_f.set_jt(test_f.mcts.returnBestAction());
    test_f.state = test_f.state->execute(test_f.get_jt(), step_reward);
    test_f.rewards += step_reward;
    //test_f.state->reset_depth();
    test_f.pos_history.emplace_back(test_f.state->get_ego_pos());
    test_f.pos_history_other.emplace_back(test_f.state->get_agent_states()[1].x_pos);
    ++steps;
  }
  test_f.rewards += test_f.state->get_final_reward();
  LOG(INFO) << "Ego history:" << test_f.pos_history;
  LOG(INFO) << "Otr history:" << test_f.pos_history_other;
}

CrossingStateParameter setup_scalarization() {
  CrossingStateParameter p = make_default_crossing_state_parameters();
  p = make_default_crossing_state_parameters();
  p.depth_prio = 0;
  p.speed_deviation_prio = 0;
  p.depth_weight = 100;
  p.speed_deviation_weight = 100;
  return p;
}
template<class T>
JointReward calculate_default_reward(T &test_f) {
  JointReward step_reward(test_f.rewards.size(), Reward::Zero());
  JointReward accu_reward(test_f.rewards.size(), Reward::Zero());
  auto const &actions = test_f.get_action_history();
  CrossingTestEnv<> d_test_env;
  for (auto const &jt : actions) {
    d_test_env.state = d_test_env.state->execute(jt, step_reward);
    accu_reward += step_reward;
  }
  accu_reward += d_test_env.state->get_final_reward();
  DVLOG(2) << "Default reward: " << accu_reward;
  return accu_reward;
}

template<class T, class U>
double calculate_metric(T &test_f, U &optimal) {
  double u = 0.0;
  double shift = 1.0;
  Reward candidate = rewards_to_mat(test_f.rewards).rowwise().sum();
  // Create an approximate utility function of the lexicographical ordering
  // by shifting higher priority rewards to the left
  for (int d = candidate.rows() - 1; d >= 0; --d) {
    u += shift * candidate(d);
    shift *= std::abs(
        test_f.mcts_parameters_.uct_statistic.UPPER_BOUND(d) - test_f.mcts_parameters_.uct_statistic.LOWER_BOUND(d))
        + 1;
  }
  return u;
}

void write_plot_output(ostream &os, double value) {
  os << value << "\t";
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  //FLAGS_minloglevel = 1;
  //  FLAGS_v = 1;
  FLAGS_logtostderr = true;
  CrossingTestEnv<> optimal;
  int const n = 1;

  ofstream ofs;
  ofs.open("/tmp/trajectory_comp.dat");
  ofs << "# Iterations\tUCT\tSlack\tScalarization\tMaximum Selection\tImproved heuristic\tThresholded selection\n";

  optimal.rewards = get_optimal_reward(optimal.state);

  ArrayXi sample_sizes = ArrayXi::LinSpaced(1, 10000, 10000);
  int step = 1;
  for (int i : sample_sizes) {
    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    vector<double> avg(StatStrategy::NUM, 0.0);

    for (int j = 0; j < n; ++j) {
      CrossingTestEnv<UctStatistic<>> uct_env;
      CrossingTestEnv<SlackUCTStatistic> slack_env;
      CrossingTestEnv<SlackUCTStatistic> scalarization_env(make_default_mcts_parameters(), setup_scalarization());
      CrossingTestEnv<MaxUCTStatistic> max_uct_env;
      CrossingTestEnv<SlackUCTStatistic, SemiRandomHeuristic> heuristic_env;
      CrossingTestEnv<ThresUCTStatistic> threshold_env;

      LOG(INFO) << "UCT:";
      run_test(uct_env, i);
      LOG(INFO) << "Slack:";
      run_test(slack_env, i);
      LOG(INFO) << "Scalarization:";
      run_test(scalarization_env, i);
      // To compare with others, need to recreate the reward with the same setting as others,
      // since we changed the way, the reward is computed
      scalarization_env.rewards = calculate_default_reward(scalarization_env);

      LOG(INFO) << "Max UCT:";
      run_test(max_uct_env, i);
      LOG(INFO) << "Heuristic:";
      run_test(heuristic_env, i);
      LOG(INFO) << "Threshold:";
      run_test(threshold_env, i);

      avg[StatStrategy::UCT] += calculate_metric(uct_env, optimal);
      avg[StatStrategy::SLACK] += calculate_metric(slack_env, optimal);
      avg[StatStrategy::SCALARIZATION] += calculate_metric(scalarization_env, optimal);
      avg[StatStrategy::MAX] += calculate_metric(max_uct_env, optimal);
      avg[StatStrategy::HEURISTIC] += calculate_metric(heuristic_env, optimal);
      avg[StatStrategy::THRESHOLD] += calculate_metric(threshold_env, optimal);
    }

    ofs << i << "\t";
    for (auto &iter : avg) {
      iter = iter / static_cast<double>(n);
      write_plot_output(ofs, iter);
    }
    ofs << "\n";

    LOG(WARNING) << "Metrics:";

    LOG(WARNING) << "Optimal:";
    LOG(WARNING) << optimal.rewards;

    LOG(WARNING) << "UCT:";
    LOG(WARNING) << avg[StatStrategy::UCT];

    LOG(WARNING) << "Slack:";
    LOG(WARNING) << avg[StatStrategy::SLACK];

    LOG(WARNING) << "Scalarization:";
    LOG(WARNING) << avg[StatStrategy::SCALARIZATION];

    LOG(WARNING) << "Max:";
    LOG(WARNING) << avg[StatStrategy::MAX];

    LOG(WARNING) << "Heuristic:";
    LOG(WARNING) << avg[StatStrategy::HEURISTIC];

    LOG(WARNING) << "Threshold:";
    LOG(WARNING) << avg[StatStrategy::THRESHOLD];
    ++step;
  }
  ofs.close();
  return 0;
}

