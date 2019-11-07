//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <vector>
#include <iostream>

#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/common.h"
#include "mcts/statistics/e_greedy_uct_statistic.h"

using std::vector;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;
typedef vector<Reward> JointReward;

enum StatStrategy {
  UCT = 0, PARETO, SLACK, EGREEDY, NUM
};

template<class T>
void run_test(T &test_f, size_t num_iter) {
  std::vector<Reward> step_reward(test_f.rewards);
  test_f.mcts.search(*test_f.state, 50000, num_iter);
  test_f.rewards = test_f.mcts.get_root()->get_value();
}

double calculate_metric(JointReward &candidate, JointReward &optimal) {
  return rewards_to_mat(candidate).cwiseAbs().sum();
}

void write_plot_output(ostream &os, JointReward &candidate, JointReward &optimal) {
  os << calculate_metric(candidate, optimal) << "\t";
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = 1;
  FLAGS_logtostderr = 1;
  CrossingTestEnv<UctStatistic<>> optimal;
  size_t const num_agents = 2;
  int const n = 5;

  ofstream ofs;
  ofs.open("/tmp/policy_comp.dat");
  ofs << "# Iterations\tUCT\tParetoUCT\tSlack\tEpsilonGreedy\n";

  JointReward opti_reward(num_agents, Reward::Zero());
  //opti_reward = get_optimal_reward(optimal.state);

  ArrayXi sample_sizes = ArrayXi::LinSpaced(30, 10, 100000);
  int step = 1;
  for (int i : sample_sizes) {
    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    vector<JointReward> avg(StatStrategy::NUM, JointReward(num_agents, Reward::Zero()));

    for (int j = 0; j < n; ++j) {
      CrossingTestEnv<UctStatistic<>> uct;
      CrossingTestEnv<ParetoUCTStatistic> pareto;
      CrossingTestEnv<SlackUCTStatistic> slack;
      CrossingTestEnv<EGreedyUCTStatistic> egreedy;
      run_test(uct, i);
      run_test(pareto, i);
      run_test(slack, i);
      run_test(egreedy, i);
      avg[StatStrategy::UCT] += uct.rewards;
      avg[StatStrategy::PARETO] += pareto.rewards;
      avg[StatStrategy::SLACK] += slack.rewards;
      avg[StatStrategy::EGREEDY] += egreedy.rewards;
    }

    ofs << i << "\t";
    for (auto &iter : avg) {
      std::transform(iter.begin(),
                     iter.end(),
                     iter.begin(),
                     [n](Reward const &r) { return r / static_cast<float>(n); });
      write_plot_output(ofs, iter, opti_reward);
    }
    ofs << "\n";

    LOG(WARNING) << "Metrics:";

    LOG(WARNING) << "Optimal vs UCT:";
    LOG(WARNING) << calculate_metric(avg[StatStrategy::UCT], opti_reward);

    LOG(WARNING) << "Optimal vs Pareto:";
    LOG(WARNING) << calculate_metric(avg[StatStrategy::PARETO], opti_reward);

    LOG(WARNING) << "Optimal vs Slack:";
    LOG(WARNING) << calculate_metric(avg[StatStrategy::SLACK], opti_reward);

    LOG(WARNING) << "Optimal vs E-Greedy:";
    LOG(WARNING) << calculate_metric(avg[StatStrategy::EGREEDY], opti_reward);
    ++step;
  }
  ofs.close();
  return 0;
}

