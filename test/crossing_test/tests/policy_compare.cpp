//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <vector>
#include <iostream>

#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/common.h"

using std::vector;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;

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
  test_f.rewards += test_f.state->get_final_reward();
}

MatrixXf rewards_to_mat(vector<Reward> const &rewards) {
  MatrixXf mat(Reward::RowsAtCompileTime, rewards.size());
  for (size_t i = 0; i < rewards.size(); ++i) {
    mat.col(i) = rewards[i];
  }
  return mat;
}

double calculate_regret(vector<Reward> const &candidate, vector<Reward> const &optimal) {
   return (rewards_to_mat(optimal) - rewards_to_mat(candidate)).rowwise().sum().norm();
}

void write_plot_output(ostream &os, vector<Reward> const &candidate, vector<Reward> const &optimal) {
  os << calculate_regret(candidate, optimal) << "\t";
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = 1;
  FLAGS_logtostderr = 1;
  CrossingTestEnv<UctStatistic<>> optimal;
  size_t const num_agents = 2;
  int const n = 30;

  ofstream ofs;
  ofs.open("/tmp/policy_comp.dat");
  ofs << "# Iterations\tUCT\tParetoUCT\tSlack\n";

  vector<Reward> opti_reward(num_agents, Reward::Zero());
  opti_reward = get_optimal_reward(optimal.state);

  ArrayXi sample_sizes = ArrayXi::LinSpaced(50, 10, 1000);
  int step = 0;
  for (int i : sample_sizes) {
    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    vector<Reward> uct_avg(num_agents, Reward::Zero());
    vector<Reward> pareto_avg(num_agents, Reward::Zero());
    vector<Reward> slack_avg(num_agents, Reward::Zero());
    for (int j = 0; j < n; ++j) {
      CrossingTestEnv<UctStatistic<>> uct;
      CrossingTestEnv<ParetoUCTStatistic> pareto;
      CrossingTestEnv<SlackUCTStatistic> slack;
      run_test(uct, i);
      run_test(pareto, i);
      run_test(slack, i);
      uct_avg += uct.rewards;
      pareto_avg += pareto.rewards;
      slack_avg += slack.rewards;
    }
    std::transform(uct_avg.begin(),
                   uct_avg.end(),
                   uct_avg.begin(),
                   [n](Reward const &r) { return r / static_cast<float>(n); });
    std::transform(pareto_avg.begin(),
                   pareto_avg.end(),
                   pareto_avg.begin(),
                   [n](Reward const &r) { return r / static_cast<float>(n); });
    std::transform(slack_avg.begin(),
                   slack_avg.end(),
                   slack_avg.begin(),
                   [n](Reward const &r) { return r / static_cast<float>(n); });

    ofs << i << "\t";
    write_plot_output(ofs, uct_avg, opti_reward);
    write_plot_output(ofs, pareto_avg, opti_reward);
    write_plot_output(ofs, slack_avg, opti_reward);
    ofs << "\n";

    LOG(WARNING) << "L2 Norm:";

    LOG(WARNING) << "Optimal vs UCT:";
    LOG(WARNING) << calculate_regret(uct_avg, opti_reward);

    LOG(WARNING) << "Optimal vs Slack:";
    LOG(WARNING) << calculate_regret(slack_avg, opti_reward);

    LOG(WARNING) << "Optimal vs Pareto:";
    LOG(WARNING) << calculate_regret(pareto_avg, opti_reward);
    ++step;
  }
  ofs.close();
  return 0;
}

