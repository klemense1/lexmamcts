//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test_runner.h"
#include "evaluation/evaluation.h"

using mcts::evaluation::QValWriter;

TestRunner::Result TestRunner::run_test(size_t num_iter, int max_steps) {
  // Always recreate test environment to isolate test iterations
  latest_test_env_ = factory_->make_test_env();
  int steps = 0;
  std::vector<Reward> step_reward(
      latest_test_env_->crossing_state_parameter_.num_other_agents + 1);
  // Store initial state in history
  latest_test_env_->state_history_.emplace_back(get_state_vector().transpose());
  QValWriter qw(
      latest_test_env_->mcts_parameters_.thres_uct_statistic_.THRESHOLD,
      q_val_fname_,
      latest_test_env_->crossing_state_parameter_.action_map.size());
  while (!latest_test_env_->state->is_terminal() && steps < max_steps) {
    latest_test_env_->search(num_iter);
    qw.WriteQVal(latest_test_env_->get_ego_qval(),
                 latest_test_env_->get_jt()[0]);
    latest_test_env_->state = latest_test_env_->state->execute(
        latest_test_env_->get_jt(), step_reward);
    latest_test_env_->state->reset_depth();
    VLOG(1) << "Iteration: " << steps
            << ", Next state: " << latest_test_env_->state->sprintf()
            << ", Ego step reward: " << step_reward[0].transpose();
    print_labels();
    print_rule_states();
    latest_test_env_->rewards += step_reward;
    latest_test_env_->state_history_.emplace_back(
        get_state_vector().transpose());
    ++steps;
  }
  latest_test_env_->rewards += latest_test_env_->state->get_final_reward();
  LOG(INFO) << "History:" << latest_test_env_->state_history_;
  auto cumulated_ego_reward = latest_test_env_->rewards[0];
  Result r;
  r.collision = cumulated_ego_reward(0) < 0;
  r.violation = cumulated_ego_reward(1) < 0;
  r.pos = latest_test_env_->state_history_.back()(0);
  r.value = cumulated_ego_reward(cumulated_ego_reward.size() - 1);
  return r;
}
Eigen::VectorXi TestRunner::get_state_vector() const {
  auto agent_states = latest_test_env_->state->get_agent_states();
  Eigen::VectorXi state = Eigen::VectorXi::Zero(agent_states.size());
  for (size_t i = 0; i < agent_states.size(); ++i) {
    state(i) = agent_states[i].x_pos;
  }
  return state;
}

ostream &operator<<(ostream &os, const TestRunner::Result &result) {
  os << result.pos << "\t" << result.value << "\t"
     << TestRunner::Result::BoolToString(result.collision) << "\t"
     << TestRunner::Result::BoolToString(result.violation);
  return os;
}
ostream &TestRunner::Result::write_header(ostream &os) {
  os << "Traveled distance\tBase reward\tCollision\tRule violation";
  return os;
}
void TestRunner::print_labels() {
  for (const auto &label : latest_test_env_->state->get_agent_labels(0)) {
    VLOG(1) << label.first.get_label_str() << " : " << label.second;
  }
}
void TestRunner::print_rule_states() {
  for (const auto &rs : latest_test_env_->state->get_rule_state_map()[0]) {
    VLOG(1) << rs.second;
  }
}
const std::shared_ptr<BaseTestEnv> &TestRunner::get_latest_test_env() const {
  return latest_test_env_;
}
void TestRunner::set_q_val_fname(const std::string &q_val_fname) {
  q_val_fname_ = q_val_fname;
}
