//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test_runner.h"
#include "evaluation/evaluation.h"

using mcts::evaluation::QValWriter;

void TestRunner::run_test(size_t num_iter, int max_steps) {
  // Always recreate test environment to isolate test iterations
  latest_test_env_ = factory_->make_test_env();
  int steps = 0;
  std::vector<Reward> step_reward(latest_test_env_->crossing_state_parameter_.num_other_agents + 1);
  // Store initial state in history
  latest_test_env_->state_history_.emplace_back(get_state_vector().transpose());
  QValWriter qw(latest_test_env_->mcts_parameters_.thres_uct_statistic_.THRESHOLD, q_val_fname_);
  while (!latest_test_env_->state->is_terminal() && steps < max_steps) {
    latest_test_env_->search(num_iter);
    qw.WriteQVal(latest_test_env_->get_ego_qval(), latest_test_env_->get_jt()[0]);
    latest_test_env_->state = latest_test_env_->state->execute(latest_test_env_->get_jt(), step_reward);
    latest_test_env_->state->reset_depth();
    VLOG(1) << "Iteration: " << steps << ", Next state: " << latest_test_env_->state->sprintf()
            << ", Ego step reward: " << step_reward[0].transpose();
    print_labels();
    print_rule_states();
    latest_test_env_->rewards += step_reward;
    latest_test_env_->state_history_.emplace_back(get_state_vector().transpose());
    ++steps;
  }
  latest_test_env_->rewards += latest_test_env_->state->get_final_reward();
  LOG(INFO) << "History:" << latest_test_env_->state_history_;
}
Eigen::VectorXi TestRunner::get_state_vector() const {
  auto agent_states = latest_test_env_->state->get_agent_states();
  Eigen::VectorXi state = Eigen::VectorXi::Zero(agent_states.size());
  for (size_t i = 0; i < agent_states.size(); ++i) {
    state(i) = agent_states[i].x_pos;
  }
  return state;
}

double TestRunner::calculate_vector_utility(const Reward &candidate) const {
  // Create an approximate utility function of the lexicographical ordering
  // by shifting higher priority rewards to the left
  double u = 0.0;
  double shift = 1.0;
  for (int d = candidate.rows() - 1; d >= 0; --d) {
    u += shift * candidate(d);
    shift *= std::abs(latest_test_env_->mcts_parameters_.uct_statistic.UPPER_BOUND(d) -
                      latest_test_env_->mcts_parameters_.uct_statistic.LOWER_BOUND(d)) +
             1;
  }
  return u;
}
TestRunner::Metrics TestRunner::calculate_metric() {
  Reward cumulated_ego_reward = rewards_to_mat(latest_test_env_->rewards).col(0);
  if (cumulated_ego_reward(0) < 0) {
    ++metrics_.collisions;
  } else {
    metrics_.value_.add_value(cumulated_ego_reward(cumulated_ego_reward.size() - 1));
    metrics_.step_cost_.add_value(cumulated_ego_reward(cumulated_ego_reward.size() - 1) / latest_test_env_->state_history_.size());
    metrics_.pos_.add_value(static_cast<double>(latest_test_env_->state_history_.back()(0)));
  }
  if (cumulated_ego_reward(1) < 0) {
    ++metrics_.violations;
  }
  return metrics_;
}

ostream &operator<<(ostream &os, const TestRunner::Metrics &metrics) {
  os << metrics.pos_ << metrics.value_ << metrics.step_cost_ << metrics.collisions << "\t" << metrics.violations << "\t";
  return os;
}
ostream &TestRunner::Metrics::write_header(ostream &os) {
  os << "# Mean final position\t\tMean cost\t\tMean step cost\t\t#Collisions\t#Rule violations\t";
  return os;
}
void TestRunner::print_labels() {
  for (const auto &label : latest_test_env_->state->get_agent_labels(0)) {
    VLOG(1) << label.first.get_label_str() << " : " << label.second;
  }
}
void TestRunner::print_rule_states() {
  for(const auto &rs : latest_test_env_->state->get_rule_state_map()[0]) {
    VLOG(1) << rs.second;
  }
}
ostream &operator<<(ostream &os, const TestRunner::MeanVar &var) {
  os << var.mean_ << "\t" << var.get_confidence_radius() << "\t";
  return os;
}
const std::shared_ptr<BaseTestEnv> &TestRunner::get_latest_test_env() const { return latest_test_env_; }
void TestRunner::set_q_val_fname(const std::string &q_val_fname) { q_val_fname_ = q_val_fname; }
