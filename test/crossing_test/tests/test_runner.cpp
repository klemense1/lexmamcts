//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test_runner.h"
void TestRunner::run_test(size_t num_iter) {
  latest_test_env_ = factory_->make_test_env();
  const int MAX_STEPS = 40;
  int steps = 0;
  std::vector<Reward> step_reward(latest_test_env_->rewards);
  latest_test_env_->pos_history.emplace_back(latest_test_env_->state->get_ego_pos());
  latest_test_env_->pos_history_other.emplace_back(latest_test_env_->state->get_agent_states()[1].x_pos);
  while (!latest_test_env_->state->is_terminal() && steps < MAX_STEPS) {
    latest_test_env_->search(num_iter);
    latest_test_env_->state = latest_test_env_->state->execute(latest_test_env_->get_jt(), step_reward);
    latest_test_env_->rewards += step_reward;
    latest_test_env_->pos_history.emplace_back(latest_test_env_->state->get_ego_pos());
    latest_test_env_->pos_history_other.emplace_back(latest_test_env_->state->get_agent_states()[1].x_pos);
    ++steps;
  }
  latest_test_env_->rewards += latest_test_env_->state->get_final_reward();
  calculate_metric();
  LOG(INFO) << "Ego history:" << latest_test_env_->pos_history;
  LOG(INFO) << "Otr history:" << latest_test_env_->pos_history_other;
}
JointReward TestRunner::calculate_default_reward() {
  JointReward step_reward(latest_test_env_->rewards.size(), Reward::Zero());
  JointReward accu_reward(latest_test_env_->rewards.size(), Reward::Zero());
  auto const &actions = latest_test_env_->get_action_history();
  auto d_test_env = DefaultTestEnvFactory().make_test_env();
  for (auto const &jt : actions) {
    d_test_env->state = d_test_env->state->execute(jt, step_reward);
    accu_reward += step_reward;
  }
  accu_reward += d_test_env->state->get_final_reward();
  DVLOG(2) << "Default reward: " << accu_reward;
  return accu_reward;
}
double TestRunner::calculate_vector_utility(const Reward &candidate) const {
  // Create an approximate utility function of the lexicographical ordering
  // by shifting higher priority rewards to the left
  double u = 0.0;
  double shift = 1.0;
  for (int d = candidate.rows() - 1; d >= 0; --d) {
    u += shift * candidate(d);
    shift *= std::abs(latest_test_env_->mcts_parameters_.uct_statistic.UPPER_BOUND(d)
                          - latest_test_env_->mcts_parameters_.uct_statistic.LOWER_BOUND(d)) + 1;
  }
  return u;
}
double TestRunner::calculate_metric() {
  // TODO: Calculate more metrics
  Reward candidate = rewards_to_mat(calculate_default_reward()).rowwise().sum();
  double new_value = calculate_vector_utility(candidate);
  ++metrics_.n;
  double delta = new_value - metrics_.mean;
  metrics_.mean += delta / metrics_.n;
  double delta2 = new_value - metrics_.mean;
  metrics_.m_2 += delta * delta2;
  metrics_.mean += calculate_vector_utility(candidate);
  return metrics_.mean;
}
void OptiTest::run_test(size_t num_iter) {
  latest_test_env_ = DefaultTestEnvFactory().make_test_env();
  get_optimal_reward(latest_test_env_.get());
}
JointReward OptiTest::calculate_default_reward() {
  return latest_test_env_->rewards;
}
const std::shared_ptr<BaseTestEnv> &TestRunner::get_latest_test_env() const {
  return latest_test_env_;
}
