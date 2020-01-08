//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test_runner.h"
void TestRunner::run_test(size_t num_iter, int max_steps) {
  latest_test_env_ = factory_->make_test_env();
  int steps = 0;
  std::vector<Reward> step_reward(latest_test_env_->rewards);
  latest_test_env_->state_history_.emplace_back(get_state_vector().transpose());
  while (!latest_test_env_->state->is_terminal() && steps < max_steps) {
    latest_test_env_->search(num_iter);
    latest_test_env_->state = latest_test_env_->state->execute(latest_test_env_->get_jt(), step_reward);
    VLOG(1) << "Iteration: " << steps
            << ", Next state: " << latest_test_env_->state->sprintf();
    latest_test_env_->rewards += step_reward;
    latest_test_env_->state_history_.emplace_back(
        get_state_vector().transpose());
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
JointReward TestRunner::calculate_default_reward() {
  JointReward step_reward(latest_test_env_->rewards.size(), Reward::Zero(latest_test_env_->mcts_parameters_.REWARD_VEC_SIZE));
  JointReward accu_reward(latest_test_env_->rewards.size(), Reward::Zero(latest_test_env_->mcts_parameters_.REWARD_VEC_SIZE));
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
void OptiTest::run_test(size_t num_iter, int max_steps) {
  latest_test_env_ = DefaultTestEnvFactory().make_test_env();
  get_optimal_reward(latest_test_env_.get());
}
JointReward OptiTest::calculate_default_reward() {
  return latest_test_env_->rewards;
}
const std::shared_ptr<BaseTestEnv> &TestRunner::get_latest_test_env() const {
  return latest_test_env_;
}
