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
#include "python/venv/lib/python3.6/site-packages/tensorflow_core/include/Eigen/src/SVD/JacobiSVD_LAPACKE.h"

using std::vector;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;
using mcts::JointReward;

class OptiTest;

class ITest {
 public:
  struct Metrics {
    Metrics() : mean(0), n(0), m_2(0) {};
    double mean;
    double m_2;
    int n;
    double get_variance() { return m_2 / static_cast<double>(n); }
  };
  virtual void run_test(size_t num_iter) = 0;
  virtual JointReward calculate_default_reward() = 0;
  virtual double calculate_metric() = 0;
  virtual const Metrics &get_metrics() const = 0;
};

template<class T>
class BaseTest : public ITest {
 public:
  BaseTest() {};
  virtual MctsParameters create_mcts_params() {
    return make_default_mcts_parameters();
  };
  virtual CrossingStateParameter create_crossing_params() {
    return make_default_crossing_state_parameters();
  };
  virtual std::shared_ptr<CrossingTestEnv<T>> create_test_env() {
    return std::make_shared<CrossingTestEnv<T>>(this->create_mcts_params(), this->create_crossing_params());
  }
  virtual void run_test(size_t num_iter) override {
    latest_test_env_ = this->create_test_env();
    const int MAX_STEPS = 40;
    int steps = 0;
    std::vector<Reward> step_reward(latest_test_env_->rewards);
    latest_test_env_->pos_history.emplace_back(latest_test_env_->state->get_ego_pos());
    latest_test_env_->pos_history_other.emplace_back(latest_test_env_->state->get_agent_states()[1].x_pos);
    while (!latest_test_env_->state->is_terminal() && steps < MAX_STEPS) {
      latest_test_env_->mcts.search(*latest_test_env_->state, 50000, num_iter);
      latest_test_env_->set_jt(latest_test_env_->mcts.returnBestAction());
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

  virtual JointReward calculate_default_reward() override {
    JointReward step_reward(latest_test_env_->rewards.size(), Reward::Zero());
    JointReward accu_reward(latest_test_env_->rewards.size(), Reward::Zero());
    auto const &actions = latest_test_env_->get_action_history();
    CrossingTestEnv<> d_test_env;
    for (auto const &jt : actions) {
      d_test_env.state = d_test_env.state->execute(jt, step_reward);
      accu_reward += step_reward;
    }
    accu_reward += d_test_env.state->get_final_reward();
    DVLOG(2) << "Default reward: " << accu_reward;
    return accu_reward;
  }

  double calculate_vector_utility(const Reward &candidate) const {
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
  double calculate_metric() override {
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

 protected:
  std::shared_ptr<CrossingTestEnv<T>> latest_test_env_;
 private:
  Metrics metrics_;
 public:
  const Metrics &get_metrics() const override {
    return metrics_;
  }
};

class ScalarizedTest : public BaseTest<UctStatistic<>> {
 public:
  CrossingStateParameter create_crossing_params() override {
    CrossingStateParameter p = make_default_crossing_state_parameters();
    p = make_default_crossing_state_parameters();
    p.speed_deviation_prio = 0;
    p.depth_weight = 0;
    p.speed_deviation_weight = 200;
    p.acceleration_weight = 0;
    p.potential_weight = 0;
    return p;
  }
  MctsParameters create_mcts_params() override {
    MctsParameters p = make_default_mcts_parameters();
    p.DISCOUNT_FACTOR = 0.9;
    return p;
  }
  std::shared_ptr<CrossingTestEnv<UctStatistic<>>> create_test_env() override {
    auto test_env_ptr = BaseTest::create_test_env();
    auto automata = test_env_ptr->get_automata();
    for (auto &aut : automata) {
      aut.erase(Rule::NO_SPEEDING);
      aut.erase(Rule::REACH_GOAL);
      aut.at(Rule::GIVE_WAY).set_weight(0.0f);
      aut.at(Rule::NO_COLLISION).set_weight(-1000.0f);
      aut.at(Rule::LEAVE_INTERSECTION).set_weight(-300.0f);
    }
    automata[0].at(Rule::GIVE_WAY).set_weight(-500.0f);
    automata[0].at(Rule::GIVE_WAY).set_type(RewardPriority::SAFETY);
    test_env_ptr->set_automata(automata);
    return test_env_ptr;
  }
};

class MaxTest : public BaseTest<MaxUCTStatistic> {
 public:
  CrossingStateParameter create_crossing_params() override {
    CrossingStateParameter p = make_default_crossing_state_parameters();
    p = make_default_crossing_state_parameters();
    p.depth_prio = static_cast<int>(RewardPriority::GOAL);
    p.speed_deviation_prio = static_cast<int>(RewardPriority::GOAL);
    p.acceleration_prio = static_cast<int>(RewardPriority::GOAL);
    p.potential_prio = static_cast<int>(RewardPriority::GOAL);
    p.depth_weight = 0;
    p.speed_deviation_weight = 0;
    p.acceleration_weight = 0;
    p.potential_weight = 1;
    return p;
  }

  MctsParameters create_mcts_params() override {
    MctsParameters p = make_default_mcts_parameters();
    p.uct_statistic.PROGRESSIVE_WIDENING_ALPHA = 0.25;
    p.uct_statistic.PROGRESSIVE_WIDENING_ENABLED = true;
    p.DISCOUNT_FACTOR = 1.0;
    return p;
  }

  std::shared_ptr<CrossingTestEnv<MaxUCTStatistic>> create_test_env() override {
    auto test_env_ptr = BaseTest::create_test_env();
    auto automata = test_env_ptr->get_automata();
    for (auto &aut : automata) {
      aut.erase(Rule::LEAVE_INTERSECTION);
      aut.erase(Rule::REACH_GOAL);
      aut.at(Rule::NO_SPEEDING).set_weight(-1.0f);
      aut.at(Rule::GIVE_WAY).set_weight(0.0f);
      aut.at(Rule::NO_COLLISION).set_weight(-1.0f);
    }
    automata[0].at(Rule::GIVE_WAY).set_weight(-1.0f);
    test_env_ptr->set_automata(automata);
    return test_env_ptr;
  }
};

class ThresholdTest : public BaseTest<ThresUCTStatistic> {
 private:
 public:
  CrossingStateParameter create_crossing_params() override {
    CrossingStateParameter p = make_default_crossing_state_parameters();
    p.depth_prio = static_cast<int>(RewardPriority::GOAL);
    p.speed_deviation_prio = static_cast<int>(RewardPriority::GOAL);
    p.acceleration_prio = static_cast<int>(RewardPriority::GOAL);
    p.potential_prio = static_cast<int>(RewardPriority::GOAL);
    p.depth_weight = 0;
    p.speed_deviation_weight = 0;
    p.acceleration_weight = 0;
    p.potential_weight = 1;
    return p;
  }
  std::shared_ptr<CrossingTestEnv<ThresUCTStatistic>> create_test_env() override {
    auto test_env_ptr = BaseTest::create_test_env();
    auto automata = test_env_ptr->get_automata();
    for (auto &aut : automata) {
      aut.at(Rule::NO_SPEEDING).set_weight(-1.0f);
      aut.at(Rule::REACH_GOAL).set_weight(-100.0f);
      aut.at(Rule::REACH_GOAL).set_final_reward(100.0f);
      aut.at(Rule::GIVE_WAY).set_weight(0.0f);
      aut.at(Rule::NO_COLLISION).set_weight(-1.0f);
      aut.erase(Rule::LEAVE_INTERSECTION);
    }
    automata[0].at(Rule::GIVE_WAY).set_weight(-1.0f);
    test_env_ptr->set_automata(automata);
    return test_env_ptr;
  }
};

class OptiTest : public BaseTest<UctStatistic<>> {
 public:
  void run_test(size_t num_iter) override {
    latest_test_env_ = std::make_shared<CrossingTestEnv<>>(this->create_mcts_params(), this->create_crossing_params());
    get_optimal_reward(latest_test_env_.get());
  }
  JointReward calculate_default_reward() override {
    return latest_test_env_->rewards;
  }
};

void write_plot_output(ostream &os, double value) {
  os << value << "\t";
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  //FLAGS_minloglevel = 1;
  FLAGS_v = 1;
  FLAGS_logtostderr = true;
  OptiTest optimal;
  int const n = 1;

  ofstream ofs;
  ofs.open("/tmp/trajectory_comp.dat");
  ofs << "# Iterations\tScalarized\tMax Selection\tThreshold Statistics\n";

  optimal.run_test(0);
  ITest::Metrics optimal_metrics = optimal.get_metrics();

  std::vector<std::unique_ptr<ITest>> test_runners;
  test_runners.emplace_back(new ScalarizedTest());
  test_runners.emplace_back(new MaxTest());
  test_runners.emplace_back(new ThresholdTest());

  ArrayXi sample_sizes = ArrayXi::LinSpaced(1, 10000, 10000);
  int step = 1;
  for (int i : sample_sizes) {
    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    for (int j = 0; j < n; ++j) {
      for (auto &it : test_runners) {
        it->run_test(i);
      }
    }

    ofs << i << "\t";
    for (auto &iter : test_runners) {
      auto m = iter->get_metrics();
      write_plot_output(ofs, m.mean);
      write_plot_output(ofs, m.get_variance());
      write_plot_output(ofs, std::abs(m.mean - optimal_metrics.mean) / optimal_metrics.mean);
    }
    ofs << "\n";
    ++step;
  }
  ofs.close();
  return 0;
}

