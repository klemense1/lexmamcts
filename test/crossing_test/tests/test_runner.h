//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_

#include "glog/logging.h"

#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/util.h"
#include "test_env_factory.h"

using std::vector;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;
using mcts::JointReward;

class TestRunner {
 public:
  TestRunner() : factory_(new DefaultTestEnvFactory()) {};
  explicit TestRunner(ITestEnvFactory *factory) : factory_(factory) {};
  virtual void run_test(size_t num_iter);

  virtual JointReward calculate_default_reward();

  double calculate_vector_utility(const Reward &candidate) const;
  double calculate_metric();

  struct Metrics {
    Metrics() : mean(0), n(0), m_2(0) {};
    double mean;
    int n;
    double m_2;
    inline double get_variance() { return m_2 / static_cast<double>(n); }
  };
 protected:
  std::shared_ptr<ITestEnvFactory> factory_;
  std::shared_ptr<BaseTestEnv> latest_test_env_;
 private:
  Metrics metrics_;
 public:
  const Metrics &get_metrics() const {
    return metrics_;
  }
  const std::shared_ptr<BaseTestEnv> &get_latest_test_env() const;
};

class OptiTest : public TestRunner {
 public:
  void run_test(size_t num_iter) override;
  JointReward calculate_default_reward() override;
};

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_
