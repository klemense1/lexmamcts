//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_

#include <ostream>
#include "glog/logging.h"

#include "boost/math/distributions/students_t.hpp"
#include "test/crossing_test/factories/test_env_factory.h"
#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/util.h"

using std::vector;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;
using mcts::JointReward;

class TestRunner {
 public:
  struct MeanVar {
    MeanVar() : mean_(0.0), m_2_(0), n_(0) {}
    friend ostream &operator<<(ostream &os, const MeanVar &var);
    double mean_;
    double m_2_;
    int n_;
    void add_value(double value) {
      ++n_;
      double delta = value - mean_;
      mean_ += delta / n_;
      double delta2 = value - mean_;
      m_2_ += delta * delta2;
    }
    inline double get_variance() const { return m_2_ / static_cast<double>(n_); }
    inline double get_confidence_radius() const {
      const double std_dev = std::sqrt(get_variance());
      if (n_ > 1) {
        boost::math::students_t dist(n_ - 1);
        // 95% confidence interval
        double t = quantile(complement(dist, 0.05 / 2.0));
        return t * std_dev / sqrt(static_cast<double>(n_));
      } else {
        return 0.0;
      }
    }
  };

  struct Metrics {
    Metrics() :  n(0), collisions(0), violations(0){};
    friend ostream &operator<<(ostream &os, const Metrics &metrics);
    static ostream &write_header(ostream &os);
    MeanVar pos_;
    MeanVar value_;
    MeanVar step_cost_;
    int n;
    int collisions;
    int violations;
  };

  TestRunner() : factory_(nullptr), q_val_fname_("/tmp/q_val.dat") {};
  explicit TestRunner(ITestEnvFactory *factory) : factory_(factory), q_val_fname_("/tmp/q_val.dat") {};
  virtual void run_test(size_t num_iter, int max_steps = 40);
  double calculate_vector_utility(const Reward &candidate) const;
  Metrics calculate_metric();
  const std::shared_ptr<BaseTestEnv> &get_latest_test_env() const;
  Eigen::VectorXi get_state_vector() const;
  const Metrics &get_metrics() const { return metrics_; }


 protected:
  std::shared_ptr<ITestEnvFactory> factory_;
  std::shared_ptr<BaseTestEnv> latest_test_env_;
 private:
  void print_labels();
  void print_rule_states();
  Metrics metrics_;
  std::string q_val_fname_;

 public:
  void set_q_val_fname(const std::string &q_val_fname);
};

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_
