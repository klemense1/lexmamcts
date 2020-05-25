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

  struct Result {
    Result() :  collision(false), violation(false){};
    friend ostream &operator<<(ostream &os, const Result &result);
    static ostream &WriteHeader(ostream &os);
    int pos;
    float value;
    bool collision;
    bool violation;
   private:
    static inline const char* BoolToString(bool b) {
      return b ? "true" : "false";
    }
  };

  TestRunner() : factory_(nullptr), q_val_fname_("/tmp/q_val.dat") {};
  explicit TestRunner(ITestEnvFactory *factory) : factory_(factory), q_val_fname_("/tmp/q_val.dat") {};
  Result RunTest(size_t num_iter, int max_steps = 40);
  const std::shared_ptr<BaseTestEnv> &GetLatestTestEnv() const;
  Eigen::VectorXi GetStateVector() const;
  void SetQValFname(const std::string &q_val_fname);

 private:
  void PrintLabels();
  void PrintRuleStates();
  std::shared_ptr<ITestEnvFactory> factory_;
  std::shared_ptr<BaseTestEnv> latest_test_env_;
  std::string q_val_fname_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_RUNNER_H_
