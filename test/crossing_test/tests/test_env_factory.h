//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_ENV_FACTORY_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_ENV_FACTORY_H_

#include <memory>

#include "crossing_test_env.h"
#include "mcts/mcts_parameters.h"
#include "test/crossing_test/crossing_state_parameter.h"

class ITestEnvFactory {
 public:
  virtual std::shared_ptr<BaseTestEnv> make_test_env() = 0;
};

class DefaultTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

class ThresholdTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

class MinimumViolationTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

class MaxUCTTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

class ScalarizedTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};
#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_ENV_FACTORY_H_
