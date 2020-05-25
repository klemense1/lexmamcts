//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_ENV_FACTORY_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_ENV_FACTORY_H_

#include <memory>

#include "test/crossing_test/tests/crossing_test_env.h"

class ITestEnvFactory {
 public:
  virtual std::shared_ptr<BaseTestEnv> MakeTestEnv() = 0;
};

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_TEST_ENV_FACTORY_H_
