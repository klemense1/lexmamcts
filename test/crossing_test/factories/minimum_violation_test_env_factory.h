//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_FACTORIES_MINIMUM_VIOLATION_TEST_ENV_FACTORY_H_
#define TEST_CROSSING_TEST_FACTORIES_MINIMUM_VIOLATION_TEST_ENV_FACTORY_H_

#include <memory>

#include "test/crossing_test/factories/test_env_factory.h"

class MinimumViolationTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

#endif  // TEST_CROSSING_TEST_FACTORIES_MINIMUM_VIOLATION_TEST_ENV_FACTORY_H_
