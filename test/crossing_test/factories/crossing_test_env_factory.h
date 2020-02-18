//
// Created by Luis Gressenbuch on 17.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_FACTORIES_CROSSING_TEST_ENV_FACTORY_H_
#define TEST_CROSSING_TEST_FACTORIES_CROSSING_TEST_ENV_FACTORY_H_

#include "test/crossing_test/factories/test_env_factory.h"
template <class Stat>
class CrossingTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override { return std::make_shared<CrossingTestEnv<Stat>>(); }
};

#endif  // TEST_CROSSING_TEST_FACTORIES_CROSSING_TEST_ENV_FACTORY_H_
