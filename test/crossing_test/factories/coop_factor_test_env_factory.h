//
// Created by Luis Gressenbuch on 06.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_FACTORIES_COOP_FACTOR_TEST_ENV_FACTORY_H_
#define TEST_CROSSING_TEST_FACTORIES_COOP_FACTOR_TEST_ENV_FACTORY_H_

#include "test/crossing_test/factories/test_env_factory.h"

class CoopFactorTestEnvFactory : public ITestEnvFactory {
 public:
  CoopFactorTestEnvFactory(const float coop_factor);
  std::shared_ptr<BaseTestEnv> make_test_env() override;

 private:
  const float coop_factor_;
};

#endif  // TEST_CROSSING_TEST_FACTORIES_COOP_FACTOR_TEST_ENV_FACTORY_H_
