//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_TESTS_ZIPPER_TEST_ENV_FACTORY_H_
#define TEST_CROSSING_TEST_TESTS_ZIPPER_TEST_ENV_FACTORY_H_

#include "test/crossing_test/factories/test_env_factory.h"

class ZipperTestEnvFactory : public ITestEnvFactory{
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

#endif  // TEST_CROSSING_TEST_TESTS_ZIPPER_TEST_ENV_FACTORY_H_
