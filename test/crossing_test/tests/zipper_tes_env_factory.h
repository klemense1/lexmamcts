//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_ZIPPER_TES_ENV_FACTORY_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_ZIPPER_TES_ENV_FACTORY_H_

#include "test/crossing_test/tests/test_env_factory.h"

class ZipperTesEnvFactory : public ITestEnvFactory{
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override;
};

#endif  //  MAMCTS_TEST_CROSSING_TEST_TESTS_ZIPPER_TES_ENV_FACTORY_H_
