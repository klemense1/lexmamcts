//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "default_test_env_factory.h"

std::shared_ptr<BaseTestEnv> DefaultTestEnvFactory::make_test_env() {
  return std::make_shared<CrossingTestEnv<>>();
}
