//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "max_uct_test_env_factory.h"
#include "mcts/statistics/max_uct_statistic.h"

std::shared_ptr<BaseTestEnv> MaxUCTTestEnvFactory::make_test_env() {
  return std::make_shared<CrossingTestEnv<mcts::MaxUCTStatistic>>();
}