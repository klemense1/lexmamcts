//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "threshold_test_env_factory.h"
#include "mcts/statistics/thres_uct_statistic.h"

std::shared_ptr<BaseTestEnv> ThresholdTestEnvFactory::make_test_env() {
  return std::make_shared<CrossingTestEnv<mcts::ThresUCTStatistic>>();
}