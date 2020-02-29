//
// Created by Luis Gressenbuch on 21.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include <vector>
#include <easy/profiler.h>
#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/factories/crossing_test_env_factory.h"
#include "test/crossing_test/factories/test_env_factory.h"
#include "test/crossing_test/factories/zipper_test_env_factory.h"
#include "test/crossing_test/tests/test_runner.h"
#include "test/crossing_test/tests/util.h"
#include "evaluation/evaluation.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  setup();
  // Use true randomness
  mcts::RandomGenerator::random_generator_ = std::mt19937(std::random_device()());
//  auto test_runner = std::make_unique<TestRunner>(new CrossingTestEnvFactory<ThresUCTStatistic>());
      auto test_runner = std::make_unique<TestRunner>(new ZipperTestEnvFactory<ThresUCTStatistic>(0, <#initializer #>));
  test_runner->run_test(10000, 1);
  shutdown("/tmp/crossing_performance.prof");
  return 0;
}