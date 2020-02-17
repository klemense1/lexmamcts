//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#define DEBUG
#define PLAN_DEBUG_INFO

#include <vector>

#include "test/crossing_test/factories/max_uct_test_env_factory.h"
#include "test/crossing_test/factories/slack_test_env_factory.h"
#include "test/crossing_test/factories/test_env_factory.h"
#include "test/crossing_test/factories/threshold_test_env_factory.h"
#include "test/crossing_test/tests/test_runner.h"
#include "test/crossing_test/tests/util.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_v = 1;
  FLAGS_logtostderr = true;

  // Use true randomness
  mcts::RandomGenerator::random_generator_ = std::mt19937(std::random_device()());

  auto test_runner = std::make_unique<TestRunner>(new ThresholdTestEnvFactory());
  test_runner->run_test(10000, 25);
  TestRunner::Metrics::write_header(LOG(INFO));
  LOG(INFO) << test_runner->calculate_metric();
  return 0;
}
