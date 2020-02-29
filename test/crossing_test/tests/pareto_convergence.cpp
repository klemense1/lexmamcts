//
// Created by Luis Gressenbuch on 24.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include <vector>

#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/factories/pareto_convergence_test_env_factory.h"
#include "test/crossing_test/factories/crossing_test_env_factory.h"
#include "test/crossing_test/state_file_writer.h"
#include "test/crossing_test/tests/test_runner.h"
#include "test/crossing_test/tests/util.h"

std::string Q_VAL_DUMPFILE;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_v = 0;
  FLAGS_logtostderr = true;
  TestRunner::Metrics m;
  mcts::RandomGenerator::random_generator_ = std::mt19937(1000);
  Q_VAL_DUMPFILE = "/home/luis/Documents/thesis/data/pareto_convergence_tlo.dat";
  auto test_runner = std::make_unique<TestRunner>(new CrossingTestEnvFactory<ThresUCTStatistic>());
  test_runner->run_test(10000, 1);
  TestRunner::Metrics::write_header(LOG(INFO));
  m = test_runner->calculate_metric();
  LOG(INFO) << m;

  mcts::RandomGenerator::random_generator_ = std::mt19937(1000);
  Q_VAL_DUMPFILE = "/home/luis/Documents/thesis/data/pareto_convergence_uct.dat";
  test_runner = std::make_unique<TestRunner>(new CrossingTestEnvFactory<UctStatistic<>>());
  test_runner->run_test(10000, 1);
  TestRunner::Metrics::write_header(LOG(INFO));
  m = test_runner->calculate_metric();
  LOG(INFO) << m;

  mcts::RandomGenerator::random_generator_ = std::mt19937(1000);
  Q_VAL_DUMPFILE = "/home/luis/Documents/thesis/data/pareto_convergence_slack.dat";
  test_runner = std::make_unique<TestRunner>(new CrossingTestEnvFactory<SlackUCTStatistic>());
  test_runner->run_test(10000, 1);
  TestRunner::Metrics::write_header(LOG(INFO));
  m = test_runner->calculate_metric();
  LOG(INFO) << m;

  return 0;
}
