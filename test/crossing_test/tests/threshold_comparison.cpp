//
// Created by Luis Gressenbuch on 18.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include <vector>

#include "mcts/statistics/thres_uct_statistic.h"
#include "mcts/statistics/e_greedy_statistic.h"
#include "test/crossing_test/factories/zipper_test_env_factory.h"
#include "test/crossing_test/state_file_writer.h"
#include "test/crossing_test/tests/test_runner.h"

void run(const ObjectiveVec& thres, int test_no) {
  char fname[100];
  sprintf(fname, "/home/luis/Documents/thesis/data/threshold_compare/threshold_compare_%d.dat", test_no);
  StateFileWriter sfw(3, std::string(fname));
  auto test_runner = std::make_unique<TestRunner>(new ZipperTestEnvFactory<ThresUCTStatistic>(thres, 0.0));
  sprintf(fname, "/home/luis/Documents/thesis/data/threshold_compare/threshold_compare_q_val_%d.dat", test_no);
  test_runner->SetQValFname(std::string(fname));
  auto res = test_runner->RunTest(5000, 16);
  TestRunner::Result::WriteHeader(LOG(INFO));
  LOG(INFO) << res;
  sfw.write_multi_timestep(test_runner->GetLatestTestEnv()->state_history_);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_v = 1;
  FLAGS_alsologtostderr = true;
  mcts::RandomGenerator::random_generator_ = std::mt19937(1000);

  std::vector<ObjectiveVec> thresholds(3, ObjectiveVec::Zero(3));
  thresholds[0] << -1.0, -1.0, std::numeric_limits<ObjectiveVec::Scalar>::max();
  thresholds[1] << -0.37, -0.37  , std::numeric_limits<ObjectiveVec::Scalar>::max();
  thresholds[2] << -0.1, -0.8, std::numeric_limits<ObjectiveVec::Scalar>::max();

  for(size_t i = 0; i < thresholds.size(); ++i) {
    run(thresholds[i], i);
  }

  return 0;
}