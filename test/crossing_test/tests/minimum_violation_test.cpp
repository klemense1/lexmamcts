//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "test_env_factory.h"
#include "test_runner.h"

TEST(MinimumViolation, main) {
  TestRunner test_runner(new MinimumViolationTestEnvFactory());
  test_runner.run_test(5000);
  LOG(INFO) << "Trajectories:";
  LOG(INFO) << "Ego: " << test_runner.get_latest_test_env()->pos_history;
  LOG(INFO) << "Other: " << test_runner.get_latest_test_env()->pos_history_other;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = true;
  FLAGS_v = 1;
  return RUN_ALL_TESTS();
}