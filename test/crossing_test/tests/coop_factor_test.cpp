//
// Created by Luis Gressenbuch on 06.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/factories/coop_factor_test_env_factory.h"
#include "test/crossing_test/state_file_writer.h"
#include "test/crossing_test/tests/test_runner.h"

class CoopFactorTest : public testing::TestWithParam<float> {};

TEST_P(CoopFactorTest, no_coop) {
  TestRunner tr(new CoopFactorTestEnvFactory(GetParam()));
  tr.run_test(10000, 25);
  char filename[50];
  sprintf(filename, "/tmp/coop_%.1f.dat", GetParam());
  StateFileWriter sfw(2, filename);
  sfw.write_multi_timestep(tr.get_latest_test_env()->state_history_);
  auto rule_states = tr.get_latest_test_env()->state->get_rule_state_map();
  for (size_t ai = 0; ai < rule_states.size(); ++ai) {
    EXPECT_EQ(rule_states.at(ai)
                  .find(Rule::NO_COLLISION)
                  ->second.get_violation_count(),
              0);
  }
}

const float coop_factors[] = {0.0, 0.5, 1.0};

INSTANTIATE_TEST_CASE_P(CoopFactor, CoopFactorTest,
                        testing::ValuesIn(coop_factors));

int main(int argc, char **argv) {
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}