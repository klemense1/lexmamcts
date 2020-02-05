//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <iostream>
#include <vector>

#include "test/crossing_test/factories/max_uct_test_env_factory.h"
#include "test/crossing_test/factories/slack_test_env_factory.h"
#include "test/crossing_test/factories/test_env_factory.h"
#include "test/crossing_test/factories/threshold_test_env_factory.h"
#include "test/crossing_test/tests/test_runner.h"
#include "test/crossing_test/tests/util.h"

using Eigen::ArrayXi;
using Eigen::MatrixXf;
using mcts::JointReward;
using std::ofstream;
using std::ostream;
using std::stringstream;

const std::string spacer = "\t\t\t\t";
const std::array<std::string, 5> stat_names = {"UCTStatistic", "ThresholdStatistic", "SlackStatistic",
                                               "MaxUCTStatistic"};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = 1;
  //  FLAGS_v = 1;
  FLAGS_logtostderr = true;
  int const iterations = 3;

  // Use true randomness
  mcts::RandomGenerator::random_generator_ = std::mt19937(std::random_device()());

  ofstream ofs;
  ofs.open("/tmp/trajectory_comp.dat");
  ofs << "# Iterations\t";
  for (const auto &name : stat_names) {
    ofs << name << spacer;
  }
  ofs << "\n\t";
  for (size_t i = 0; i < stat_names.size(); ++i) {
    TestRunner::Metrics::write_header(ofs);
  }

  ArrayXi sample_sizes = ArrayXi::LinSpaced(20, 100, 50000);
  int step = 1;
  for (int i : sample_sizes) {
    std::vector<std::unique_ptr<TestRunner>> test_runners;
    test_runners.emplace_back(new TestRunner(new DefaultTestEnvFactory()));
    test_runners.emplace_back(new TestRunner(new ThresholdTestEnvFactory()));
    test_runners.emplace_back(new TestRunner(new SlackTestEnvFactory()));
    test_runners.emplace_back(new TestRunner(new MaxUCTTestEnvFactory()));

    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    ofs << i << "\t";
    int n_test_runners = 0;
    for (auto &it : test_runners) {
      LOG(WARNING) << "\tRunner [ " << n_test_runners + 1 << " / " << test_runners.size() << " ]";
      for (int j = 0; j < iterations; ++j) {
        LOG(WARNING) << "\t\tIterations [ " << j + 1 << " / " << iterations << " ]";
        it->run_test(i);
      }
      ofs << it->calculate_metric();
      ++n_test_runners;
    }
    ofs << "\n";
    ++step;
  }
  ofs.close();
  LOG(WARNING) << "Completed!";
  return 0;
}
