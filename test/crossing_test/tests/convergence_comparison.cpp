//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <iostream>
#include <vector>

#include "mcts/statistics/e_greedy_statistic.h"
#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/factories/zipper_test_env_factory.h"
#include "test/crossing_test/tests/test_runner.h"
#include "test/crossing_test/tests/util.h"

using Eigen::ArrayXi;
using Eigen::MatrixXf;
using mcts::JointReward;
using std::ofstream;
using std::ostream;
using std::stringstream;

const std::array<std::string, 5> stat_names = {"Strict", "TLO", "Slack0.1", "Slack0.2", "Slack0.3"};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = 1;
  //  FLAGS_v = 1;
  FLAGS_logtostderr = true;
  int const iterations = 20;

  // Use true randomness
  mcts::RandomGenerator::random_generator_ = std::mt19937(std::random_device()());

  ofstream ofs;

  ofs.open("/home/luis/Documents/thesis/data/convergence_comparison.dat");

  ofs << "Iterations\tSelection policy\t";
  TestRunner::Result::WriteHeader(ofs);
  ofs << "\n";

  //  ArrayXi sample_sizes = ArrayXi::LinSpaced(5, 4, 500);
    std::vector<int> sample_sizes = {10, 25, 50, 75, 100, 250, 500, 750, 1000, 2500, 5000, 7500, 10000, 25000, 50000};
//  std::vector<int> sample_sizes = {10, 25, 50, 75, 100};
  int step = 1;
  std::vector<std::unique_ptr<TestRunner>> test_runners;

  test_runners.emplace_back(new TestRunner(new ZipperTestEnvFactory<mcts::UctStatistic<>>()));
  test_runners.emplace_back(new TestRunner(new ZipperTestEnvFactory<mcts::ThresUCTStatistic>()));
  test_runners.emplace_back(new TestRunner(new ZipperTestEnvFactory<mcts::SlackUCTStatistic>(0.1)));
  test_runners.emplace_back(new TestRunner(new ZipperTestEnvFactory<mcts::SlackUCTStatistic>(0.2)));
  test_runners.emplace_back(new TestRunner(new ZipperTestEnvFactory<mcts::SlackUCTStatistic>(0.3)));

  for (int i : sample_sizes) {
    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    int n_test_runners = 0;
    for (auto &it : test_runners) {
      LOG(WARNING) << "\tRunner [ " << n_test_runners + 1 << " / " << test_runners.size() << " ]";
      for (int j = 0; j < iterations; ++j) {
        LOG(WARNING) << "\t\tIterations [ " << j + 1 << " / " << iterations << " ]";
        auto res = it->RunTest(i, 15);
        ofs << i << "\t" << stat_names.at(n_test_runners) << "\t" << res << "\n";
        ofs.flush();
      }
      ++n_test_runners;
    }
    ++step;
  }
  ofs.close();
  LOG(WARNING) << "Completed!";
  return 0;
}
