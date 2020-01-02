//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#define DEBUG
#define PLAN_DEBUG_INFO

#include <vector>
#include <iostream>

#include "test/crossing_test/factories/test_env_factory.h"
#include "test/crossing_test/factories/threshold_test_env_factory.h"
#include "test/crossing_test/tests/test_runner.h"
#include "test/crossing_test/tests/util.h"

using Eigen::ArrayXi;
using std::ostream;
using std::ofstream;
using std::stringstream;
using Eigen::MatrixXf;
using Eigen::ArrayXi;
using mcts::JointReward;



void write_plot_output(ostream &os, double value) {
  os << value << "\t";
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  //FLAGS_minloglevel = 1;
  FLAGS_v = 1;
  FLAGS_logtostderr = true;
  OptiTest optimal;
  int const n = 1;

  ofstream ofs;
  ofs.open("/tmp/trajectory_comp.dat");
  ofs << "# Iterations\tScalarized\tMax Selection\tThreshold Statistics\n";

  optimal.run_test(0);
  TestRunner::Metrics optimal_metrics = optimal.get_metrics();

  std::vector<std::unique_ptr<TestRunner>> test_runners;
  //  test_runners.emplace_back(new TestRunner(new ScalarizedTestEnvFactory()));
  //  test_runners.emplace_back(new TestRunner(new MaxUCTTestEnvFactory()));
  test_runners.emplace_back(new TestRunner(new ThresholdTestEnvFactory()));

  ArrayXi sample_sizes = ArrayXi::LinSpaced(1, 10000, 10000);
  int step = 1;
  for (int i : sample_sizes) {
    LOG(WARNING) << "Sample size: " << i << "  [ " << step << " / " << sample_sizes.size() << " ]";
    for (int j = 0; j < n; ++j) {
      for (auto &it : test_runners) {
        it->run_test(i);
      }
    }

    ofs << i << "\t";
    for (auto &iter : test_runners) {
      auto m = iter->get_metrics();
      write_plot_output(ofs, m.mean);
      write_plot_output(ofs, m.get_variance());
      write_plot_output(ofs, std::abs(m.mean - optimal_metrics.mean) / optimal_metrics.mean);
    }
    ofs << "\n";
    ++step;
  }
  ofs.close();
  return 0;
}

