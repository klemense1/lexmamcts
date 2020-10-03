// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "gtest/gtest.h"

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO
#include <cstdio>
#include "mvmcts/heuristics/random_heuristic.h"
#include "mvmcts/statistics/uct_statistic.h"
#include "test/mcts_test_class.h"
#include "test/simple_state.h"

using namespace std;
using namespace mvmcts;

std::mt19937 mvmcts::RandomGenerator::random_generator_;

TEST(test_mcts, verify_uct) {
  RandomGenerator::random_generator_ = std::mt19937(1000);
  Mvmcts<SimpleState, UctStatistic<>, UctStatistic<>, RandomHeuristic> mcts(
      MakeDefaultMctsParameters());
  SimpleState state(4);

  mcts.Search(state, 50000, 2000);

  // MctsTest test;
  // test.verify_uct(mcts, 1);
}

// TEST(test_mcts, generate_dot_file) {
//   RandomGenerator::random_generator_ = std::mt19937(1000);
//   Mvmcts<SimpleState, UctStatistic<>, UctStatistic<>, RandomHeuristic> mcts(
//       MakeDefaultMctsParameters());
//   SimpleState state(4);

//   mcts.Search(state, 50000, 20);
//   mcts.PrintTreeToDotFile("test_tree");
// }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}