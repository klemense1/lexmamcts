//
// Created by Luis Gressenbuch on 07.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"

#include "mcts/statistics/slack_uct_statistic.h"
#include "mcts/statistics/thres_uct_statistic.h"

using namespace mcts;

TEST(ComperatorTest, slack_comperator) {
  ObjectiveVec a = ObjectiveVec::Ones();
  ObjectiveVec b = ObjectiveVec::Ones();
  ObjectiveVec slack_a = ObjectiveVec::Zero();
  ObjectiveVec slack_b = ObjectiveVec::Zero();
  a << 0, 0, 0, 0;
  b << 4, 4, 4, 4;
  slack_a << 2, 2, 4, 0;
  slack_b << 2, 3, 0, 4;
  // a == b
  ASSERT_FALSE(slack_compare(a, b, slack_a, slack_b));
  ASSERT_FALSE(slack_compare(b, a, slack_b, slack_a));
  // a < b
  slack_a << 1, 1, 0, 0;
  slack_b << 1, 1, 0, 0;
  ASSERT_TRUE(slack_compare(a, b, slack_a, slack_b));
  ASSERT_FALSE(slack_compare(b, a, slack_b, slack_a));
  // a > b
  a << 4, 4, 4, 4;
  b << 0, 0, 0, 0;
  slack_a << 1, 1, 0, 0;
  slack_b << 1, 1, 0, 0;
  ASSERT_FALSE(slack_compare(a, b, slack_a, slack_b));
  ASSERT_TRUE(slack_compare(b, a, slack_b, slack_a));
}

TEST(ComperatorTest, threshold_comperator_random) {
  ObjectiveVec a = ObjectiveVec::Ones();
  ObjectiveVec b = ObjectiveVec::Ones();
  ObjectiveVec thr = ObjectiveVec::Constant(std::numeric_limits<float>::max());
  std::mt19937 prng(1000);
  std::uniform_int_distribution<int> distr(-100, 100);
  auto gen = [&distr, &prng]() {
    return distr(prng);
  };
  for (size_t i = 0; i < 100; i++) {
    std::generate(a.begin(), a.end(), gen);
    std::generate(b.begin(), b.end(), gen);
    a(0) = b(0);
    ASSERT_EQ(std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()), threshold_compare(a, b, thr));
  }
}

TEST(ComperatorTest, threshold_comperator) {
  ObjectiveVec a;
  ObjectiveVec b;
  ObjectiveVec thr;
  a << 20, 10, 20, 10;
  b << 10, 20, 10, 20;
  thr << 10, 100, 100, 100;
  ASSERT_FALSE(std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()));
  ASSERT_TRUE(threshold_compare(a, b, thr));
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}