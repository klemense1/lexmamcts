//
// Created by Luis Gressenbuch on 07.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"

#include "mvmcts/statistics/slack_uct_statistic.h"
#include "mvmcts/statistics/threshold_comperator.h"

using namespace mvmcts;

TEST(ComperatorTest, threshold_comperator_random) {
  const size_t reward_vec_size = 5;
  ObjectiveVec a = ObjectiveVec::Ones(reward_vec_size);
  ObjectiveVec b = ObjectiveVec::Ones(reward_vec_size);
  ObjectiveVec thr = ObjectiveVec::Constant(
      reward_vec_size, std::numeric_limits<ObjectiveVec::Scalar>::max());
  ThresholdComparator<ObjectiveVec> threshold_compare(thr);
  std::mt19937 prng(1000);
  std::uniform_int_distribution<int> distr(-100, 100);
  auto gen = [&distr, &prng]() { return distr(prng); };
  for (size_t i = 0; i < 100; i++) {
    std::generate(a.begin(), a.end(), gen);
    std::generate(b.begin(), b.end(), gen);
    a(0) = b(0);
    ASSERT_EQ(
        std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()),
        threshold_compare(a, b));
  }
}

TEST(ComperatorTest, threshold_comperator) {
  size_t vec_size = 3;
  ObjectiveVec a = ObjectiveVec::Zero(vec_size);
  ObjectiveVec b = ObjectiveVec::Zero(vec_size);
  ObjectiveVec thr = ObjectiveVec::Zero(vec_size);
  a << 10, 5, 10;
  b << 5, 10, 15;
  thr << 0, 0, 0;
  ThresholdComparator<ObjectiveVec> threshold_compare(thr);
  ASSERT_FALSE(threshold_compare(a, b));

  a << 10, 5, 10;
  b << 5, 10, 15;
  thr << 0, 0, 20;
  ThresholdComparator<ObjectiveVec> threshold_compare_2(thr);
  ASSERT_TRUE(threshold_compare_2(a, b));
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}