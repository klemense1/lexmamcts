//
// Created by Luis Gressenbuch on 07.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"

#include "mcts/statistics/slack_uct_statistic.h"
#include "mcts/statistics/thres_uct_statistic.h"

using namespace mcts;

TEST(ComperatorTest, slack_comperator) {
  const size_t reward_vec_size = 5;
  ActionUCBMap::value_type a = {0, UcbPair(reward_vec_size)};
  ActionUCBMap::value_type b = {1, UcbPair(reward_vec_size)};
  std::vector<ObjectiveVec> slack = {ObjectiveVec::Zero(reward_vec_size), ObjectiveVec::Zero(reward_vec_size)};
  ObjectiveVec &slack_a = slack[0];
  ObjectiveVec &slack_b = slack[1];
  a.second.action_value_ << 0, 0, 0, 0, 0;
  b.second.action_value_ << 4, 4, 4, 4, 4;
  slack_a << 2, 2, 4, 0, 0;
  slack_b << 2, 3, 0, 4, 4;
  // a == b
  SlackUCTStatistic::SlackComperator slack_compare(slack);
  ASSERT_FALSE(slack_compare(a, b));
  ASSERT_FALSE(slack_compare(b, a));
  // a < b
  slack_a << 1, 1, 0, 0, 0;
  slack_b << 1, 1, 0, 0, 0;
  SlackUCTStatistic::SlackComperator slack_compare1(slack);
  ASSERT_TRUE(slack_compare1(a, b));
  ASSERT_FALSE(slack_compare1(b, a));
  // a > b
  a.second.action_value_ << 4, 4, 4, 4, 4;
  b.second.action_value_ << 0, 0, 0, 0, 0;
  slack_a << 1, 1, 0, 0, 0;
  slack_b << 1, 1, 0, 0, 0;
  SlackUCTStatistic::SlackComperator slack_compare2(slack);
  ASSERT_FALSE(slack_compare2(a, b));
  ASSERT_TRUE(slack_compare2(b, a));
}

TEST(ComperatorTest, threshold_comperator_random) {
  const size_t reward_vec_size = 5;
  ObjectiveVec a = ObjectiveVec::Ones(reward_vec_size);
  ObjectiveVec b = ObjectiveVec::Ones(reward_vec_size);
  ObjectiveVec thr = ObjectiveVec::Constant(reward_vec_size, std::numeric_limits<ObjectiveVec::Scalar>::max());
  ThresUCTStatistic::ThresholdComparator threshold_compare(thr);
  std::mt19937 prng(1000);
  std::uniform_int_distribution<int> distr(-100, 100);
  auto gen = [&distr, &prng]() {
    return distr(prng);
  };
  for (size_t i = 0; i < 100; i++) {
    std::generate(a.begin(), a.end(), gen);
    std::generate(b.begin(), b.end(), gen);
    a(0) = b(0);
    ASSERT_EQ(std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()), threshold_compare(a, b));
  }
}

TEST(ComperatorTest, threshold_comperator) {
  size_t vec_size = 5;
  ObjectiveVec a = ObjectiveVec::Zero(vec_size);
  ObjectiveVec b = ObjectiveVec::Zero(vec_size);
  ObjectiveVec thr = ObjectiveVec::Zero(vec_size);
  a << 20, 10, 20, 10, 10;
  b << 10, 20, 10, 20, 20;
  thr << 10, 100, 100, 100, 100;
  ThresUCTStatistic::ThresholdComparator threshold_compare(thr);
  ASSERT_FALSE(std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()));
  ASSERT_TRUE(threshold_compare(a, b));
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}