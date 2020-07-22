//
// Created by luis on 08.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO

#include "gtest/gtest.h"
#include "mvmcts/statistics/thres_greedy_statistic.h"
#include "test/mo_deep_sea/mo_deep_sea_state.hpp"

using namespace mvmcts;

MvmctsParameters MakeDefaultMctsParameters() {
  MvmctsParameters param;

  param.REWARD_VEC_SIZE = 2;

  param.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = 1;
  param.random_heuristic.MAX_NUMBER_OF_ITERATIONS = 1000;
  param.COOP_FACTOR = 0.0;
  param.DISCOUNT_FACTOR = 0.99;

  param.uct_statistic.EXPLORATION_CONSTANT =
      Eigen::VectorXd::Constant(param.REWARD_VEC_SIZE, 1.4);
  param.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  param.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);

  param.thres_uct_statistic_.THRESHOLD = ObjectiveVec::Constant(
      param.REWARD_VEC_SIZE, std::numeric_limits<ObjectiveVec::Scalar>::max());

  return param;
}

class DeepSeaTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sea_map_.emplace_back(MODSMapElement{1, 1.0f});
    sea_map_.emplace_back(MODSMapElement{2, 2.0f});
    sea_map_.emplace_back(MODSMapElement{3, 3.0f});
    sea_map_.emplace_back(MODSMapElement{4, 5.0f});
    sea_map_.emplace_back(MODSMapElement{4, 8.0f});
    sea_map_.emplace_back(MODSMapElement{4, 16.0f});
    sea_map_.emplace_back(MODSMapElement{7, 24.0f});
    sea_map_.emplace_back(MODSMapElement{7, 50.0f});
    sea_map_.emplace_back(MODSMapElement{8, 74.0f});
    sea_map_.emplace_back(MODSMapElement{10, 124.0f});
    init_pos << 0, 0;
   mvmcts_parameters_ = MakeDefaultMctsParameters();
   mvmcts_parameters_.REWARD_VEC_SIZE = 2;
   mvmcts_parameters_.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(2);
   mvmcts_parameters_.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(2);
   mvmcts_parameters_.uct_statistic.LOWER_BOUND << 0.0f, -20.0f;
   mvmcts_parameters_.uct_statistic.UPPER_BOUND << 124.0f, 0.0f;
  }

  SeaMap sea_map_;
  Eigen::Vector2i init_pos;
  MvmctsParameters mvmcts_parameters_;
};

void PrintSolution(std::vector<Eigen::Vector2i> &pos_history, SeaMap &sea_map) {
  int num_cols = sea_map.size();
  int num_rows =
      std::max_element(sea_map.begin(), sea_map.end(),
                       [](MODSMapElement a, MODSMapElement b) -> bool {
                         return (a.row < b.row);
                       })
          ->row +
      1;
  std::ostringstream out;
  out << std::fixed;
  out.precision(0);
  for (int row = 0; row < num_rows; row++) {
    out << "|";
    for (int col = 0; col < num_cols; col++) {
      if (std::find(pos_history.begin(), pos_history.end(),
                    Eigen::Vector2i(row, col)) != pos_history.end()) {
        out << " O ";
      } else if (row < sea_map[col].row) {
        out << "   ";
      } else if (row == sea_map[col].row) {
        out << std::setw(3) << sea_map[col].reward;
      } else {
        out << " X ";
      }
      out << "|";
    }
    out << "\n";
    for (int col = 0; col < num_cols; col++) {
      out << "----";
    }
    out << "\n";
  }
  std::cout << out.str() << std::endl;
}

TEST_F(DeepSeaTest, move) {
  MoDeepSeaState init_state(sea_map_, init_pos);
  std::vector<Reward> rewards(1,
                              Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
  JointAction jt;
  jt.resize(1);
  jt[0] = 3;  // Right
  std::shared_ptr<MoDeepSeaState> state = init_state.Execute(jt, rewards);
  ASSERT_EQ(state->GetEgoPos(), Eigen::Vector2i(0, 1));

  jt[0] = 0;  // Up
  state = state->Execute(jt, rewards);
  // Should stay in the same cell
  ASSERT_EQ(state->GetEgoPos(), Eigen::Vector2i(0, 1));

  jt[0] = 1;  // Down
  state = state->Execute(jt, rewards);
  ASSERT_EQ(state->GetEgoPos(), Eigen::Vector2i(1, 1));

  jt[0] = 3;  // Right
  state = state->Execute(jt, rewards);
  ASSERT_EQ(state->GetEgoPos(), Eigen::Vector2i(1, 2));

  jt[0] = 2;  // Left
  state = state->Execute(jt, rewards);
  ASSERT_EQ(state->GetEgoPos(), Eigen::Vector2i(1, 1));

  jt[0] = 1;  // Down
  state = state->Execute(jt, rewards);
  ASSERT_EQ(state->GetEgoPos(), Eigen::Vector2i(2, 1));
  ASSERT_TRUE(state->IsTerminal());
  ASSERT_EQ(rewards[0](0), sea_map_[1].reward);
}

TEST_F(DeepSeaTest, general) {
  FLAGS_v = 1;
  Mvmcts<MoDeepSeaState, UctStatistic<>, UctStatistic<>, RandomHeuristic> mcts(
     mvmcts_parameters_);
  auto state = std::make_shared<MoDeepSeaState>(sea_map_, init_pos);
  std::vector<Reward> rewards(1,
                              Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
  JointAction jt;
  jt.resize(1);
  std::vector<Eigen::Vector2i> pos_history;
  pos_history.clear();
  pos_history.emplace_back(state->GetEgoPos());
  while (!state->IsTerminal()) {
    mcts.Search(*state, 10000, 5000);
    jt[0] = mcts.ReturnBestAction()[0];
    state = state->Execute(jt, rewards);
    pos_history.emplace_back(state->GetEgoPos());
  }
  std::cout << "Positions:" << std::endl;
  for (auto p : pos_history) {
    std::cout << p << ", ";
  }
  std::cout << std::endl;
  std::cout << "END" << std::endl;
  PrintSolution(pos_history, sea_map_);
  //  std::cout << mcts.GetRoot()->PrintNode();
  EXPECT_EQ(pos_history.size(), 20);
  EXPECT_EQ((pos_history.end() - 1)->operator()(0), 10);
  EXPECT_EQ((pos_history.end() - 1)->operator()(1), 9);
}

TEST_F(DeepSeaTest, e_greedy) {
  FLAGS_v = 1;
  Mvmcts<MoDeepSeaState, ThresGreedyStatistic, ThresGreedyStatistic,
       RandomHeuristic>
      mcts(mvmcts_parameters_);
  auto state = std::make_shared<MoDeepSeaState>(sea_map_, init_pos);
  std::vector<Reward> rewards(1,
                              Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
  JointAction jt;
  jt.resize(1);
  std::vector<Eigen::Vector2i> pos_history;
  pos_history.clear();
  pos_history.emplace_back(state->GetEgoPos());
  while (!state->IsTerminal()) {
    mcts.Search(*state, 10000, 5000);
    jt[0] = mcts.ReturnBestAction()[0];
    state = state->Execute(jt, rewards);
    pos_history.emplace_back(state->GetEgoPos());
  }
  std::cout << "Positions:" << std::endl;
  for (auto p : pos_history) {
    std::cout << p << ", ";
  }
  std::cout << std::endl;
  std::cout << "END" << std::endl;
  PrintSolution(pos_history, sea_map_);
  //  std::cout << mcts.GetRoot()->PrintNode();
  EXPECT_EQ(pos_history.size(), 20);
  EXPECT_EQ((pos_history.end() - 1)->operator()(0), 10);
  EXPECT_EQ((pos_history.end() - 1)->operator()(1), 9);
}