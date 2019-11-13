//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_MCTS_STATISTICS_MAX_UCT_STATISTIC_H_
#define MAMCTS_MCTS_STATISTICS_MAX_UCT_STATISTIC_H_

#include <vector>
#include <string>

#include "mcts/statistics/uct_statistic.h"

namespace mcts {

class MaxUCTStatistic : public UctStatistic<MaxUCTStatistic> {
  typedef NodeStatistic<MaxUCTStatistic> ParentType;

 public:
  MaxUCTStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<MaxUCTStatistic>(
      num_actions, mcts_parameters) {
    for (auto &pair : ucb_statistics_) {
      pair.second.action_value_ = mcts_parameters.uct_statistic.LOWER_BOUND;
    }
  }

  void update_statistic(const ParentType &changed_child_statistic) {
    const MaxUCTStatistic &changed_uct_statistic = changed_child_statistic.impl();

    //Action Value update step
    UcbPair &ucb_pair =
        ucb_statistics_[this->collected_reward_.first]; // we remembered for which action we got the reward, must be the same as during backprop, if we linked parents and childs correctly
    //action value: Q'(s,a) = Q'(s,a) + (latest_return - Q'(s,a))/N
    latest_return_ =
        this->collected_reward_.second + this->mcts_parameters_.DISCOUNT_FACTOR * changed_uct_statistic.latest_return_;
    ucb_pair.action_count_ += 1;
    ucb_pair.action_value_ = std::lexicographical_compare(ucb_pair.action_value_.begin(),
                                                          ucb_pair.action_value_.end(),
                                                          latest_return_.begin(),
                                                          latest_return_.end()) ? latest_return_
                                                                                : ucb_pair.action_value_;
    total_node_visits_ += 1;
    value_ = std::lexicographical_compare(value_.begin(), value_.end(), latest_return_.begin(), latest_return_.end())
             ? latest_return_ : value_;;
  }
};
}  // namespace mcts

#endif //MAMCTS_MCTS_STATISTICS_MAX_UCT_STATISTIC_H_
