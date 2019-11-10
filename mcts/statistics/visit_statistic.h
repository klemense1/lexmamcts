//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_MCTS_STATISTICS_VISIT_STATISTIC_H_
#define MAMCTS_MCTS_STATISTICS_VISIT_STATISTIC_H_

#include <vector>
#include <string>

#include "glog/logging.h"
#include "boost/math/distributions/students_t.hpp"

#include "mcts/statistics/uct_statistic.h"

namespace mcts {

class VisitStatistic : public UctStatistic<VisitStatistic> {
  typedef NodeStatistic<VisitStatistic> ParentType;

 public:
  VisitStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<VisitStatistic>(
      num_actions,
      mcts_parameters) {}

  ActionIdx get_best_action() {
    auto max = std::max_element(ucb_statistics_.begin(),
                                ucb_statistics_.end(),
                                [](ActionUCBMap::value_type const &a, ActionUCBMap::value_type const &b) {
                                  return (a.second.action_count_ < b.second.action_count_);
                                });
    return max->first;
  }
};
}  // namespace mcts

#endif //MAMCTS_MCTS_STATISTICS_VISIT_STATISTIC_H_
