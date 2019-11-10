//
// Created by Luis Gressenbuch on 07.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_MCTS_STATISTICS_THRES_UCT_STATISTIC_H_
#define MAMCTS_MCTS_STATISTICS_THRES_UCT_STATISTIC_H_

#include <vector>
#include <string>

#include "glog/logging.h"

#include "mcts/statistics/uct_statistic.h"

namespace mcts {

bool threshold_compare(Eigen::VectorXf const &a, Eigen::VectorXf const &b, Eigen::VectorXf const &thr) {
  assert(a.rows() == b.rows() && a.rows() == thr.rows());
  for (auto ai = a.begin(), bi = b.begin(), thri = thr.begin(); ai != a.end(); ++ai, ++bi, ++thri) {
    if ((*ai >= *thri && *bi >= *thri) || (*ai == *bi)) {
      continue;
    } else {
      return *ai < *bi;
    }
  }
  return false;
}

class ThresUCTStatistic : public UctStatistic<ThresUCTStatistic> {
  typedef NodeStatistic<ThresUCTStatistic> ParentType;

 public:
  ThresUCTStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<ThresUCTStatistic>(
      num_actions,
      mcts_parameters){}

  ActionIdx get_best_action() {
    auto max = std::max_element(ucb_statistics_.begin(),
                                ucb_statistics_.end(),
                                [this](ActionUCBMap::value_type const &a, ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return threshold_compare(a.second.action_value_,
                                                             b.second.action_value_,
                                                             mcts_parameters_.thres_uct_statistic_.THRESHOLD);
                                  }
                                });
    return max->first;
  }
};
}  // namespace mcts

#endif //MAMCTS_MCTS_STATISTICS_THRES_UCT_STATISTIC_H_
