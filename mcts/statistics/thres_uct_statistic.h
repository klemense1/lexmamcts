//
// Created by Luis Gressenbuch on 07.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MCTS_STATISTICS_THRES_UCT_STATISTIC_H_
#define MCTS_STATISTICS_THRES_UCT_STATISTIC_H_

#include <string>
#include <vector>

#include "glog/logging.h"

#include "mcts/statistics/uct_statistic.h"
#include "mcts/statistics/threshold_comperator.h"

namespace mcts {

class ThresUCTStatistic : public UctStatistic<ThresUCTStatistic> {
  typedef NodeStatistic<ThresUCTStatistic> ParentType;

 public:
  ThresUCTStatistic(ActionIdx num_actions,
                    MctsParameters const &mcts_parameters)
      : UctStatistic<ThresUCTStatistic>(num_actions, mcts_parameters) {}

  template <class S>
  ActionIdx choose_next_action(const S &state,
                               std::vector<int> &unexpanded_actions,
                               unsigned int iteration) {
    ActionIdx selected_action;
    if (unexpanded_actions.empty()) {
      std::uniform_real_distribution<double> uniform_norm(0.0, 1.0);
      std::vector<Eigen::VectorXd> values;
      auto thres = mcts_parameters_.thres_uct_statistic_.THRESHOLD.cast<double>();
      auto lower = mcts_parameters_.uct_statistic.LOWER_BOUND.cast<double>();
      auto upper = mcts_parameters_.uct_statistic.UPPER_BOUND.cast<double>();
      Eigen::VectorXd normalized_thresholds = (thres - lower).cwiseQuotient(upper-lower);
      calculate_ucb_values(ucb_statistics_, values);
      selected_action = std::distance(
          values.begin(), std::max_element(values.begin(), values.end(),
                                           ThresholdComparator<Eigen::VectorXd>(
                                               normalized_thresholds)));
      const double p = uniform_norm(random_generator_);
      if (p < mcts_parameters_.thres_uct_statistic_.EPSILON && num_actions_ >= 2) {
        std::uniform_int_distribution<ActionIdx> uniform_action(0, num_actions_ - 2);
        ActionIdx random_action = uniform_action(random_generator_);
        selected_action = (random_action >= selected_action ? random_action + 1 : random_action);
      }
    } else {
      // Select randomly an unexpanded action
      std::uniform_int_distribution<ActionIdx> random_action_selection(
          0, unexpanded_actions.size() - 1);
      ActionIdx array_idx = random_action_selection(random_generator_);
      selected_action = unexpanded_actions[array_idx];
      unexpanded_actions.erase(unexpanded_actions.begin() + array_idx);
    }
    return selected_action;
  }

  ActionIdx get_best_action() {
    Reward thr = mcts_parameters_.thres_uct_statistic_.THRESHOLD;
    DVLOG(2) << "Thresholds:" << thr.transpose();
    auto max = std::max_element(ucb_statistics_.begin(), ucb_statistics_.end(),
                                [thr](ActionUCBMap::value_type const &a,
                                      ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return (ThresholdComparator<Eigen::VectorXf>(thr))(a.second.action_value_,
                                        b.second.action_value_);
                                  }
                                });
    return max->first;
  }
};
}  // namespace mcts

#endif  // MCTS_STATISTICS_THRES_UCT_STATISTIC_H_
