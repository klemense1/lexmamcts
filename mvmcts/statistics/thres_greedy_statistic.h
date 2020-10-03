//
// Created by Luis Gressenbuch on 04.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MVMCTS_STATISTICS_THRES_GREEDY_STATISTIC_H_
#define MVMCTS_STATISTICS_THRES_GREEDY_STATISTIC_H_

#include <algorithm>
#include <random>
#include <vector>

#include "mvmcts/mvmcts_parameters.h"
#include "mvmcts/statistics/threshold_comperator.h"
#include "mvmcts/statistics/uct_statistic.h"

namespace mvmcts {
class ThresGreedyStatistic : public UctStatistic<ThresGreedyStatistic> {
 public:
  ThresGreedyStatistic(ActionIdx num_actions,
                       MvmctsParameters const mvmcts_parameters)
      : UctStatistic<ThresGreedyStatistic>(num_actions,mvmcts_parameters) {}

  template <class S>
  ActionIdx ChooseNextAction(const S &state,
                             std::vector<int> &unexpanded_actions,
                             unsigned int iteration) {
    ActionIdx selected_action;
    // TODO(@cirrostratus1): Parameters
    const double c =mvmcts_parameters_.thres_greedy_statistic_.DECAY1;
    const double d =mvmcts_parameters_.thres_greedy_statistic_.DECAY2;
    const double K = num_actions_;
    // From P. Auer, N. Cesa-Bianchi, und P. Fischer,
    // „Finite-time Analysis of the Multiarmed Bandit Problem“
    const double current_epsilon =
        std::min(1.0, c * K / (d * d * static_cast<double>(iteration)));
    if (unexpanded_actions.empty()) {
      std::uniform_real_distribution<double> uniform_norm(0.0, 1.0);
      std::vector<Eigen::VectorXd> values;
      selected_action = GetBestAction();
      const double p = uniform_norm(random_generator_);
      if (p < current_epsilon && num_actions_ >= 2) {
        std::uniform_int_distribution<ActionIdx> uniform_action(
            0, num_actions_ - 2);
        ActionIdx random_action = uniform_action(random_generator_);
        selected_action = (random_action >= selected_action ? random_action + 1
                                                            : random_action);
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

  ActionIdx GetBestAction() {
    Reward thr =mvmcts_parameters_.thres_uct_statistic_.THRESHOLD;
    DVLOG(2) << "Thresholds:" << thr.transpose();
    auto max = std::max_element(
        ucb_statistics_.begin(), ucb_statistics_.end(),
        [thr](ActionUCBMap::value_type const &a,
              ActionUCBMap::value_type const &b) {
          if (a.second.action_count_ == 0) {
            return true;
          } else if (b.second.action_count_ == 0) {
            return false;
          } else {
            return (ThresholdComparator<Eigen::VectorXf>(thr))(
                a.second.action_value_, b.second.action_value_);
          }
        });
    return max->first;
  }
};
}  // namespace mvmcts

#endif  // MVMCTS_STATISTICS_THRES_GREEDY_STATISTIC_H_
