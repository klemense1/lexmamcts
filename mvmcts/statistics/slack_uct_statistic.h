//
// Created by Luis Gressenbuch on 31.10.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MVMCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
#define MVMCTS_STATISTICS_SLACK_UCT_STATISTIC_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "mvmcts/statistics/threshold_comperator.h"
#include "mvmcts/statistics/uct_statistic.h"

namespace mvmcts {

class SlackUCTStatistic : public UctStatistic<SlackUCTStatistic> {
  typedef NodeStatistic<SlackUCTStatistic> ParentType;

 public:
  SlackUCTStatistic(ActionIdx num_actions,
                    MvmctsParameters const mvmcts_parameters)
      : UctStatistic<SlackUCTStatistic>(num_actions,mvmcts_parameters) {}

  ActionIdx GetBestAction() {
    ObjectiveVec slack;
    std::vector<ObjectiveVec> qval;
    std::transform(ucb_statistics_.begin(), ucb_statistics_.end(),
                   std::back_inserter(qval),
                   [](const ActionUCBMap::value_type &elem) {
                     return elem.second.action_value_;
                   });
    CalculateSlackValues(qval, &slack);
    ThresholdComparator<ObjectiveVec> comp(slack);
    VLOG(1) << "Slack: " << slack;
    auto max = std::max_element(ucb_statistics_.begin(), ucb_statistics_.end(),
                                [comp](ActionUCBMap::value_type const &a,
                                       ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return comp(a.second.action_value_,
                                                b.second.action_value_);
                                  }
                                });
    return max->first;
  }

  template <class S>
  ActionIdx ChooseNextAction(const S &state,
                             std::vector<int> &unexpanded_actions,
                             unsigned int iteration) {
    ActionIdx selected_action;
    if (unexpanded_actions.empty()) {
      std::uniform_real_distribution<double> uniform_norm(0.0, 1.0);
      std::vector<Eigen::VectorXd> values;
      Eigen::VectorXd slack;
      CalculateUcbValues(ucb_statistics_, values);
      CalculateSlackValues(values, &slack);
      selected_action = std::distance(
          values.begin(),
          std::max_element(values.begin(), values.end(),
                           ThresholdComparator<Eigen::VectorXd>(slack)));
      const double p = uniform_norm(random_generator_);
      if (p <mvmcts_parameters_.thres_uct_statistic_.EPSILON &&
          num_actions_ >= 2) {
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

 protected:
  template <class T>
  void CalculateSlackValues(const std::vector<T> &qval, T *values) const {
    values->resize(qval.size());
    *values = T::Constant(mvmcts_parameters_.REWARD_VEC_SIZE,
                          -std::numeric_limits<typename T::Scalar>::infinity());
    for (ActionIdx idx = 0; idx < qval.size(); ++idx) {
      *values = values->cwiseMax(qval.at(idx));
    }
    // After C. Li and K. Czarnecki, “Urban Driving with Multi-Objective Deep
    // Reinforcement Learning,” arXiv:1811.08586 [cs], Nov. 2018.
    *values = (1.0 -mvmcts_parameters_.slack_uct_statistic_.SLACK_FACTOR) *
              values->cwiseAbs();
  }
};
}  // namespace mvmcts
#endif  // MVMCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
