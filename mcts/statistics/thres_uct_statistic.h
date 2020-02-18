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

namespace mcts {

class ThresUCTStatistic : public UctStatistic<ThresUCTStatistic> {
  typedef NodeStatistic<ThresUCTStatistic> ParentType;

 public:
  ThresUCTStatistic(ActionIdx num_actions,
                    MctsParameters const &mcts_parameters)
      : UctStatistic<ThresUCTStatistic>(num_actions, mcts_parameters) {}
  template <class T>
  struct ThresholdComparator {
    explicit ThresholdComparator(const T &thr) : thr_(thr) {}
    const T thr_;
    bool operator()(const T &a, const T &b) const {
      assert(a.rows() == b.rows() && a.rows() == thr_.rows());
      for (auto ai = a.begin(), bi = b.begin(), thri = thr_.begin();
           ai != a.end(); ++ai, ++bi, ++thri) {
        if ((*ai > *thri && *bi > *thri) || (*ai == *bi)) {
          continue;
        } else {
          return *ai < *bi;
        }
      }
      return false;
    }
  };

  template <class S>
  ActionIdx choose_next_action(const S &state,
                               std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      std::uniform_real_distribution<double> uniform_norm(0.0, 1.0);
      std::vector<Eigen::VectorXd> values;
      calculate_ucb_values(ucb_statistics_, values);
      ActionIdx selected_action = std::distance(
          values.begin(), std::max_element(values.begin(), values.end(),
                                           ThresholdComparator<Eigen::VectorXd>(
                                               mcts_parameters_.thres_uct_statistic_.THRESHOLD.cast<double>())));
      if (uniform_norm(random_generator_) >= mcts_parameters_.e_greedy_uct_statistic_.EPSILON) {
        // Select an action based on the UCB formula
        return selected_action;
      } else {
        std::uniform_int_distribution<ActionIdx> uniform_action(0, num_actions_ - 2);
        ActionIdx random_action = uniform_action(random_generator_);
        return (random_action == selected_action ? random_action + 1 : random_action);
      }
    } else {
      // Select randomly an unexpanded action
      std::uniform_int_distribution<ActionIdx> random_action_selection(
          0, unexpanded_actions.size() - 1);
      ActionIdx array_idx = random_action_selection(random_generator_);
      ActionIdx selected_action = unexpanded_actions[array_idx];
      unexpanded_actions.erase(unexpanded_actions.begin() + array_idx);
      return selected_action;
    }
  }

  ActionIdx get_best_action() {
    Eigen::MatrixXf mat(this->mcts_parameters_.REWARD_VEC_SIZE,
                        ucb_statistics_.size());
    for (ActionIdx i = 0; i < static_cast<ActionIdx>(mat.cols()); ++i) {
      mat.col(i) = ucb_statistics_.find(i)->second.action_value_;
    }

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
