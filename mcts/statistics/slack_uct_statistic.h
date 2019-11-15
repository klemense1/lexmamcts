//
// Created by Luis Gressenbuch on 31.10.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
#define MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_

#include <vector>
#include <string>

#include "glog/logging.h"
#include "boost/math/distributions/students_t.hpp"

#include "mcts/statistics/uct_statistic.h"

namespace mcts {

class SlackUCTStatistic : public UctStatistic<SlackUCTStatistic> {
  typedef NodeStatistic<SlackUCTStatistic> ParentType;

 public:

  struct SlackComperator {
    SlackComperator(const std::vector<ObjectiveVec> &slack) : slack_(slack) {}
    bool operator()(const ActionUCBMap::value_type &a, const ActionUCBMap::value_type &b) const {
      assert(a.second.action_value_.rows() == b.second.action_value_.rows()
                 && a.second.action_value_.rows() == slack_[a.first].rows()
                 && b.second.action_value_.rows() == slack_[b.first].rows());
      Eigen::VectorXf a_upper = a.second.action_value_ + slack_[a.first];
      Eigen::VectorXf a_lower = a.second.action_value_ - slack_[a.first];
      Eigen::VectorXf b_upper = b.second.action_value_ + slack_[b.first];
      Eigen::VectorXf b_lower = b.second.action_value_ - slack_[b.first];
      for (int i = 0; i < a_upper.rows(); ++i) {
        if (a_upper(i) < b_lower(i)) {
          return true;
        } else if (b_upper(i) < a_lower(i)) {
          return false;
        }
      }
      return false;
    }
    const std::vector<ObjectiveVec> slack_;
  };

  SlackUCTStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<SlackUCTStatistic>(
      num_actions,
      mcts_parameters), m_2_(num_actions, ObjectiveVec::Zero()) {}

  ActionIdx get_best_action() {
    // Lexicographical ordering of the UCT value vectors
    std::vector<ObjectiveVec> slack;
    calculate_slack_values(ucb_statistics_, slack);
    SlackComperator comp(slack);
    VLOG(1) << "Slack: " << slack;
    auto max = std::max_element(ucb_statistics_.begin(),
                                ucb_statistics_.end(),
                                [comp](ActionUCBMap::value_type const &a, ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return comp(a, b);
                                  }
                                });
    return max->first;
  }

  void update_statistic(const ParentType &changed_child_statistic) {
    const SlackUCTStatistic &changed_uct_statistic = changed_child_statistic.impl();
    // Action Value update step
    UcbPair &ucb_pair = ucb_statistics_[this->collected_reward_.first];
    // we remembered for which action we got the reward,
    // must be the same as during backprop,
    // if we linked parents and childs correctly
    // action value: Q'(s,a) = Q'(s,a) + (latest_return - Q'(s,a))/N
    Reward reward =
        this->collected_reward_.second + this->mcts_parameters_.DISCOUNT_FACTOR * changed_uct_statistic.latest_return_;
    size_t action_count = ucb_pair.action_count_ + 1;
    ObjectiveVec delta = reward - ucb_pair.action_value_;
    ObjectiveVec action_value = ucb_pair.action_value_ + (reward - ucb_pair.action_value_) / action_count;
    ObjectiveVec delta2 = reward - action_value;
    // Recursive variance calculation
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    m_2_.at(this->collected_reward_.first) += delta.cwiseProduct(delta2);
    UctStatistic<SlackUCTStatistic>::update_statistic(changed_child_statistic);
  }

  std::string print_edge_information(const ActionIdx &action) const {
    std::stringstream ss;
    ss << UctStatistic<SlackUCTStatistic>::print_edge_information(action);
    auto action_it = ucb_statistics_.find(action);
    if (action_it != ucb_statistics_.end()) {
      ss << ", sigma="
         << (get_reward_variance(action)).cwiseSqrt().transpose();
    }
    return ss.str();
  }

 protected:
  void calculate_slack_values(const ActionUCBMap &ucb_statistics, std::vector<ObjectiveVec> &values) const {
    values.resize(ucb_statistics.size());
    for (ActionIdx idx = 0; idx < ucb_statistics.size(); ++idx) {
      UcbPair pair = ucb_statistics.at(idx);
      // Students t distribution with n-1 degrees of freedom
      VLOG_IF(1, pair.action_count_ < 2) << "action_count < 2 -> falling back to lexicographical compare";
      if (pair.action_count_ >= 2) {
        boost::math::students_t dist(pair.action_count_ - 1);
        double t = quantile(complement(dist, mcts_parameters_.slack_uct_statistic_.ALPHA / 2.0));
        // Standard deviation according to
        // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
        ObjectiveVec std_dev = (get_reward_variance(idx)).cwiseSqrt();
        // confidence interval radius
        values[idx] = t * std_dev / sqrt(static_cast<double>(pair.action_count_));
      } else {
        values[idx] = ObjectiveVec::Zero();
      }
    }
  }

 private:
  ObjectiveVec get_reward_variance(ActionIdx action_idx) const {
    const UcbPair &pair = ucb_statistics_.at(action_idx);
    return m_2_.at(action_idx) / static_cast<double>(pair.action_count_);
  }
  std::vector<ObjectiveVec> m_2_;
};
}  // namespace mcts
#endif  // MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
