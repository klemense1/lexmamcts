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

bool slack_compare(Eigen::VectorXf const &a,
                   Eigen::VectorXf const &b,
                   Eigen::VectorXf const &slack_a,
                   Eigen::VectorXf const &slack_b) {
  assert(a.rows() == b.rows() && a.rows() == slack_a.rows() && b.rows() == slack_b.rows());
  for (auto ai = a.begin(), bi = b.begin(), sai = slack_a.begin(), sbi = slack_b.begin(); ai != a.end();
       ++ai, ++bi, ++sai, ++sbi) {
    if ((*ai + *sai) < (*bi - *sbi)) {
      return true;
    } else if ((*bi + *sbi) < (*ai - *sai)) {
      return false;
    }
  }
  // Approximate Equal
  // Fall back to lexicographic ordering
  return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
}

class SlackUCTStatistic : public UctStatistic<SlackUCTStatistic> {
  typedef NodeStatistic<SlackUCTStatistic> ParentType;

 public:
  SlackUCTStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<SlackUCTStatistic>(
      num_actions,
      mcts_parameters), m_2_(num_actions, ObjectiveVec::Zero()) {}

  ActionIdx get_best_action() {
    // Lexicographical ordering of the UCT value vectors
    std::vector<ObjectiveVec> slack;
    calculate_slack_values(ucb_statistics_, slack);
    VLOG(1) << "Slack: " << slack;
    auto max = std::max_element(ucb_statistics_.begin(),
                                ucb_statistics_.end(),
                                [slack](ActionUCBMap::value_type const &a, ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return slack_compare(a.second.action_value_,
                                                         b.second.action_value_,
                                                         slack[a.first],
                                                         slack[b.first]);
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
         << (m_2_.at(action) / static_cast<double>(action_it->second.action_count_)).cwiseSqrt().transpose();
    }
    return ss.str();
  }

 protected:
  void calculate_slack_values(const ActionUCBMap &ucb_statistics, std::vector<ObjectiveVec> &values) const {
    values.resize(ucb_statistics.size());
    double const ALPHA = 0.05;
    for (ActionIdx idx = 0; idx < ucb_statistics.size(); ++idx) {
      UcbPair pair = ucb_statistics.at(idx);
      // Students t distribution with n-1 degrees of freedom
      boost::math::students_t dist(pair.action_count_ - 1);
      double t = quantile(complement(dist, ALPHA / 2));
      // Standard deviation according to
      // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
      ObjectiveVec std_dev = (m_2_.at(idx) / static_cast<double>(pair.action_count_)).cwiseSqrt();
      // confidence interval radius
      values[idx] = t * std_dev / sqrt(static_cast<double>(pair.action_count_));
    }
  }

 private:
  std::vector<ObjectiveVec> m_2_;
};
}  // namespace mcts
#endif  // MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
