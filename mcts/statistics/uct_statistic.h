// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef UCT_STATISTIC_H
#define UCT_STATISTIC_H

#include <cfloat>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include "mcts/mcts.h"

namespace mcts {

typedef struct UcbPair {
  UcbPair(size_t reward_vec_size) : action_count_(0), action_value_(ObjectiveVec::Zero(reward_vec_size)){};
  unsigned action_count_;
  ObjectiveVec action_value_;
} UcbPair;
typedef std::map<ActionIdx, UcbPair> ActionUCBMap;

class NodeStatistic_Final_Impl;

// A upper confidence bound implementation
template <typename IMPL = NodeStatistic_Final_Impl>
class UctStatistic : public std::conditional<std::is_same<IMPL, NodeStatistic_Final_Impl>::value,
                                             NodeStatistic<UctStatistic<>>, NodeStatistic<IMPL>>::type,
                     public mcts::RandomGenerator {
 private:
  typedef typename std::conditional<std::is_same<IMPL, NodeStatistic_Final_Impl>::value, UctStatistic<>,
                                    UctStatistic<IMPL>>::type ThisType;
  typedef typename std::conditional<std::is_same<IMPL, NodeStatistic_Final_Impl>::value, NodeStatistic<UctStatistic<>>,
                                    NodeStatistic<IMPL>>::type ParentType;

 public:
  MCTS_TEST

  struct LexicographicalComperator {
    const double eps_ = 1e-5;
    template <class T>
    bool operator()(const T &a, const T &b) const {
      return std::lexicographical_compare(
          a.begin(), a.end(), b.begin(), b.end(), [this](const typename T::Scalar &a, const typename T::Scalar &b) {
            return (b - a) > ((std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) * eps_);
          });
    }
  };

  UctStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters)
      : ParentType(num_actions, mcts_parameters),
        value_(ObjectiveVec::Zero(mcts_parameters.REWARD_VEC_SIZE)),
        latest_return_(ObjectiveVec::Zero(mcts_parameters.REWARD_VEC_SIZE)),
        ucb_statistics_([&]() -> ActionUCBMap {
          ActionUCBMap map;
          for (ActionIdx ai = 0; ai < num_actions; ++ai) {
            map.insert({ai, UcbPair(mcts_parameters.REWARD_VEC_SIZE)});
          }
          return map;
        }()),
        total_node_visits_(0){};

  ~UctStatistic(){};

  template <class S>
  ActionIdx choose_next_action(const S &state,
                               std::vector<int> &unexpanded_actions,
                               unsigned int iteration) {
    if (unexpanded_actions.empty() || pw_limit_reached(unexpanded_actions)) {
      // Select an action based on the UCB formula
      std::vector<Eigen::VectorXd> values;
      calculate_ucb_values(ucb_statistics_, values);
      // find largest index
      ActionIdx selected_action =
          std::distance(values.begin(), std::max_element(values.begin(), values.end(), LexicographicalComperator()));
      return selected_action;
    } else {
      // Select randomly an unexpanded action
      std::uniform_int_distribution<ActionIdx> random_action_selection(0, unexpanded_actions.size() - 1);
      ActionIdx array_idx = random_action_selection(random_generator_);
      ActionIdx selected_action = unexpanded_actions[array_idx];
      unexpanded_actions.erase(unexpanded_actions.begin() + array_idx);
      return selected_action;
    }
  }

  ActionIdx get_best_action() {
    // Lexicographical ordering of the UCT value vectors
    LexicographicalComperator lex_comp;
    auto max = std::max_element(ucb_statistics_.begin(), ucb_statistics_.end(),
                                [lex_comp](ActionUCBMap::value_type const &a, ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return lex_comp(a.second.action_value_, b.second.action_value_);
                                  }
                                });
    return max->first;
  }

  void update_from_heuristic(const ParentType &heuristic_statistic) {
    const ThisType &heuristic_statistic_impl = heuristic_statistic.impl();
    value_ = heuristic_statistic_impl.value_;
    latest_return_ = value_;
    MCTS_EXPECT_TRUE(total_node_visits_ == 0);  // This should be the first visit
    total_node_visits_ += 1;
  }

  void update_statistic(const ParentType &changed_child_statistic) {
    const ThisType &changed_uct_statistic = changed_child_statistic.impl();

    // Action Value update step
    auto ucb_pair = ucb_statistics_.find(
        this->collected_reward_.first);  // we remembered for which action we got the reward, must be the same as during
                                         // backprop, if we linked parents and childs correctly
    // action value: Q'(s,a) = Q'(s,a) + (latest_return - Q'(s,a))/N
    latest_return_ =
        this->collected_reward_.second + this->mcts_parameters_.DISCOUNT_FACTOR * changed_uct_statistic.latest_return_;
    ucb_pair->second.action_count_ += 1;
    ucb_pair->second.action_value_ = ucb_pair->second.action_value_ +
                                     (latest_return_ - ucb_pair->second.action_value_) / ucb_pair->second.action_count_;
    total_node_visits_ += 1;
    value_ = value_ + (latest_return_ - value_) / total_node_visits_;
  }

  void set_heuristic_estimate(const Reward &accum_rewards) { value_ = accum_rewards; }

 public:
  std::string print_node_information() const {
    std::stringstream ss;
    ss << std::setprecision(2) << "V=" << value_ << ", N=" << total_node_visits_;
    return ss.str();
  }

  std::string print_edge_information(const ActionIdx &action) const {
    std::stringstream ss;
    auto action_it = ucb_statistics_.find(action);
    if (action_it != ucb_statistics_.end()) {
      ss << "a=" << int(action) << ", N=" << action_it->second.action_count_
         << ", V=" << action_it->second.action_value_.transpose();
    }
    return ss.str();
  }

  std::map<ActionIdx, Reward> get_expected_rewards() const {
    std::map<ActionIdx, Reward> v;
    for (auto const &pair : ucb_statistics_) {
      v[pair.first] = pair.second.action_value_;
    }
    return v;
  }
  const ObjectiveVec get_value() const { return value_; }

 protected:
  void calculate_ucb_values(const ActionUCBMap &ucb_statistics, std::vector<Eigen::VectorXd> &values) const {
    values.resize(ucb_statistics.size());
    auto upper_bound = this->mcts_parameters_.uct_statistic.UPPER_BOUND.template cast<double>();
    auto lower_bound = this->mcts_parameters_.uct_statistic.LOWER_BOUND.template cast<double>();
    const auto scale = this->mcts_parameters_.uct_statistic.EXPLORATION_CONSTANT * (upper_bound - lower_bound);
    double exploration_term;
    Eigen::VectorXd exploration_offset;
    for (size_t idx = 0; idx < ucb_statistics.size(); ++idx) {
      exploration_term = sqrt((2.0 * log(total_node_visits_)) / (ucb_statistics.at(idx).action_count_));
      exploration_offset = scale * exploration_term + lower_bound;
      values[idx] = ucb_statistics.at(idx).action_value_.template cast<double>() + exploration_offset;
    }
  }

  inline bool pw_limit_reached(const std::vector<int> &unexpanded_actions) const {
    return ((ucb_statistics_.size() - unexpanded_actions.size()) >
            std::floor(
                std::pow(total_node_visits_ + 1, this->mcts_parameters_.uct_statistic.PROGRESSIVE_WIDENING_ALPHA))) &&
           this->mcts_parameters_.uct_statistic.PROGRESSIVE_WIDENING_ENABLED;
  }

  ObjectiveVec value_;
  ObjectiveVec latest_return_;   // tracks the return during backpropagation
  ActionUCBMap ucb_statistics_;  // first: action selection count, action-value
  unsigned int total_node_visits_;
};

}  // namespace mcts

#endif
