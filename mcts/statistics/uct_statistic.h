// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef UCT_STATISTIC_H
#define UCT_STATISTIC_H

#include "mcts/mcts.h"
#include <iostream>
#include <iomanip>
#include <type_traits>

namespace mcts {

struct {
    bool operator()(double const &a, double const &b) {
        return (round(a) < round(b));
    }
} StableComp;

typedef struct UcbPair {
  UcbPair() : action_count_(0), action_value_(ObjectiveVec::Zero()) {};
  unsigned action_count_;
  ObjectiveVec action_value_;
} UcbPair;
typedef std::map<ActionIdx, UcbPair> ActionUCBMap;

class NodeStatistic_Final_Impl;

// A upper confidence bound implementation
template<typename IMPL = NodeStatistic_Final_Impl>
class UctStatistic :
    public std::conditional<std::is_same<IMPL, NodeStatistic_Final_Impl>::value,
                            NodeStatistic<UctStatistic<>>, NodeStatistic<IMPL>>::type, public mcts::RandomGenerator {
 private:
  typedef typename std::conditional<std::is_same<IMPL, NodeStatistic_Final_Impl>::value,
                                    UctStatistic<>,
                                    UctStatistic<IMPL>>::type this_type;
  typedef typename std::conditional<std::is_same<IMPL, NodeStatistic_Final_Impl>::value,
                                    NodeStatistic<UctStatistic<>>, NodeStatistic<IMPL>>::type parent_type;

 public:
  MCTS_TEST

  UctStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) :
      parent_type(num_actions, mcts_parameters),
      value_(),
      latest_return_(),
      ucb_statistics_([&]() -> ActionUCBMap {
        ActionUCBMap map;
        for (ActionIdx ai = 0; ai < num_actions; ++ai) { map[ai] = UcbPair(); }
        return map;
      }()),
      total_node_visits_(0) {};

  ~UctStatistic() {};

  template<class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      // Select an action based on the UCB formula
      std::vector<Eigen::VectorXf> values;
      calculate_ucb_values(ucb_statistics_, values);
      // find largest index
      ActionIdx selected_action = std::distance(values.begin(), std::max_element(values.begin(), values.end(),
                                                                                 [](Eigen::VectorXf &a,
                                                                                    Eigen::VectorXf &b) -> bool {
                                                                                   return std::lexicographical_compare(a.begin(),
                                                                                                                       a.end(),
                                                                                                                       b.begin(),
                                                                                                                       b.end(),
                                                                                                                       StableComp);
                                                                                 }));
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
    auto max = std::max_element(ucb_statistics_.begin(), ucb_statistics_.end(),
                                [](ActionUCBMap::value_type &a, ActionUCBMap::value_type &b) -> bool {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return std::lexicographical_compare(a.second.action_value_.begin(),
                                                                        a.second.action_value_.end(),
                                                                        b.second.action_value_.begin(),
                                                                        b.second.action_value_.end(),
                                                                        StableComp);
                                  }
                                  // Original final selection policy from paper
                                  // return (a.second.action_count_ < b.second.action_count_);
                                }
    );
    return max->
        first;
  }

  void update_from_heuristic(const parent_type &heuristic_statistic) {
    const this_type &heuristic_statistic_impl = heuristic_statistic.impl();
    value_ = heuristic_statistic_impl.value_;
    latest_return_ = value_;
    MCTS_EXPECT_TRUE(total_node_visits_ == 0); // This should be the first visit
    total_node_visits_ += 1;
  }

  void update_statistic(const parent_type &changed_child_statistic) {
    const this_type &changed_uct_statistic = changed_child_statistic.impl();

    //Action Value update step
    UcbPair &ucb_pair =
        ucb_statistics_[this->collected_reward_.first]; // we remembered for which action we got the reward, must be the same as during backprop, if we linked parents and childs correctly
    //action value: Q'(s,a) = Q'(s,a) + (latest_return - Q'(s,a))/N
    latest_return_ = this->collected_reward_.second + mcts_parameters_.DISCOUNT_FACTOR * changed_uct_statistic.latest_return_;
    ucb_pair.action_count_ += 1;
    ucb_pair.action_value_ =
        ucb_pair.action_value_ + (latest_return_ - ucb_pair.action_value_) / ucb_pair.action_count_;

    total_node_visits_ += 1;
    value_ = value_ + (latest_return_ - value_) / total_node_visits_;
  }

  void set_heuristic_estimate(const Reward &accum_rewards) {
    value_ = accum_rewards;
  }

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
      ss << "a=" << int(action) << ", N=" << action_it->second.action_count_ << ", V="
         << action_it->second.action_value_.transpose();
    }
    return ss.str();
  }

 protected:

  void calculate_ucb_values(const ActionUCBMap &ucb_statistics, std::vector<Eigen::VectorXf> &values) const {
    values.resize(ucb_statistics.size());

    for (size_t idx = 0; idx < ucb_statistics.size(); ++idx) {
      Eigen::VectorXf
          action_value_normalized =
          (ucb_statistics.at(idx).action_value_ - mcts_parameters_.uct_statistic.LOWER_BOUND).cwiseQuotient(mcts_parameters_.uct_statistic.UPPER_BOUND - mcts_parameters_.uct_statistic.LOWER_BOUND);
      //MCTS_EXPECT_TRUE(action_value_normalized >= 0);
      //MCTS_EXPECT_TRUE(action_value_normalized <= 1);
      values[idx] = action_value_normalized.array()
          + 2 * k_exploration_constant * sqrt((2 * log(total_node_visits_)) / (ucb_statistics.at(idx).action_count_));
    }
  }

  ObjectiveVec value_;
  ObjectiveVec latest_return_;   // tracks the return during backpropagation
  ActionUCBMap ucb_statistics_; // first: action selection count, action-value
  unsigned int total_node_visits_;

};

} // namespace mcts

#endif