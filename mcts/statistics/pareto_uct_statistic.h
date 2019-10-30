//
// Created by luis on 30.10.19.
//

#ifndef MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_
#define MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_

#include "mcts/mcts.h"
#include "mcts/statistics/pareto_set.h"
#include <iostream>
#include <iomanip>
namespace mcts {

class ParetoUCTStatistic : public mcts::NodeStatistic<ParetoUCTStatistic>, RandomGenerator {

 public:
  MCTS_TEST

  ParetoUCTStatistic(ActionIdx num_actions) :
      NodeStatistic<ParetoUCTStatistic>(num_actions),
      value_(),
      latest_return_(),
      ucb_statistics_([&]() -> ActionUCBMap {
        ActionUCBMap map;
        for (auto ai = 0; ai < num_actions; ++ai) { map[ai] = UcbPair(); }
        return map;
      }()),
      total_node_visits_(0),
      upper_bound(mcts::MctsParameters::UPPER_BOUND),
      lower_bound(mcts::MctsParameters::LOWER_BOUND),
      k_discount_factor(mcts::MctsParameters::DISCOUNT_FACTOR),
      k_exploration_constant(mcts::MctsParameters::EXPLORATION_CONSTANT) {};

  ~ParetoUCTStatistic() {};

  template<class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      // Select an action based on the UCB formula
      std::vector<Eigen::VectorXf> values;
      calculate_ucb_values(ucb_statistics_, values);
      ParetoSet<ActionIdx, Eigen::VectorXf> pareto_set;
      pareto_set.add(values);
      ActionIdx selected_action = pareto_set.get_random();
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

  void update_from_heuristic(const NodeStatistic<ParetoUCTStatistic> &heuristic_statistic) {
    const ParetoUCTStatistic &heuristic_statistic_impl = heuristic_statistic.impl();
    value_ = heuristic_statistic_impl.value_;
    latest_return_ = value_;
    MCTS_EXPECT_TRUE(total_node_visits_ == 0); // This should be the first visit
    total_node_visits_ += 1;
  }

  void update_statistic(const NodeStatistic<ParetoUCTStatistic> &changed_child_statistic) {
    const ParetoUCTStatistic &changed_uct_statistic = changed_child_statistic.impl();

    //Action Value update step
    UcbPair &ucb_pair =
        ucb_statistics_[collected_reward_.first]; // we remembered for which action we got the reward, must be the same as during backprop, if we linked parents and childs correctly
    //action value: Q'(s,a) = Q'(s,a) + (latest_return - Q'(s,a))/N
    latest_return_ = collected_reward_.second + k_discount_factor * changed_uct_statistic.latest_return_;
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
  typedef struct UcbPair {
    UcbPair() : action_count_(0), action_value_(ObjectiveVec::Zero()) {};
    unsigned action_count_;
    ObjectiveVec action_value_;
  } UcbPair;
  typedef std::map<ActionIdx, UcbPair> ActionUCBMap;

  void calculate_ucb_values(const ActionUCBMap &ucb_statistics, std::vector<Eigen::VectorXf> &values) const {
    values.resize(ucb_statistics.size());

    for (size_t idx = 0; idx < ucb_statistics.size(); ++idx) {
      Eigen::VectorXf
          action_value_normalized =
          (ucb_statistics.at(idx).action_value_ - lower_bound).cwiseQuotient(upper_bound - lower_bound);
      //MCTS_EXPECT_TRUE(action_value_normalized >= 0);
      //MCTS_EXPECT_TRUE(action_value_normalized <= 1);
      values[idx] = action_value_normalized.array()
          + sqrt((4 * log(total_node_visits_) + log(REWARD_DIM)) / (2 * ucb_statistics.at(idx).action_count_));
    }
  }

  ObjectiveVec value_;
  ObjectiveVec latest_return_;   // tracks the return during backpropagation
  ActionUCBMap ucb_statistics_; // first: action selection count, action-value
  unsigned int total_node_visits_;

// PARAMS
  const ObjectiveVec upper_bound;
  const ObjectiveVec lower_bound;
  const double k_discount_factor;
  const double k_exploration_constant;
};

}

#endif //MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_
