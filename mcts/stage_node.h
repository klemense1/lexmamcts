// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_STAGE_NODE_H
#define MCTS_STAGE_NODE_H

#include <memory>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <easy/profiler.h>

#include "boost/functional/hash.hpp"
#include "glog/logging.h"

#include "common.h"
#include "state.h"
#include "intermediate_node.h"
#include "node_statistic.h"
#include "mcts_parameters.h"

namespace mcts {

// hash function to use JoinAction as std::unordered map key
template<typename Container>
struct container_hash {
  std::size_t operator()(Container const &c) const {
    return boost::hash_range(c.begin(), c.end());
  }
};

typedef std::unordered_map<JointAction, JointReward, container_hash<JointAction>> StageRewardMap;

/*
    * S: State Model
    * SE: Statistics Ego Agent
    * SO: Statistics Other Agents, e.g. probabilistic opponent models
    * H: Heuristic Model
    */
template<class S, class SE, class SO, class H>
class StageNode : public std::enable_shared_from_this<StageNode<S, SE, SO, H>> {
 private:
  using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;
  using StageNodeWPtr = std::weak_ptr<StageNode<S, SE, SO, H>>;
  typedef std::unordered_map<JointAction, StageNodeSPtr, container_hash<JointAction>> StageChildMap;
  //< remembers joint rewards
  //of state execute to avoid rerunning execute during node selection

  // Environment State
  std::shared_ptr<S> state_;

  // Parents and children
  StageNodeWPtr parent_;
  StageChildMap children_;
  StageRewardMap joint_rewards_;
  std::unordered_map<JointAction, size_t, container_hash<JointAction>>
      joint_action_counter_;

  // Intermediate decision nodes
  IntermediateNode<S, SE> ego_int_node_;
  typedef std::vector<IntermediateNode<S, SO>> InterNodeVector;
  InterNodeVector other_int_nodes_;

  const JointAction joint_action_; // action_idx leading to this node
  const unsigned int max_num_joint_actions_;
  const unsigned int id_;
  const unsigned int depth_;

  MctsParameters const &mcts_parameters_;

  static unsigned int num_nodes_;

 public:
  StageNode(const StageNodeSPtr &parent,
            std::shared_ptr<S> state,
            const JointAction &joint_action,
            const unsigned int &depth,
            MctsParameters const &mcts_parameters);
  ~StageNode();
  bool select_or_expand(StageNodeSPtr &next_node, unsigned int iteration);
  void update_statistics(const std::vector<SE> &heuristic_estimates);
  void update_statistics(const StageNodeSPtr &changed_child_node);
  bool each_agents_actions_expanded();
  bool each_joint_action_expanded();
  StageNodeSPtr get_shared();
  const S *get_state() const { return state_.get(); }
  StageNodeWPtr get_parent() { return parent_; }
  bool is_root() const { return !parent_.lock(); }
  JointAction get_best_action();

  std::string sprintf() const;
  void printTree(std::string filename, const unsigned int &max_depth = 5);
  void printLayer(std::string filename, const unsigned int &max_depth);
  const StageChildMap &get_children() const;
  JointReward get_q_func(JointAction const &joint_action);
  JointReward get_value();
  const StageRewardMap &get_joint_rewards() const;

  static void reset_counter();
  const IntermediateNode < S, SE> &get_ego_int_node() const;

  MCTS_TEST
};

template<class S, class SE, class SO, class H> using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;

template<class S, class SE, class SO, class H>
StageNode<S, SE, SO, H>::StageNode(const StageNodeSPtr &parent,
                                   std::shared_ptr<S> state,
                                   const JointAction &joint_action,
                                   const unsigned int &depth,
                                   MctsParameters const &mcts_parameters) :
    state_(state),
    parent_(parent),
    children_(),
    joint_rewards_(),
    ego_int_node_(*state_, S::ego_agent_idx, state_->get_num_actions(S::ego_agent_idx), mcts_parameters),
    other_int_nodes_([this, mcts_parameters]() -> InterNodeVector {
      // Initialize the intermediate nodes of other agents
      InterNodeVector vec;
      // vec.resize(state_.get_agent_idx().size()-1);
      for (AgentIdx ai = S::ego_agent_idx + 1; ai < AgentIdx(state_->get_agent_idx().size()); ++ai) {
        vec.emplace_back(*state_, ai, state_->get_num_actions(ai), mcts_parameters);
      }
      return vec;
    }()),
    joint_action_(joint_action),
    max_num_joint_actions_([this]() -> unsigned int {
      ActionIdx num_actions(state_->get_num_actions(S::ego_agent_idx));
      for (auto ai = S::ego_agent_idx + 1; ai < AgentIdx(state_->get_agent_idx().size()); ++ai) {
        num_actions *= state_->get_num_actions(ai);
      }
      return num_actions;
    }()),
    id_(++num_nodes_),
    depth_(depth),
    mcts_parameters_(mcts_parameters) {};

template<class S, class SE, class SO, class H>
StageNode<S, SE, SO, H>::~StageNode() {
}

template<class S, class SE, class SO, class H>
StageNodeSPtr<S, SE, SO, H> StageNode<S, SE, SO, H>::get_shared() {
  return this->shared_from_this();
}

template<class S, class SE, class SO, class H>
bool StageNode<S, SE, SO, H>::select_or_expand(StageNodeSPtr &next_node,
                                               unsigned int iteration) {
  EASY_FUNCTION();
  // helper function to fill rewards
  auto fill_rewards = [this](const std::vector<Reward> &reward_list, const JointAction &ja) {
    Reward coop_sum = Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE);
    coop_sum = std::accumulate(reward_list.begin(), reward_list.end(), coop_sum);
    coop_sum = coop_sum * mcts_parameters_.COOP_FACTOR;
    ego_int_node_.collect_reward(
        (coop_sum +
         (1 - mcts_parameters_.COOP_FACTOR) * reward_list[S::ego_agent_idx]),
        ja[S::ego_agent_idx]);
    for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
      it->collect_reward((coop_sum + (1 - mcts_parameters_.COOP_FACTOR) *
                                         reward_list[it->get_agent_idx()]),
                         ja[it->get_agent_idx()]);
    }
  };

  // First check if state of node is terminal
  if (this->get_state()->is_terminal()) {
    next_node = get_shared();
    return false;
  }

  // Let each agent select an action according to its statistic model -> yields joint_action
  JointAction joint_action(state_->get_agent_idx().size());
  joint_action[ego_int_node_.get_agent_idx()] =
      ego_int_node_.choose_next_action(iteration);
  for(auto& it : other_int_nodes_) {
    joint_action[it.get_agent_idx()] = it.choose_next_action(iteration);
  }

  // Check if joint action was already expanded
  auto it = children_.find(joint_action);
  if (it != children_.end()) {
    // SELECT EXISTING NODE
    EASY_EVENT("select");
    next_node = it->second;
    fill_rewards(joint_rewards_[joint_action], joint_action);
    ++joint_action_counter_[joint_action];
    return true;
  } else {   // EXPAND NEW NODE BASED ON NEW JOINT ACTION
    EASY_BLOCK("expand");
    std::vector<Reward> rewards(state_->get_agent_idx().size(), Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
    next_node = std::make_shared<StageNode<S, SE, SO, H>,
                                 StageNodeSPtr,
                                 std::shared_ptr<S>,
                                 const JointAction &,
                                 const unsigned int &>(get_shared(),
                                                       state_->execute(joint_action, rewards),
                                                       joint_action,
                                                       depth_ + 1,
                                                       mcts_parameters_);
    EASY_END_BLOCK;
    children_[joint_action] = next_node;
    joint_action_counter_[joint_action] = 0;
#ifdef PLAN_DEBUG_INFO
    //     std::cout << "expanded node state: " << state_->execute(joint_action, rewards)->sprintf();
#endif
    if (next_node->get_state()->is_terminal()) {
      rewards += next_node->get_state()->get_final_reward();
    }
    // collect intermediate rewards and selected action indexes
    fill_rewards(rewards, joint_action);
    joint_rewards_[joint_action] = rewards;

    return false;
  }

}

template<class S, class SE, class SO, class H> unsigned int StageNode<S, SE, SO, H>::num_nodes_ = 0;

template<class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::reset_counter() {
  StageNode<S, SE, SO, H>::num_nodes_ = 0;
}

template<class S, class SE, class SO, class H>
bool StageNode<S, SE, SO, H>::each_joint_action_expanded() {
  return children_.size() == max_num_joint_actions_;

}

template<class S, class SE, class SO, class H>
bool StageNode<S, SE, SO, H>::each_agents_actions_expanded() {
  if (!ego_int_node_.all_actions_expanded()) { return false; }

  for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
    if (!*it->all_actions_expanded()) { return false; }
  }
}

template<class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::update_statistics(const std::vector<SE> &heuristic_estimates) {
  ego_int_node_.update_from_heuristic(heuristic_estimates[S::ego_agent_idx]);
  for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
    it->update_from_heuristic(heuristic_estimates[it->get_agent_idx()]);
  }
}

template<class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::update_statistics(const StageNodeSPtr &changed_child_node) {
  ego_int_node_.update_statistic(changed_child_node->ego_int_node_);
  for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
    it->update_statistic(changed_child_node->other_int_nodes_[it->get_agent_idx()
        - 1]); // -1: Ego Agent is at zero, but not contained in other int nodes
  }
}

template<class S, class SE, class SO, class H>
JointAction StageNode<S, SE, SO, H>::get_best_action() {
  JointAction best(other_int_nodes_.size() + 1);
  best[0] = ego_int_node_.get_best_action();
  int i = 1;
  for (auto &int_node : other_int_nodes_) {
    best[i] = int_node.get_best_action();
    ++i;
  }
  VLOG(1) << "Ego:" << std::endl;
  for (ActionIdx i = 0; i < state_->get_num_actions(S::ego_agent_idx); ++i) {
    VLOG(1) << ego_int_node_.print_edge_information(i);
  }
  for(AgentIdx agent_idx = S::ego_agent_idx + 1; agent_idx < state_->get_agent_idx().size(); ++agent_idx) {
    VLOG(1) << "Other " << static_cast<size_t>(agent_idx) << ":";
    for (ActionIdx i = 0; i < state_->get_num_actions(agent_idx); ++i) {
      VLOG(1) << other_int_nodes_[static_cast<size_t>(agent_idx) - 1].print_edge_information(i);
    }
  }
  return best;
}

template<class S, class SE, class SO, class H>
std::string StageNode<S, SE, SO, H>::sprintf() const {
  auto tabs = [](const unsigned int &depth) -> std::string {
    return std::string(depth, '\t');
  };

  std::stringstream ss;
  ss << tabs(depth_) << "StageNode: ID " << id_;

  if (!joint_action_.empty()) {
    ss << ", Joint Action " << joint_action_;
  }
  ss << ", " << state_->sprintf() << ", Stats: { (0) " << ego_int_node_.sprintf();
  for (int i = 0; i < other_int_nodes_.size(); ++i) {
    ss << ", (" << i + 1 << ") " << other_int_nodes_[i].sprintf();
  }
  ss << "}" << std::endl;

  if (!children_.empty()) {
    for (auto it = children_.begin(); it != children_.end(); ++it)
      ss << it->second->sprintf();

  }
  return ss.str();
};

template<class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::printTree(std::string filename, const unsigned int &max_depth) {
  std::ofstream outfile(filename + ".gv");
  outfile << "digraph G {" << std::endl;
  outfile << "label = \"MCTS with Exploration constant = " << mcts_parameters_.uct_statistic.EXPLORATION_CONSTANT
          << "\";" << std::endl;
  outfile << "labelloc = \"t\";" << std::endl;
  outfile.close();

  this->printLayer(filename, max_depth);

  outfile.open(filename + ".gv", std::ios::app);
  outfile << "}" << std::endl;
  outfile.close();
};

template<class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::printLayer(std::string filename, const unsigned int &max_depth) {
  if (depth_ > max_depth) {
    return;
  }

  std::ofstream logging;
  logging.open(filename + ".gv", std::ios::app);
  // DRAW SUBGRAPH FOR THIS STAGE
  logging << "subgraph cluster_node_" << this->id_ << "{" << std::endl;
  logging << "node" << this->id_ << "_" << int(ego_int_node_.get_agent_idx()) << "[label=\""
          << ego_int_node_.print_node_information() << " \n Ag." << int(ego_int_node_.get_agent_idx()) << "\"]" << ";"
          << std::endl;
  for (auto other_agent_it = other_int_nodes_.begin(); other_agent_it != other_int_nodes_.end(); ++other_agent_it) {
    logging << "node" << this->id_ << "_" << int(other_agent_it->get_agent_idx()) << "[label=\""
            << other_agent_it->print_node_information() << " \n Ag." << int(other_agent_it->get_agent_idx()) << "\"]"
            << ";" << std::endl;
  }
  logging << "label= \"ID " << this->id_ << ", " << "terminal: " << state_->is_terminal() << "\";" << std::endl;
  logging << "graph[style=dotted]; }" << std::endl;

  // DRAW ARROWS FOR EACH CHILD
  for (auto child_it = this->children_.begin(); child_it != this->children_.end(); ++child_it) {
    child_it->second->printLayer(filename, max_depth);

    // ego intermediate node
    logging << "node" << this->id_ << "_" << int(ego_int_node_.get_agent_idx()) << " -> " << "node"
            << child_it->second->id_ << "_" << int(ego_int_node_.get_agent_idx()) << "[label=\""
            << ego_int_node_.print_edge_information(ActionIdx(child_it->first[ego_int_node_.get_agent_idx()])) << "\"]"
            << ";" << std::endl;
    // other intermediate nodes
    for (auto other_int_it = other_int_nodes_.begin(); other_int_it != other_int_nodes_.end(); ++other_int_it) {
      logging << "node" << this->id_ << "_" << int(other_int_it->get_agent_idx()) << " -> " << "node"
              << child_it->second->id_ << "_" << int(other_int_it->get_agent_idx()) << "[label=\""
              << other_int_it->print_edge_information(ActionIdx(child_it->first[other_int_it->get_agent_idx()]))
              << "\"]" << ";" << std::endl;

    }
  }
}
template<class S, class SE, class SO, class H>
const std::unordered_map<JointAction, std::shared_ptr<StageNode<S, SE, SO, H>>, container_hash<JointAction>> &StageNode<
    S,
    SE,
    SO,
    H>::get_children() const {
  return children_;
}
template<class S, class SE, class SO, class H>
JointReward StageNode<S, SE, SO, H>::get_q_func(JointAction const &joint_action) {
  JointReward v(state_->get_agent_idx().size(), Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
  ActionIdx action = joint_action.at(0);
  v.at(0) = ego_int_node_.get_expected_rewards().at(action);
  for (size_t i = 1; i < v.size(); ++i) {
    action = joint_action.at(i);
    v.at(i) = other_int_nodes_.at(i - 1).get_expected_rewards().at(action);
  }
  return v;
}
template<class S, class SE, class SO, class H>
JointReward StageNode<S, SE, SO, H>::get_value() {
  JointReward v(other_int_nodes_.size() + 1);
  v.at(0) = dynamic_cast<SE &>(ego_int_node_).get_value();
  for (size_t i = 1; i < other_int_nodes_.size(); ++i) {
    v.at(i) = dynamic_cast<SO &>(other_int_nodes_.at(i)).get_value();
  }
  return v;
}
template <class S, class SE, class SO, class H>
const StageRewardMap &StageNode<S, SE, SO, H>::get_joint_rewards() const {
  return joint_rewards_;
}
template <class S, class SE, class SO, class H>
    const IntermediateNode < S, SE>
    &StageNode<S, SE, SO, H>::get_ego_int_node() const {
  return ego_int_node_;
}

} // namespace mcts

#endif