// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_STAGE_NODE_H_
#define MVMCTS_STAGE_NODE_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/functional/hash.hpp"
#include "glog/logging.h"

#include "mvmcts/common.h"
#include "mvmcts/intermediate_node.h"
#include "mvmcts/mvmcts_parameters.h"
#include "mvmcts/node_statistic.h"
#include "mvmcts/state.h"

namespace mvmcts {

// hash function to use JoinAction as std::unordered map key
template <typename Container>
struct container_hash {
  std::size_t operator()(Container const &c) const {
    return boost::hash_range(c.begin(), c.end());
  }
};

typedef std::unordered_map<JointAction, JointReward,
                           container_hash<JointAction>>
    StageRewardMap;

/*
 * S: State Model
 * SE: Statistics Ego Agent
 * SO: Statistics Other Agents, e.g. probabilistic opponent models
 * H: Heuristic Model
 */
template <class S, class SE, class SO, class H>
class StageNode : public std::enable_shared_from_this<StageNode<S, SE, SO, H>> {
 private:
  using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;
  using StageNodeWPtr = std::weak_ptr<StageNode<S, SE, SO, H>>;
  typedef std::unordered_map<JointAction, StageNodeSPtr,
                             container_hash<JointAction>>
      StageChildMap;
  //< remembers joint rewards
  // of state execute to avoid rerunning Execute during node selection

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

  const JointAction joint_action_;  // action_idx leading to this node
  const unsigned int max_num_joint_actions_;
  const unsigned int id_;
  const unsigned int depth_;

  MvmctsParameters mvmcts_parameters_;

  static unsigned int num_nodes_;

 public:
  StageNode(const StageNodeSPtr &parent, std::shared_ptr<S> state,
            const JointAction &joint_action, const unsigned int &depth,
            MvmctsParameters const mvmcts_parameters);
  ~StageNode();
  bool SelectOrExpand(StageNodeSPtr &next_node, unsigned int iteration);
  void UpdateStatistics(const std::vector<SE> &heuristic_estimates);
  void UpdateStatistics(const StageNodeSPtr &changed_child_node);
  bool EachAgentsActionsExpanded();
  bool EachJointActionExpanded();
  StageNodeSPtr GetShared();
  const S *GetState() const { return state_.get(); }
  StageNodeWPtr GetParent() { return parent_; }
  bool IsRoot() const { return !parent_.lock(); }
  JointAction GetBestAction();

  std::string PrintNode() const;
  void PrintTree(std::string filename, const unsigned int &max_depth = 5);
  void PrintLayer(std::string filename, const unsigned int &max_depth);
  const StageChildMap &GetChildren() const;
  JointReward GetQFunc(JointAction const &joint_action);
  JointReward GetValue();
  const StageRewardMap &GetJointRewards() const;

  static void ResetCounter();
  const IntermediateNode<S, SE> &GetEgoIntNode() const;

  MVMCTS_TEST
};

template <class S, class SE, class SO, class H>
using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;

template <class S, class SE, class SO, class H>
StageNode<S, SE, SO, H>::StageNode(const StageNodeSPtr &parent,
                                   std::shared_ptr<S> state,
                                   const JointAction &joint_action,
                                   const unsigned int &depth,
                                   MvmctsParameters const mvmcts_parameters)
    : state_(state),
      parent_(parent),
      children_(),
      joint_rewards_(),
      ego_int_node_(*state_, S::ego_agent_idx,
                    state_->GetNumActions(S::ego_agent_idx),mvmcts_parameters),
      other_int_nodes_([this,mvmcts_parameters]() -> InterNodeVector {
        // Initialize the intermediate nodes of other agents
        InterNodeVector vec;
        // vec.resize(state_.GetAgentIdx().size()-1);
        for (AgentIdx ai = S::ego_agent_idx + 1;
             ai < AgentIdx(state_->GetAgentIdx().size()); ++ai) {
          vec.emplace_back(*state_, ai, state_->GetNumActions(ai),
                          mvmcts_parameters);
        }
        return vec;
      }()),
      joint_action_(joint_action),
      max_num_joint_actions_([this]() -> unsigned int {
        ActionIdx num_actions(state_->GetNumActions(S::ego_agent_idx));
        for (auto ai = S::ego_agent_idx + 1;
             ai < AgentIdx(state_->GetAgentIdx().size()); ++ai) {
          num_actions *= state_->GetNumActions(ai);
        }
        return num_actions;
      }()),
      id_(++num_nodes_),
      depth_(depth),
     mvmcts_parameters_(mvmcts_parameters) {}

template <class S, class SE, class SO, class H>
StageNode<S, SE, SO, H>::~StageNode() = default;

template <class S, class SE, class SO, class H>
StageNodeSPtr<S, SE, SO, H> StageNode<S, SE, SO, H>::GetShared() {
  return this->shared_from_this();
}

template <class S, class SE, class SO, class H>
bool StageNode<S, SE, SO, H>::SelectOrExpand(StageNodeSPtr &next_node,
                                             unsigned int iteration) {
  // helper function to fill rewards
  auto FillRewards = [this](const std::vector<Reward> &reward_list,
                            const JointAction &ja) {
    Reward coop_sum = Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE);
    coop_sum =
        std::accumulate(reward_list.begin(), reward_list.end(), coop_sum);
    coop_sum = coop_sum * mvmcts_parameters_.COOP_FACTOR;
    ego_int_node_.CollectReward((coop_sum + (1 -mvmcts_parameters_.COOP_FACTOR) *
                                                reward_list[S::ego_agent_idx]),
                                ja[S::ego_agent_idx]);
    for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end();
         ++it) {
      it->CollectReward((coop_sum + (1 -mvmcts_parameters_.COOP_FACTOR) *
                                        reward_list[it->GetAgentIdx()]),
                        ja[it->GetAgentIdx()]);
    }
  };

  // First check if state of node is terminal
  if (this->GetState()->IsTerminal()) {
    next_node = GetShared();
    return false;
  }

  // Let each agent select an action according to its statistic model -> yields
  // joint_action
  JointAction joint_action(state_->GetAgentIdx().size());
  joint_action[ego_int_node_.GetAgentIdx()] =
      ego_int_node_.ChooseNextAction(iteration);
  for (auto &it : other_int_nodes_) {
    joint_action[it.GetAgentIdx()] = it.ChooseNextAction(iteration);
  }

  // Check if joint action was already expanded
  auto it = children_.find(joint_action);
  if (it != children_.end()) {
    // SELECT EXISTING NODE
    next_node = it->second;
    FillRewards(joint_rewards_[joint_action], joint_action);
    ++joint_action_counter_[joint_action];
    return true;
  } else {  // EXPAND NEW NODE BASED ON NEW JOINT ACTION
    std::vector<Reward> rewards(state_->GetAgentIdx().size(),
                                Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
    next_node = std::make_shared<StageNode<S, SE, SO, H>, StageNodeSPtr,
                                 std::shared_ptr<S>, const JointAction &,
                                 const unsigned int &>(
        GetShared(), state_->Execute(joint_action, rewards), joint_action,
        depth_ + 1,mvmcts_parameters_);
    children_[joint_action] = next_node;
    joint_action_counter_[joint_action] = 0;
#ifdef PLAN_DEBUG_INFO
    //     std::cout << "expanded node state: " << state_->Execute(joint_action,
    //     rewards)->PrintState();
#endif
    if (next_node->GetState()->IsTerminal()) {
      rewards += next_node->GetState()->GetTerminalReward();
    }
    // collect intermediate rewards and selected action indexes
    FillRewards(rewards, joint_action);
    joint_rewards_[joint_action] = rewards;

    return false;
  }
}

template <class S, class SE, class SO, class H>
unsigned int StageNode<S, SE, SO, H>::num_nodes_ = 0;

template <class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::ResetCounter() {
  StageNode<S, SE, SO, H>::num_nodes_ = 0;
}

template <class S, class SE, class SO, class H>
bool StageNode<S, SE, SO, H>::EachJointActionExpanded() {
  return children_.size() == max_num_joint_actions_;
}

template <class S, class SE, class SO, class H>
bool StageNode<S, SE, SO, H>::EachAgentsActionsExpanded() {
  if (!ego_int_node_.AllActionsExpanded()) {
    return false;
  }

  for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
    if (!*it->all_actions_expanded()) {
      return false;
    }
  }
}

template <class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::UpdateStatistics(
    const std::vector<SE> &heuristic_estimates) {
  ego_int_node_.UpdateFromHeuristic(heuristic_estimates[S::ego_agent_idx]);
  for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
    it->UpdateFromHeuristic(heuristic_estimates[it->GetAgentIdx()]);
  }
}

template <class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::UpdateStatistics(
    const StageNodeSPtr &changed_child_node) {
  ego_int_node_.UpdateStatistic(changed_child_node->ego_int_node_);
  for (auto it = other_int_nodes_.begin(); it != other_int_nodes_.end(); ++it) {
    it->UpdateStatistic(
        changed_child_node
            ->other_int_nodes_[it->GetAgentIdx() -
                               1]);  // -1: Ego Agent is at zero, but not
                                     // contained in other int nodes
  }
}

template <class S, class SE, class SO, class H>
JointAction StageNode<S, SE, SO, H>::GetBestAction() {
  JointAction best(other_int_nodes_.size() + 1);
  best[0] = ego_int_node_.GetBestAction();
  int i = 1;
  for (auto &int_node : other_int_nodes_) {
    best[i] = int_node.GetBestAction();
    ++i;
  }
  VLOG(1) << "Ego:" << std::endl;
  for (ActionIdx i = 0; i < state_->GetNumActions(S::ego_agent_idx); ++i) {
    VLOG(1) << ego_int_node_.PrintEdgeInformation(i);
  }
  for (AgentIdx agent_idx = S::ego_agent_idx + 1;
       agent_idx < state_->GetAgentIdx().size(); ++agent_idx) {
    VLOG(1) << "Other " << static_cast<size_t>(agent_idx) << ":";
    for (ActionIdx i = 0; i < state_->GetNumActions(agent_idx); ++i) {
      VLOG(1) << other_int_nodes_[static_cast<size_t>(agent_idx) - 1]
                     .PrintEdgeInformation(i);
    }
  }
  return best;
}

template <class S, class SE, class SO, class H>
std::string StageNode<S, SE, SO, H>::PrintNode() const {
  auto tabs = [](const unsigned int &depth) -> std::string {
    return std::string(depth, '\t');
  };

  std::stringstream ss;
  ss << tabs(depth_) << "StageNode: ID " << id_;

  if (!joint_action_.empty()) {
    ss << ", Joint Action " << joint_action_;
  }
  ss << ", " << state_->PrintState() << ", Stats: { (0) "
     << ego_int_node_.sprintf();
  for (int i = 0; i < other_int_nodes_.size(); ++i) {
    ss << ", (" << i + 1 << ") " << other_int_nodes_[i].PrintNode();
  }
  ss << "}" << std::endl;

  if (!children_.empty()) {
    for (auto it = children_.begin(); it != children_.end(); ++it)
      ss << it->second->PrintNode();
  }
  return ss.str();
}

template <class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::PrintTree(std::string filename,
                                        const unsigned int &max_depth) {
  std::ofstream outfile(filename + ".gv");
  outfile << "digraph G {" << std::endl;
  outfile << "label = \"MCTS with Exploration constant = "
          <<mvmcts_parameters_.uct_statistic.EXPLORATION_CONSTANT << "\";"
          << std::endl;
  outfile << "labelloc = \"t\";" << std::endl;
  outfile.close();

  this->PrintLayer(filename, max_depth);

  outfile.open(filename + ".gv", std::ios::app);
  outfile << "}" << std::endl;
  outfile.close();
}

template <class S, class SE, class SO, class H>
void StageNode<S, SE, SO, H>::PrintLayer(std::string filename,
                                         const unsigned int &max_depth) {
  if (depth_ > max_depth) {
    return;
  }

  std::ofstream logging;
  logging.open(filename + ".gv", std::ios::app);
  // DRAW SUBGRAPH FOR THIS STAGE
  logging << "subgraph cluster_node_" << this->id_ << "{" << std::endl;
  logging << "node" << this->id_ << "_" << int(ego_int_node_.GetAgentIdx())
          << "[label=\"" << ego_int_node_.PrintNodeInformation() << " \n Ag."
          << int(ego_int_node_.GetAgentIdx()) << "\"]"
          << ";" << std::endl;
  for (auto other_agent_it = other_int_nodes_.begin();
       other_agent_it != other_int_nodes_.end(); ++other_agent_it) {
    logging << "node" << this->id_ << "_" << int(other_agent_it->GetAgentIdx())
            << "[label=\"" << other_agent_it->PrintNodeInformation()
            << " \n Ag." << int(other_agent_it->GetAgentIdx()) << "\"]"
            << ";" << std::endl;
  }
  logging << "label= \"ID " << this->id_ << ", "
          << "terminal: " << state_->IsTerminal() << "\";" << std::endl;
  logging << "graph[style=dotted]; }" << std::endl;

  // DRAW ARROWS FOR EACH CHILD
  for (auto child_it = this->children_.begin();
       child_it != this->children_.end(); ++child_it) {
    child_it->second->PrintLayer(filename, max_depth);

    // ego intermediate node
    logging << "node" << this->id_ << "_" << int(ego_int_node_.GetAgentIdx())
            << " -> "
            << "node" << child_it->second->id_ << "_"
            << int(ego_int_node_.GetAgentIdx()) << "[label=\""
            << ego_int_node_.PrintEdgeInformation(
                   ActionIdx(child_it->first[ego_int_node_.GetAgentIdx()]))
            << "\"]"
            << ";" << std::endl;
    // other intermediate nodes
    for (auto other_int_it = other_int_nodes_.begin();
         other_int_it != other_int_nodes_.end(); ++other_int_it) {
      logging << "node" << this->id_ << "_" << int(other_int_it->GetAgentIdx())
              << " -> "
              << "node" << child_it->second->id_ << "_"
              << int(other_int_it->GetAgentIdx()) << "[label=\""
              << other_int_it->PrintEdgeInformation(
                     ActionIdx(child_it->first[other_int_it->GetAgentIdx()]))
              << "\"]"
              << ";" << std::endl;
    }
  }
}
template <class S, class SE, class SO, class H>
const std::unordered_map<JointAction, std::shared_ptr<StageNode<S, SE, SO, H>>,
                         container_hash<JointAction>>
    &StageNode<S, SE, SO, H>::GetChildren() const {
  return children_;
}
template <class S, class SE, class SO, class H>
JointReward StageNode<S, SE, SO, H>::GetQFunc(JointAction const &joint_action) {
  JointReward v(state_->get_agent_idx().size(),
                Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
  ActionIdx action = joint_action.at(0);
  v.at(0) = ego_int_node_.get_expected_rewards().at(action);
  for (size_t i = 1; i < v.size(); ++i) {
    action = joint_action.at(i);
    v.at(i) = other_int_nodes_.at(i - 1).get_expected_rewards().at(action);
  }
  return v;
}
template <class S, class SE, class SO, class H>
JointReward StageNode<S, SE, SO, H>::GetValue() {
  JointReward v(other_int_nodes_.size() + 1);
  v.at(0) = dynamic_cast<SE &>(ego_int_node_).GetValue();
  for (size_t i = 1; i < other_int_nodes_.size(); ++i) {
    v.at(i) = dynamic_cast<SO &>(other_int_nodes_.at(i)).GetValue();
  }
  return v;
}
template <class S, class SE, class SO, class H>
const StageRewardMap &StageNode<S, SE, SO, H>::GetJointRewards() const {
  return joint_rewards_;
}
template <class S, class SE, class SO, class H>
const IntermediateNode<S, SE> &StageNode<S, SE, SO, H>::GetEgoIntNode() const {
  return ego_int_node_;
}

}  // namespace mvmcts

#endif  // MVMCTS_STAGE_NODE_H_
