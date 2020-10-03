// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_MVMCTS_H_
#define MVMCTS_MVMCTS_H_

#define PLAN_DEBUG_INFO

#include <chrono>  // for high_resolution_clock
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mvmcts/common.h"
#include "mvmcts/heuristic.h"
#include "mvmcts/stage_node.h"
namespace mvmcts {

/*
 * @tparam S State Interface
 * @tparam SE Selection & Expandsion Strategy Ego
 * @tparam SO Selection & Expansion Strategy Others
 * @tparam SR Strategy Rollout
 */

template <class S, class SE, class SO, class H>
class Mvmcts {
 public:
  using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;
  using StageNodeWPtr = std::weak_ptr<StageNode<S, SE, SO, H>>;

  explicit Mvmcts(MvmctsParameters const mvmcts_parameters)
      : root_(),
        num_iterations(0),
        heuristic_(mvmcts_parameters),
       mvmcts_parameters_(mvmcts_parameters) {
    DVLOG(2) <<mvmcts_parameters_;
  }

  ~Mvmcts() {}

  void Search(const S &current_state, unsigned int max_search_time_ms,
              unsigned int max_iterations);
  int NumIterations();
  std::string NodeInfo();
  JointAction ReturnBestAction();
  void PrintTreeToDotFile(std::string filename = "tree");

  void SetHeuristicFunction(const H &heuristic) { heuristic_ = heuristic; }
  const StageNodeSPtr &GetRoot() const;

 private:
  void Iterate(const StageNodeSPtr &root_node);

  std::string Print(const StageNodeSPtr &root_node) const;

  StageNodeSPtr root_;

  unsigned int num_iterations;

  H heuristic_;

  MvmctsParameters mvmcts_parameters_;

  MVMCTS_TEST
};

template <class S, class SE, class SO, class H>
void Mvmcts<S, SE, SO, H>::Search(const S &current_state,
                                unsigned int max_search_time_ms,
                                unsigned int max_iterations) {
  namespace chr = std::chrono;
  DLOG(INFO) << "Max search samples: " << max_iterations;
  auto start = std::chrono::high_resolution_clock::now();

  StageNode<S, SE, SO, H>::ResetCounter();

#ifdef PLAN_DEBUG_INFO
  LOG(INFO) << "starting state: " << current_state.PrintState();
#endif
  LOG_IF(FATAL, current_state.IsTerminal())
      << "Cannot search from terminal state! Aborting!";
  root_ = std::make_shared<StageNode<S, SE, SO, H>, StageNodeSPtr,
                           std::shared_ptr<S>, const JointAction &,
                           const unsigned int &>(
      nullptr, current_state.Clone(), JointAction(), 0, mvmcts_parameters_);

  num_iterations = 0;
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - start)
                 .count() < max_search_time_ms &&
         num_iterations < max_iterations) {
    Iterate(root_);
    num_iterations += 1;
  }
  VLOG(1) << "Search time: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::high_resolution_clock::now() - start)
                 .count()
          << " ms"
          << ", Samples: " << this->NumIterations();
}

template <class S, class SE, class SO, class H>
void Mvmcts<S, SE, SO, H>::Iterate(const StageNodeSPtr &root_node) {
  StageNodeSPtr node = root_node;
  StageNodeSPtr node_p;

  // --------------Select & Expand  -----------------
  // We descend the tree for all joint actions already available -> last node is
  // the newly expanded one
  while (node->SelectOrExpand(node, num_iterations)) {}
  // -------------- Heuristic Update ----------------
  // Heuristic until terminal node
  std::vector<SE> calculated_heuristic = heuristic_.GetHeuristicValues(node);
  node->UpdateStatistics(calculated_heuristic);

  // --------------- Backpropagation ----------------
  // Backpropagate starting from parent node of newly expanded node
  node_p = node->GetParent().lock();
  while (true) {
    node_p->UpdateStatistics(node);
    if (node_p->IsRoot()) {
      break;
    } else {
      node = node->GetParent().lock();
      node_p = node_p->GetParent().lock();
    }
  }

#ifdef PLAN_DEBUG_INFO
  Print(root_node);
#endif
}

template <class S, class SE, class SO, class H>
std::string Mvmcts<S, SE, SO, H>::Print(const StageNodeSPtr &root_node) const {
  std::stringstream ss;
  return ss.str();
}

template <class S, class SE, class SO, class H>
int Mvmcts<S, SE, SO, H>::NumIterations() {
  return this->num_iterations;
}

template <class S, class SE, class SO, class H>
std::string Mvmcts<S, SE, SO, H>::NodeInfo() {
  return Print(root_);
}

template <class S, class SE, class SO, class H>
JointAction Mvmcts<S, SE, SO, H>::ReturnBestAction() {
  return root_->GetBestAction();
}

template <class S, class SE, class SO, class H>
void Mvmcts<S, SE, SO, H>::PrintTreeToDotFile(std::string filename) {
  root_->PrintTree(filename, 100);
}
template <class S, class SE, class SO, class H>
const std::shared_ptr<StageNode<S, SE, SO, H>> &Mvmcts<S, SE, SO, H>::GetRoot()
    const {
  return root_;
}

}  // namespace mvmcts

#endif  // MVMCTS_MVMCTS_H_
