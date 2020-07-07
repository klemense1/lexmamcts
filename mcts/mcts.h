// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_MCTS_H_
#define MCTS_MCTS_H_

#ifdef DUMP_Q_VAL
extern std::string Q_VAL_DUMPFILE;
#endif

#define PLAN_DEBUG_INFO

#include <easy/profiler.h>
#include <chrono>  // for high_resolution_clock
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mcts/common.h"
#include "mcts/heuristic.h"
#include "mcts/stage_node.h"
namespace mcts {

/*
 * @tparam S State Interface
 * @tparam SE Selection & Expandsion Strategy Ego
 * @tparam SO Selection & Expansion Strategy Others
 * @tparam SR Strategy Rollout
 */

template <class S, class SE, class SO, class H>
class Mcts {
 public:
  using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;
  using StageNodeWPtr = std::weak_ptr<StageNode<S, SE, SO, H>>;

  explicit Mcts(MctsParameters const &mcts_parameters)
      : root_(),
        num_iterations(0),
        heuristic_(mcts_parameters),
        mcts_parameters_(mcts_parameters) {
    DVLOG(2) << mcts_parameters_;
  }

  ~Mcts() {}

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

  MctsParameters const mcts_parameters_;

  MCTS_TEST
};

template <class S, class SE, class SO, class H>
void Mcts<S, SE, SO, H>::Search(const S &current_state,
                                unsigned int max_search_time_ms,
                                unsigned int max_iterations) {
  EASY_FUNCTION();
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
      nullptr, current_state.Clone(), JointAction(), 0, mcts_parameters_);

  num_iterations = 0;
#ifdef DUMP_Q_VAL
  std::ofstream ofs;
  ofs.open(Q_VAL_DUMPFILE);
  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t");
#endif
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - start)
                 .count() < max_search_time_ms &&
         num_iterations < max_iterations) {
    Iterate(root_);
    num_iterations += 1;
#ifdef DUMP_Q_VAL
    if (num_iterations % 1 == 0) {
      ofs << num_iterations << "\t";
      for (auto const &pair :
           root_->get_ego_int_node().get_expected_rewards()) {
        ofs << pair.second.transpose().format(fmt) << "\t";
      }
      ofs << root_->get_best_action()[0] << "\n";
    }
#endif
  }
#ifdef DUMP_Q_VAL
  ofs.close();
#endif
  VLOG(1) << "Search time: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::high_resolution_clock::now() - start)
                 .count()
          << " ms"
          << ", Samples: " << this->NumIterations();
}

template <class S, class SE, class SO, class H>
void Mcts<S, SE, SO, H>::Iterate(const StageNodeSPtr &root_node) {
  EASY_FUNCTION();
  StageNodeSPtr node = root_node;
  StageNodeSPtr node_p;

  // --------------Select & Expand  -----------------
  // We descend the tree for all joint actions already available -> last node is
  // the newly expanded one
  EASY_BLOCK("selection");
  while (node->SelectOrExpand(node, num_iterations)) {}
  EASY_END_BLOCK;
  // -------------- Heuristic Update ----------------
  // Heuristic until terminal node
  std::vector<SE> calculated_heuristic = heuristic_.GetHeuristicValues(node);
  EASY_BLOCK("backpropagation");
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
  EASY_END_BLOCK;

#ifdef PLAN_DEBUG_INFO
  Print(root_node);
#endif
}

template <class S, class SE, class SO, class H>
std::string Mcts<S, SE, SO, H>::Print(const StageNodeSPtr &root_node) const {
  std::stringstream ss;
  return ss.str();
}

template <class S, class SE, class SO, class H>
int Mcts<S, SE, SO, H>::NumIterations() {
  return this->num_iterations;
}

template <class S, class SE, class SO, class H>
std::string Mcts<S, SE, SO, H>::NodeInfo() {
  return Print(root_);
}

template <class S, class SE, class SO, class H>
JointAction Mcts<S, SE, SO, H>::ReturnBestAction() {
  return root_->GetBestAction();
}

template <class S, class SE, class SO, class H>
void Mcts<S, SE, SO, H>::PrintTreeToDotFile(std::string filename) {
  root_->PrintTree(filename, 100);
}
template <class S, class SE, class SO, class H>
const std::shared_ptr<StageNode<S, SE, SO, H>> &Mcts<S, SE, SO, H>::GetRoot()
    const {
  return root_;
}

}  // namespace mcts

#endif  // MCTS_MCTS_H_
