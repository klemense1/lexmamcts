// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_H
#define MCTS_H

#include "stage_node.h"
#include "heuristic.h"
#include <chrono>  // for high_resolution_clock
#include "common.h"
#include <string>

namespace mcts {

/*
 * @tparam S State Interface
 * @tparam SE Selection & Expandsion Strategy Ego
 * @tparam SO Selection & Expansion Strategy Others
 * @tparam SR Strategy Rollout
 */


template<class S, class SE, class SO, class H>
class Mcts {

 public:
  using StageNodeSPtr = std::shared_ptr<StageNode<S, SE, SO, H>>;
  using StageNodeWPtr = std::weak_ptr<StageNode<S, SE, SO, H>>;

  Mcts(MctsParameters const &mcts_parameters)
      : root_(), num_iterations(0), heuristic_(mcts_parameters), mcts_parameters_(mcts_parameters) {
    DVLOG(1) << mcts_parameters_;
  };

  ~Mcts() {}

  void search(const S &current_state, unsigned int max_search_time_ms, unsigned int max_iterations);
  int numIterations();
  std::string nodeInfo();
  JointAction returnBestAction();
  void printTreeToDotFile(std::string filename = "tree");

  void set_heuristic_function(const H &heuristic) { heuristic_ = heuristic; }
  const StageNodeSPtr &get_root() const;

 private:

  void iterate(const StageNodeSPtr &root_node);

  StageNodeSPtr root_;

  unsigned int num_iterations;

  H heuristic_;

  std::string sprintf(const StageNodeSPtr &root_node) const;

  MctsParameters const mcts_parameters_;

  MCTS_TEST
};

template<class S, class SE, class SO, class H>
void Mcts<S, SE, SO, H>::search(const S &current_state, unsigned int max_search_time_ms, unsigned int max_iterations) {
  namespace chr = std::chrono;
  DLOG(INFO) << "Max search samples: " << max_iterations;
  auto start = std::chrono::high_resolution_clock::now();

  StageNode<S, SE, SO, H>::reset_counter();

#ifdef PLAN_DEBUG_INFO
  LOG(INFO) << "starting state: " << current_state.sprintf();
#endif
  LOG_IF(FATAL, current_state.is_terminal()) << "Cannot search from terminal state! Aborting!";
  root_ = std::make_shared<StageNode<S, SE, SO, H>, StageNodeSPtr, std::shared_ptr<S>, const JointAction &,
                           const unsigned int &>(nullptr, current_state.clone(), JointAction(), 0, mcts_parameters_);

  num_iterations = 0;
#ifdef DUMP_Q_VAL
  std::ofstream ofs;
  ofs.open(Q_VAL_DUMPFILE);
  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t");
#endif
  while (
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count()
          < max_search_time_ms && num_iterations < max_iterations) {
    iterate(root_);
    num_iterations += 1;
#ifdef DUMP_Q_VAL
    if(num_iterations % 50 == 0) {
      ofs << num_iterations << "\t";
      for(auto const& pair : root_->get_ego_int_node().get_expected_rewards()) {
        ofs << pair.second.transpose().format(fmt) << "\t";
      }
      ofs << root_->get_best_action()[0] << "\n";
    }
#endif
  }
#ifdef DUMP_Q_VAL
  ofs.close();
#endif
  VLOG(1) << "Search time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << " ms" << ", Samples: " << this->numIterations();
}

template<class S, class SE, class SO, class H>
void Mcts<S, SE, SO, H>::iterate(const StageNodeSPtr &root_node) {

  StageNodeSPtr node = root_node;
  StageNodeSPtr node_p;

  // --------------Select & Expand  -----------------
  // We descend the tree for all joint actions already available -> last node is the newly expanded one
  while (node->select_or_expand(node));

  // -------------- Heuristic Update ----------------
  // Heuristic until terminal node
  std::vector<SE> calculated_heuristic = heuristic_.get_heuristic_values(node);
  node->update_statistics(calculated_heuristic);

  // --------------- Backpropagation ----------------
  // Backpropagate starting from parent node of newly expanded node
  node_p = node->get_parent().lock();
  while (true) {
    node_p->update_statistics(node);
    if (node_p->is_root()) {
      break;
    } else {
      node = node->get_parent().lock();
      node_p = node_p->get_parent().lock();

    }
  }

#ifdef PLAN_DEBUG_INFO
  sprintf(root_node);
#endif
}

template<class S, class SE, class SO, class H>
std::string Mcts<S, SE, SO, H>::sprintf(const StageNodeSPtr &root_node) const {
  std::stringstream ss;
  return ss.str();
}

template<class S, class SE, class SO, class H>
int Mcts<S, SE, SO, H>::numIterations() {
  return this->num_iterations;
}

template<class S, class SE, class SO, class H>
std::string Mcts<S, SE, SO, H>::nodeInfo() {
  return sprintf(root_);
}

template<class S, class SE, class SO, class H>
JointAction Mcts<S, SE, SO, H>::returnBestAction() {
  return root_->get_best_action();
}

template<class S, class SE, class SO, class H>
void Mcts<S, SE, SO, H>::printTreeToDotFile(std::string filename) {
  root_->printTree(filename, 100);
}
template<class S, class SE, class SO, class H>
const std::shared_ptr<StageNode<S, SE, SO, H>> &Mcts<S, SE, SO, H>::get_root() const {
  return root_;
}

} // namespace mcts


#endif