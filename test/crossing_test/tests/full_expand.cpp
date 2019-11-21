//
// Created by Luis Gressenbuch on 05.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include <vector>
#include <deque>
#include <iostream>
#include <thread>
#include <future>

#include "test/crossing_test/tests/crossing_test_env.h"
#include "test/crossing_test/tests/util.h"

using namespace std;

typedef deque<JointAction> ActionList;
typedef vector<Reward> JointReward;
typedef shared_ptr<CrossingState> CrossingStateSPtr;

ActionList possible_actions(ActionIdx n, size_t k) {
  ActionList l;
  JointAction jt(k, 0);
  size_t i;
  ActionIdx val = 0;
  do {
    l.emplace_back(jt);
    for (i = 0; i < k; ++i) {
      val = (jt.at(i) + 1);
      jt[i] = val % n;
      if (jt[i] != 0) {
        break;
      }
    }
  } while (!(i >= (k - 1) && val >= n));
  return l;
}

struct QValPair {
  QValPair(size_t num_agents, size_t reward_vec_size) : action(num_agents, 0), qval(num_agents, Reward::Zero(reward_vec_size)) {};
  JointAction action;
  JointReward qval;
  shared_ptr<QValPair> child;
};

bool cmp_lt(JointReward const &a, JointReward const &b) {
  for (size_t i = 0; i < a.size(); ++i) {
    if (!std::lexicographical_compare(a.at(i).begin(), a.at(i).end(), b.at(i).begin(), b.at(i).end())) {
      return false;
    }
  }
  return true;
}

unsigned long long node_count = 0;

bool dfs(CrossingStateSPtr root, ActionList const &actions, shared_ptr<QValPair> qval) {
  ++node_count;
  if (root->is_terminal()) {
    qval->qval = root->get_final_reward();
    qval->child = shared_ptr<QValPair>();
    return false;
  }
  JointReward step_reward(root->get_agent_idx().size(), Reward::Zero(qval->qval.size()));
  JointReward total_reward(root->get_agent_idx().size(), Reward::Zero(qval->qval.size()));
  JointAction best_action;

  qval->qval = JointReward(root->get_agent_idx().size(), Reward::Constant(qval->qval.size(), -std::numeric_limits<Reward::Scalar>::max()));
  for (auto const &act : actions) {
    CrossingStateSPtr next_state = root->execute(act, step_reward);
    shared_ptr<QValPair> candidate = make_shared<QValPair>(root->get_agent_idx().size(), qval->qval.size());
    dfs(next_state, actions, candidate);
    total_reward = candidate->qval + step_reward;
    if (cmp_lt(qval->qval, total_reward)) {
      qval->qval = total_reward;
      qval->action = act;
      qval->child = candidate;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = 1;
  FLAGS_logtostderr = 1;
  size_t const num_agents = 2;
  CrossingTestEnv<> test_env;
  int const max_depth = test_env.state->get_parameters().terminal_depth_;
  shared_ptr<QValPair> root = make_shared<QValPair>(num_agents, test_env.mcts_parameters_.REWARD_VEC_SIZE);
  ActionList actions = possible_actions(static_cast<ActionIdx>(Actions::NUM), num_agents);
  double total_nodes = (pow(actions.size(), max_depth + 1) - 1)/ static_cast<double>(actions.size() - 1);
  LOG(WARNING) << "# Actions: " << actions.size();
  LOG(WARNING) << "Total nodes to expand: " << total_nodes;
  auto start = chrono::high_resolution_clock::now();
  auto fut = async(dfs, test_env.state, actions, root);
  while (fut.wait_for(chrono::seconds(10)) != future_status::ready) {
    unsigned long long n = node_count;
    double elapsed = chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - start).count();
    double speed = static_cast<double>(n) / elapsed;
    LOG(WARNING) << "Nodes expanded: " << n << " [ " << 100.0 * static_cast<double>(n) / total_nodes
      << "% ]" << ", " << speed << " nodes/s, ETA: " << (total_nodes - n) / speed / 60.0 << " min";
  }
  LOG(WARNING) << "Total nodes expanded: " << node_count;
  LOG(WARNING) << "Optimal Reward: " << root->qval;
  LOG(WARNING) << "Actions:";
  while (root->child) {
    LOG(WARNING) << root->action;
    root = root->child;
  }
  return 0;
}