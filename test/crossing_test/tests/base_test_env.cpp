//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/tests/base_test_env.h"
#include "test/crossing_test/label_evaluator/evaluator_label_ego_range.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_range.h"

#include <utility>

BaseTestEnv::BaseTestEnv(
    MctsParameters mcts_parameters,
    CrossingStateParameter crossing_state_parameter,
    std::vector<std::map<Rule, RuleMonitorSPtr>> automata,
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators)
    : mcts_parameters_(std::move(mcts_parameters)),
      crossing_state_parameter_(std::move(crossing_state_parameter)),
      label_evaluators_(std::move(label_evaluators)),
      rewards(crossing_state_parameter.num_other_agents + 1,
              Reward::Zero(mcts_parameters.REWARD_VEC_SIZE)),
      automata_(std::move(automata)),
      jt_(2, static_cast<int>(Actions::FORWARD)) {
  create_state();
}
std::vector<std::map<Rule, RuleMonitorSPtr>> BaseTestEnv::make_default_automata(size_t num_agents) {
  std::vector<std::map<Rule, RuleMonitorSPtr>> automata(num_agents);
  automata[0].insert({Rule::NO_COLLISION, RuleMonitor::make_rule("G !collision", -1.0f, 0)});
  // Same default rules for all agents
  for (size_t i = 1; i < automata.size(); ++i) {
    automata[i] = automata[0];
  }
  // Ego should give way
  automata[0].insert({Rule::GIVE_WAY, RuleMonitor::make_rule("G(other_near -> !crossed W other_crossed)", -1.0f, 1)});
  return automata;
}
std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> BaseTestEnv::make_default_labels(
    const CrossingStateParameter &params) {
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> labels;
  labels.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision", params.crossing_point + 1));
  labels.emplace_back(std::make_shared<EvaluatorLabelEgoRange>("crossed", params.crossing_point, 1000));
  labels.emplace_back(std::make_shared<EvaluatorLabelOtherRange>("other_crossed", params.crossing_point + 1, 1000));
  labels.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
  return labels;
}
RuleStateMap BaseTestEnv::get_automata_vec() const {
  RuleStateMap aut_v(automata_.size());
  std::vector<int> agent_ids(automata_.size(), 0);
  std::iota(agent_ids.begin(), agent_ids.end(), 0);
  std::vector<int> others;
  for (size_t i = 0; i < automata_.size(); ++i) {
    LOG(INFO) << "Rules for agent " << i << ":";
    others = agent_ids;
    others.erase(others.begin() + i);
    for (const auto &it : automata_[i]) {
      LOG(INFO) << *(it.second);
      auto rule_states = it.second->make_rule_state(others, {});
      for (const auto &rs : rule_states) {
        aut_v[i].insert({it.first, rs});
      }
    }
  }
  return aut_v;
}
void BaseTestEnv::create_state() {
  RuleStateMap aut_v = get_automata_vec();
  state = std::make_shared<CrossingState>(aut_v, label_evaluators_,
                                          crossing_state_parameter_);
}
void BaseTestEnv::set_jt(const JointAction &jt) {
  jt_ = jt;
  action_history_.emplace_back(jt);
}
const std::deque<JointAction> &BaseTestEnv::get_action_history() const { return action_history_; }
const JointAction &BaseTestEnv::get_jt() const { return jt_; }
BaseTestEnv::~BaseTestEnv() {}
