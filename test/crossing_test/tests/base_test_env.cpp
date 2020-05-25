//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/tests/base_test_env.h"
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
  CreateState();
}
std::vector<std::map<Rule, RuleMonitorSPtr>> BaseTestEnv::MakeDefaultAutomata(size_t num_agents) {
  std::vector<std::map<Rule, RuleMonitorSPtr>> automata(num_agents);
  automata[0].insert({Rule::NO_COLLISION, RuleMonitor::MakeRule("G !collision", -1.0f, 0)});
  // Same default rules for all agents
  for (size_t i = 1; i < automata.size(); ++i) {
    automata[i] = automata[0];
  }
  return automata;
}
std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> BaseTestEnv::MakeDefaultLabels(
    const CrossingStateParameter &params) {
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> labels;
  labels.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision", params.crossing_point));
  return labels;
}
RuleStateMap BaseTestEnv::GetAutomataVec() const {
  RuleStateMap aut_v(automata_.size());
  std::vector<int> agent_ids(automata_.size(), 0);
  std::iota(agent_ids.begin(), agent_ids.end(), 0);
  std::vector<int> others;
  for (size_t i = 0; i < automata_.size(); ++i) {
    VLOG(1) << "Rules for agent " << i << ":";
    others = agent_ids;
    others.erase(others.begin() + i);
    for (const auto &it : automata_[i]) {
      VLOG(1) << *(it.second);
      auto rule_states = it.second->MakeRuleState(others, {});
      for (const auto &rs : rule_states) {
        aut_v[i].insert({it.first, rs});
      }
    }
  }
  return aut_v;
}
void BaseTestEnv::CreateState() {
  RuleStateMap aut_v = GetAutomataVec();
  state = std::make_shared<CrossingState>(aut_v, label_evaluators_,
                                          crossing_state_parameter_);
}
void BaseTestEnv::SetJt(const JointAction &jt) {
  jt_ = jt;
  action_history_.emplace_back(jt);
}
const std::deque<JointAction> &BaseTestEnv::GetActionHistory() const { return action_history_; }
const JointAction &BaseTestEnv::GetJt() const { return jt_; }
BaseTestEnv::~BaseTestEnv() {}
