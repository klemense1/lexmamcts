// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================


#ifndef MCTS_EPISODE_RUNNER_H_
#define MCTS_EPISODE_RUNNER_H_

#include "glog/logging.h"

#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/heuristics/random_heuristic.h"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/viewer.h"
#include "mcts/random_generator.h"
#include "test/crossing_test/evaluator_label_collision.hpp"
#include "test/crossing_test/evaluator_label_goal_reached.hpp"
#include "test/crossing_test/evaluator_label_hold_at_xing.hpp"
#include "test/crossing_test/evaluator_label_other_near.hpp"
#include "test/crossing_test/common.hpp"
#include "mcts/statistics/pareto_uct_statistic.h"
#include "mcts/statistics/slack_uct_statistic.h"

namespace mcts {

class CrossingTest {
 public:
  CrossingTest() :
      mcts_parameters_(make_std_mcts_parameters()),
      mcts(mcts_parameters_) {
    // SETUP LABEL EVALUATORS
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision",
                                                                            CrossingState::crossing_point));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelGoalReached>("goal_reached",
                                                                              CrossingState::ego_goal_reached_position));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelHoldAtXing>("at_hp_xing",
                                                                             CrossingState::crossing_point));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
    // SETUP RULES
    automata.resize(CrossingState::num_other_agents + 1);

    // Finally arrive at goal (Liveness)
    //automata[0].emplace_back("F goal_reached", -500.f, RewardPriority::GOAL, 1.0f);
    // Do not collide with others (Safety)
    automata[0].emplace_back("G !collision", -1000.f, RewardPriority::SAFETY);
    // Copy rules to other agents
    for (size_t i = 1; i < automata.size(); ++i) {
      automata[i] = Automata::value_type(automata[0]);
    }

    // Rules only for ego
    //automata[0].emplace_back("G((at_hp_xing & other_near) -> (X at_hp_xing))", -100.0f, RewardPriority::SAFETY);
    // Arrive before others (Guarantee)
    // Currently not possible because ego can't drive faster than others
    //automata.emplace_back("!other_goal_reached U ego_goal_reached", -1000.f, RewardPriority::GOAL);
    for (size_t i = 0; i < automata.size(); i++) {
      LOG(INFO) << "Rules for agent " << i << ":";
      for (auto const &rule : automata[i]) {
        LOG(INFO) << rule;
      }
    }
    state = std::make_shared<CrossingState>(automata, label_evaluators);
    rewards = std::vector<Reward>(1, Reward::Zero());
    jt = JointAction(2, (int) Actions::FORWARD);
  }
  ~CrossingTest() {
    LOG(INFO) << "Ego positions:" << pos_history;
    LOG(INFO) << "Otr positions:" << pos_history_other;
  }
  MctsParameters const mcts_parameters_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
  Automata automata;
  std::vector<Reward> rewards;
  JointAction jt;
  std::vector<std::size_t> pos_history;
  std::vector<size_t> pos_history_other;
  typedef SlackUCTStatistic Stats;
  //typedef UctStatistic<> Stats;
  Mcts<CrossingState, Stats, Stats, RandomHeuristic> mcts;
  std::shared_ptr<CrossingState> state;
};

class CrossingStateEpisodeRunner : public CrossingTest {
 public:
  CrossingStateEpisodeRunner(const unsigned int max_steps, Viewer *viewer) :
      viewer_(viewer),
      current_step_(0),
      MAX_STEPS(max_steps) {};

  void step() {
    if (state->is_terminal() || current_step_ > MAX_STEPS) {
      return;
    }
    std::vector<Reward> rewards(2);

        JointAction jointaction(state->get_agent_idx().size());
        mcts.search(*state, 5000, 1000);
        jointaction = mcts.returnBestAction();
        std::cout << "Step " << current_step_ << ", Action = " << jointaction << ", " << state->sprintf()
                  << std::endl;
        state = state->execute(jointaction, rewards);

    current_step_ += 1;

    if (viewer_) {
      state->draw(viewer_);
    }
  }

 private:
  Viewer *viewer_;
  unsigned int current_step_;
  const unsigned int MAX_STEPS;
};

}  // namespace mcts

#endif // MCTS_EPISODE_RUNNER_H_