// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================


#ifndef MCTS_EPISODE_RUNNER_H_
#define MCTS_EPISODE_RUNNER_H_

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

namespace mcts {

class CrossingStateEpisodeRunner {
 public:
  CrossingStateEpisodeRunner(const unsigned int max_steps, Viewer *viewer) :
      current_state_(),
      last_state_(),
      MAX_STEPS(max_steps),
      current_step_(0),
      viewer_(viewer) {
    //RandomGenerator::random_generator_ = std::mt19937(1000);
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
    Automata automata;
    // SETUP LABEL EVALUATORS
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision", XING_POINT));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelGoalReached>("goal_reached", EGO_GOAL_POS));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelHoldAtXing>("at_hp_xing", XING_POINT));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
    // SETUP RULES
    automata.resize(NUM_OTHER_AGENTS + 1);
    // Do not collide with others (Safety)
    automata[0].emplace_back("G !collision", -1000.f, RewardPriority::SAFETY);
    // Finally arrive at goal (Liveness)
    automata[0].emplace_back("F goal_reached", -1000.f, RewardPriority::GOAL);
    // Copy rules to other agents
    for (size_t i = 1; i < automata.size(); ++i) {
      automata[i] = Automata::value_type(automata[0]);
    }
    // Rules only for ego
    // Arrive before others (Guarantee)
    // Currently not possible because ego can't drive faster than others
    // TODO: Add more actions for ego
    //automata.emplace_back("!other_goal_reached U ego_goal_reached", -1000.f, RewardPriority::GOAL);
    automata[0].emplace_back("G((at_hp_xing & other_near) -> (X at_hp_xing))", -500.0f, RewardPriority::SAFETY);

    current_state_ = std::make_shared<CrossingState>(automata, label_evaluators);
    last_state_ = current_state_;
  }

  void step() {
    if (current_state_->is_terminal()) {
      return;
    }
    std::vector<Reward> rewards(2);

    JointAction jointaction(current_state_->get_agent_idx().size());
    Mcts<CrossingState, UctStatistic, UctStatistic, RandomHeuristic> mcts;
    for (auto agent_idx : current_state_->get_agent_idx()) {
      if (agent_idx == CrossingState::ego_agent_idx) {
        // Plan for ego agent with hypothesis-based search
        //
        mcts.search(*current_state_, 5000, 1000);
        jointaction[agent_idx] = mcts.returnBestAction();
      } else {
        jointaction[agent_idx] = aconv(Actions::FORWARD);
      }
    }
    std::cout << "Step " << current_step_ << ", Action = " << jointaction << ", " << current_state_->sprintf()
              << std::endl;
    last_state_ = current_state_;
    current_state_ = current_state_->execute(jointaction, rewards);

    const bool collision = current_state_->is_terminal() && !current_state_->ego_goal_reached();
    const bool goal_reached = current_state_->ego_goal_reached();
    current_step_ += 1;
    const bool max_steps = current_step_ > MAX_STEPS;

    if (viewer_) {
      current_state_->draw(viewer_);
    }
  }

 private:
  Viewer *viewer_;
  std::shared_ptr<CrossingState> current_state_;
  std::shared_ptr<CrossingState> last_state_;
  const unsigned int MAX_STEPS;
  unsigned int current_step_;
};

} // namespace mcts

#endif // MCTS_EPISODE_RUNNER_H_