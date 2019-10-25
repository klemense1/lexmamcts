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

namespace mcts {

class CrossingTest {
 public:
    CrossingTest() {
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
        //automata[0].emplace_back("F goal_reached", -500.f, RewardPriority::GOAL, 500.f);
        // Do not collide with others (Safety)
        automata[0].emplace_back("G !collision", -100.f, RewardPriority::SAFETY);
        // Copy rules to other agents
        for (size_t i = 1; i < automata.size(); ++i) {
            automata[i] = Automata::value_type(automata[0]);
        }

        // Rules only for ego
        // Arrive before others (Guarantee)
        // Currently not possible because ego can't drive faster than others
        //automata.emplace_back("!other_goal_reached U ego_goal_reached", -1000.f, RewardPriority::GOAL);
        for(size_t i = 0; i < automata.size(); i++) {
            LOG(INFO) << "Rules for agent " << i << ":";
            for(auto const& rule : automata[i]) {
                LOG(INFO) << rule;
            }
        }
        state = std::make_shared<CrossingState>(automata, label_evaluators);
        rewards = std::vector<Reward>(1, Reward::Zero());
        jt = JointAction(2, (int) Actions::FORWARD);
    }
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
    Automata automata;
    std::vector<Reward> rewards;
    JointAction jt;
    std::vector<std::size_t> pos_history;
    Mcts<CrossingState, UctStatistic, UctStatistic, RandomHeuristic> mcts;
    std::shared_ptr<CrossingState> state;
};

class CrossingStateEpisodeRunner : public CrossingTest {
 public:
    CrossingStateEpisodeRunner(const unsigned int max_steps, Viewer *viewer) :
        MAX_STEPS(max_steps),
        current_step_(0),
        viewer_(viewer) {};

    void step() {
        if (state->is_terminal()) {
            return;
        }
        std::vector<Reward> rewards(2);

        JointAction jointaction(state->get_agent_idx().size());
        Mcts<CrossingState, UctStatistic, UctStatistic, RandomHeuristic> mcts;
        mcts.search(*state, 5000, 1000);
        jointaction = mcts.returnBestAction();
        std::cout << "Step " << current_step_ << ", Action = " << jointaction << ", " << state->sprintf()
                  << std::endl;
        state = state->execute(jointaction, rewards);

        const bool collision = state->is_terminal() && !state->ego_goal_reached();
        const bool goal_reached = state->ego_goal_reached();
        current_step_ += 1;
        const bool max_steps = current_step_ > MAX_STEPS;

        if (viewer_) {
            state->draw(viewer_);
        }
    }

 private:
    Viewer *viewer_;
    const unsigned int MAX_STEPS;
    unsigned int current_step_;
};

} // namespace mcts

#endif // MCTS_EPISODE_RUNNER_H_