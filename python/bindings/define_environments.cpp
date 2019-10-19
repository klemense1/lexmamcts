// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "python/bindings/define_environments.hpp"
#include "mcts/mcts.h"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/crossing_state_episode_runner.h"

namespace py = pybind11;
using namespace mcts;

void define_environments(py::module m) {
  py::class_<Viewer,
             PyViewer,
             std::shared_ptr<Viewer>>(m, "Viewer")
      .def(py::init<>())
      .def("drawPoint", &Viewer::drawPoint)
      .def("drawLine", &Viewer::drawLine);

  py::class_<AgentState,
             std::shared_ptr<AgentState>>(m, "AgentCrossingState")
      .def("__repr__", [](const AgentState &m) {
        return "mamcts.AgentCrossingState";
      })
      .def_readonly("position", &AgentState::x_pos)
      .def_readonly("last_action", &AgentState::last_action);

  py::class_<CrossingState,
             std::shared_ptr<CrossingState>>(m, "CrossingState")
      .def(py::init<Automata &,
                    const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>>>())
      .def("__repr__", [](const CrossingState &m) {
        return "mamcts.CrossingState";
      })
      .def("draw", &CrossingState::draw)
      .def_property_readonly("agents_states", &CrossingState::get_agent_states);

  py::class_<CrossingStateEpisodeRunner,
             std::shared_ptr<CrossingStateEpisodeRunner>>(m, "CrossingStateEpisodeRunner")
      .def(py::init<const unsigned int,
                    mcts::Viewer *>())
      .def("__repr__", [](const CrossingStateEpisodeRunner &m) {
        return "mamcts.CrossingStateEpisodeRunner";
      })
      .def("step", &CrossingStateEpisodeRunner::step);
}