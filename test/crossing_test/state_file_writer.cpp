//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "state_file_writer.h"

void StateFileWriter::write_multi_timestep(
    const std::vector<Eigen::MatrixXi>& states) {
  for (const auto &t : states) {
    ofstream_ << timestamp_++;
    for (const auto &agent_state : t.row(0)) {
      ofstream_ << "\t" << agent_state;
    }
    ofstream_ << "\n";
  }
}
StateFileWriter::StateFileWriter(size_t num_agents, const std::string &filename)
    : timestamp_(0), num_agents_(num_agents), filename_(filename) {
  ofstream_.open(filename);
  ofstream_ << "# Timestep";
  for (size_t i = 0; i < num_agents; ++i) {
    ofstream_ << "\t Agent " << i;
  }
  ofstream_ << "\n";
}
StateFileWriter::~StateFileWriter() { ofstream_.close(); }
