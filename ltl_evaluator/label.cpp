//
// Created by Luis Gressenbuch on 09.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "label.h"
ltl::Label::Label(const std::string& label_str, int agent_id)
    : label_str_(label_str), agent_id_(agent_id), is_agent_specific_(true) {}
ltl::Label::Label(const std::string& label_str)
    : label_str_(label_str), agent_id_(-1), is_agent_specific_(false) {}
const std::string& ltl::Label::get_label_str() const { return label_str_; }
int ltl::Label::get_agent_id() const { return agent_id_; }
bool ltl::Label::is_agent_specific() const { return is_agent_specific_; }
bool ltl::Label::operator==(const ltl::Label& rhs) const {
  return label_str_ == rhs.label_str_ && agent_id_ == rhs.agent_id_ &&
         is_agent_specific_ == rhs.is_agent_specific_;
}
bool ltl::Label::operator!=(const ltl::Label& rhs) const {
  return !(rhs == *this);
}
ltl::Label::Label() {}
std::ostream& ltl::operator<<(std::ostream& os, const ltl::Label& label) {
  os << "label_str_: " << label.label_str_ << " agent_id_: " << label.agent_id_
     << " is_agent_specific_: " << label.is_agent_specific_;
  return os;
}
