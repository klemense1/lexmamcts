// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_COMMON_H_
#define MVMCTS_COMMON_H_

namespace mvmcts {

class MctsTest;

#ifdef UNIT_TESTING
#define MVMCTS_TEST friend class MctsTest;
#else
#define MVMCTS_TEST
#endif

#ifdef CRTP_DYNAMIC_INTERFACE
#define CRTP_INTERFACE(type)                     \
  inline Implementation& impl() {                \
    auto derivedptr = dynamic_cast<type*>(this); \
    MVMCTS_EXPECT_TRUE(derivedptr != nullptr);     \
    return *derivedptr;                          \
  }
#else
#define CRTP_INTERFACE(type) \
  inline Implementation& impl() { return *static_cast<type*>(this); }
#endif

#ifdef CRTP_DYNAMIC_INTERFACE
#define CRTP_CONST_INTERFACE(type)                           \
  inline const Implementation& impl() const {                \
    auto const derivedptr = dynamic_cast<const type*>(this); \
    MVMCTS_EXPECT_TRUE(derivedptr != nullptr);                 \
    return *derivedptr;                                      \
  }
#else
#define CRTP_CONST_INTERFACE(type)            \
  inline const Implementation& impl() const { \
    return *static_cast<const type*>(this);   \
  }
#endif

#define MVMCTS_EXPECT_TRUE(cond)

}  // namespace mvmcts
#endif  // MVMCTS_COMMON_H_
