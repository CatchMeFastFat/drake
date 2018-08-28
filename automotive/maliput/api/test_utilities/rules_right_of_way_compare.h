#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {
namespace test {


<<<<<<< HEAD
/// Predicate-formatter which tests equality of RightOfWayRule::ZoneType.
=======
/// Predicate-formatter which tests equality of RightOfWayRule::Type.
>>>>>>> intial
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
<<<<<<< HEAD
                                   rules::RightOfWayRule::ZoneType a,
                                   rules::RightOfWayRule::ZoneType b);


/// Predicate-formatter which tests equality of RightOfWayRule::State::Type.
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   rules::RightOfWayRule::State::Type a,
                                   rules::RightOfWayRule::State::Type b);


/// Predicate-formatter which tests equality of RightOfWayRule::State.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RightOfWayRule::State& a,
                                   const rules::RightOfWayRule::State& b);
=======
                                   rules::RightOfWayRule::Type a,
                                   rules::RightOfWayRule::Type b);
>>>>>>> intial


/// Predicate-formatter which tests equality of RightOfWayRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b);


<<<<<<< HEAD
/// Predicate-formatter which tests equality of RightOfWayStateProvider::Result.
::testing::AssertionResult IsEqual(
     const char* a_expression,
     const char* b_expression,
     const rules::RightOfWayStateProvider::Result& a,
     const rules::RightOfWayStateProvider::Result& b);


=======
>>>>>>> intial
}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
