#include "core/multiple_lane_change.h"
#include "core/parameters.h"
#include <gmock/gmock.h>

using namespace ::testing;

TEST(Test, PassingDummyTest)
{
    Parameters parameters{};
    MultipleLaneChange multiple_lane_change{parameters};

    multiple_lane_change.Step();

    ASSERT_THAT(true, Eq(true));
}
