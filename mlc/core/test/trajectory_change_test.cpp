#include "core/lane_change_trajectory_planner.h"
#include <gmock/gmock.h>

using namespace ::testing;

TEST(TestTrajectoryChange, ComputeLaneChangeTrajectoryTest)
{
    Parameters parameters{};

    VehicleState ego_vehicle_state{};

    const auto result{ComputeLaneChangeTrajectory(ego_vehicle_state, parameters)};

    std::vector<double> trajectory_x;
    for (auto ea:result)
        trajectory_x.push_back(ea.x);

    std::vector<double> expect{};

    for(int i = 0; i < 4;i++)
    {
        expect.push_back(2.5*i);
    }
    EXPECT_THAT(trajectory_x, ElementsAreArray(expect));
}
