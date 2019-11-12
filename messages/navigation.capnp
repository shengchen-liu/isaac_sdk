#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xf5031e6b6ad6bb47;

using import "geometry.capnp".PlaneProto;
using import "math.capnp".Pose2dProto;
using import "math.capnp".Vector2dProto;
using import "image.capnp".ImageProto;

# An occupancy grid map which provides mean and standard deviation for every grid cell. This
# represents the average number of times a cell was observed as occupied.
struct OccupancyMapProto {
  # The name of the map frame as registered in the PoseTree
  mapFrame @0: Text;
  # The size of a grid cell in meters
  cellSize @1: Float32;
  # Stores the mean for very grid cell as a "1ub" image. A value of 0 means 0.0 while a value of
  # 255 means 1.0.
  meanImage @2: ImageProto;
  # Stores the standard deviation for very grid cell using a similar encoding as `meanImage`.
  standardDeviationImage @3: ImageProto;
}

# A binary occupancy grid map which indicates for every grid cell if the cell is free or occupied.
struct BinaryMapProto {
  # The name of the map frame as registered in the PoseTree
  mapFrame @0: Text;
  # The size of a grid cell in meters
  cellSize @1: Float32;
  # Stores the binary occupancy as 0 (free) or 255 (occupied) for very grid cell.
  image @2: ImageProto;
}

# A distance grid map which provides the distance to the nearest obstacle for every grid cell.
struct DistanceMapProto {
  # The name of the map frame as registered in the PoseTree
  mapFrame @0: Text;
  # The size of a grid cell in meters
  cellSize @1: Float32;
  # Stores the distance to the nearest obstacle for every grid cell as a single-prescision float.
  image @2: ImageProto;
}

# Describes how an entity is moving through space in a local diverging coordinate frame.
struct Odometry2Proto {
  # The pose of the "robot" relative to the reference odometric frame
  odomTRobot @0: Pose2dProto;
  speed @1: Vector2dProto;
  angularSpeed @2: Float64;
  acceleration @3: Vector2dProto;
  # Contains the name of the odometry frame and the robot frame.
  odometryFrame @4: Text;
  robotFrame @5: Text;
}

# Describes a plan in a given referential frame.
struct Plan2Proto {
  # List of poses the robot need to go through
  poses @0: List(Pose2dProto);
  # name of the frame the plan is.
  planFrame @1: Text;
}

# Describe a goal in a given referential frame
struct Goal2Proto {
  # The goal expressed in Pose2
  goal @0: Pose2dProto;
  # the tolerance radius of the goal.
  tolerance @1: Float32;
  # name of the frame the goal is.
  goalFrame @2: Text;
  # Whether or not we should stop the robot. If set to true all the other parameters will be ingored
  stopRobot @3: Bool;
}

# A status update which is sent continously as a reply to receiving a goal message. This can be
# used to keep track if the robot has arrived at the destination.
struct Goal2FeedbackProto {
  # Remaining relative pose to the goal, or identity in case hasGoal is false.
  robotTGoal @0: Pose2dProto;
  # Whether the robot currently has a goal
  hasGoal @1: Bool;
  # Whether the robot considers himself to be arrived at the target
  hasArrived @2: Bool;
  # Whether the robot considers himself to not move anymore
  isStationary @3: Bool;
}

# A desired target waypoint
struct GoalWaypointProto {
  # the name of the waypoint
  waypoint @0: Text;
}

# Describes how an entity is moving through space in a local diverging coordinate frame.
struct DifferentialState {
  positionX @0: Float64;
  positionY @1: Float64;
  heading @2: Float64;
  speedX @3: Float64;
  speedY @4: Float64;
  angularSpeed @5: Float64;
  timestamp @6: Float64;
}

# A trajectory plan for a differential base consisting of a sequence of states
struct DifferentialTrajectoryPlanProto {
  # List of states the robot will need to be.
  states @0: List(DifferentialState);
  # name of the frame the plan is
  planFrame @1: Text;
}

# A message describing the location of the ground.
struct GroundPlaneProto {
  # The ground plane.
   plane @0: PlaneProto;
}
