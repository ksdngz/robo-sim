#pragma once

// model
// https://github.com/google-deepmind/mujoco_menagerie/blob/main/google_robot/robot.xml

// panda arm
// constexpr int DOF = 7;
// const char* model_path = "../models/urdf/panda_arm/robot/panda_arm_mjcf.xml";

// google_robot
constexpr int DOF = 9;
const char* model_path = "../models/urdf/mujoco_menagerie/google_robot/robot.xml";

// color code(rgba)
const float CLR_RED[4] = {1.f, 0.f, 0.f, 1.f};
const float CLR_GREEN[4] = {0.f, 1.f, 0.f, 1.f};
const float CLR_BLUE[4] = {0.f, 0.f, 1.f, 1.f};
const float CLR_PURPLE[4] = {1.f, 0.f, 1.f, 1.f};
const float CLR_YELLOW[4] = {1.f, 1.f, 0.f, 1.f};
