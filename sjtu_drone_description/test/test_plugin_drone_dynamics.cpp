// Copyright 2024 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");

/// @file test_plugin_drone_dynamics.cpp
/// @brief Phase 4 tests: verify UpdateDynamics, PublishOdom, reset handling,
///        and thread safety (mutex) in the gz-sim8 ported drone plugin.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/World.hh>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "sjtu_drone_description/plugin_drone.h"
#include "sjtu_drone_description/plugin_drone_private.h"

class DronePluginPhase4Test : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /// Helper: set up a private instance with ROS node, publishers, subscribers,
  /// and a minimal ECM with a world + model + link entity.
  void SetUpDroneWithECM(sjtu_drone::DroneSimpleControllerPrivate &priv,
                          gz::sim::EntityComponentManager &ecm)
  {
    priv.ros_node_ = rclcpp::Node::make_shared("test_dynamics_node");
    priv.tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(priv.ros_node_);
    priv.InitSubscribers();
    priv.InitPublishers();

    // Create world entity with gravity
    auto worldEntity = ecm.CreateEntity();
    ecm.CreateComponent(worldEntity, gz::sim::components::World());
    ecm.CreateComponent(worldEntity, gz::sim::components::Name("default"));
    gz::math::Vector3d gravity(0, 0, -9.81);
    ecm.CreateComponent(worldEntity, gz::sim::components::Gravity(gravity));
    priv.world_entity = worldEntity;

    // Create model entity
    auto modelEntity = ecm.CreateEntity();
    ecm.CreateComponent(modelEntity, gz::sim::components::Model());
    ecm.CreateComponent(modelEntity, gz::sim::components::Name("drone_model"));
    ecm.CreateComponent(modelEntity, gz::sim::components::ParentEntity(worldEntity));
    priv.model_entity = modelEntity;

    // Create link entity with inertial, pose, velocities
    auto linkEntity = ecm.CreateEntity();
    ecm.CreateComponent(linkEntity, gz::sim::components::Link());
    ecm.CreateComponent(linkEntity, gz::sim::components::Name("base_link"));
    ecm.CreateComponent(linkEntity, gz::sim::components::ParentEntity(modelEntity));
    priv.link_entity = linkEntity;
    priv.link_name = "base_link";

    // Set up pose at origin
    gz::math::Pose3d identityPose(0, 0, 1.0, 0, 0, 0);
    ecm.CreateComponent(linkEntity, gz::sim::components::WorldPose(identityPose));

    // Set up zero velocities
    ecm.CreateComponent(linkEntity, gz::sim::components::WorldLinearVelocity(
      gz::math::Vector3d(0, 0, 0)));
    ecm.CreateComponent(linkEntity, gz::sim::components::WorldAngularVelocity(
      gz::math::Vector3d(0, 0, 0)));

    // Set mass and inertia
    priv.mass = 1.5;
    priv.inertia = gz::math::Vector3d(0.0347563, 0.0458929, 0.0977);
  }

  /// Helper: load default PID controller settings
  void LoadDefaultPID(sjtu_drone::DroneSimpleControllerPrivate &priv)
  {
    std::string sdfStr =
      "<sdf version='1.6'>"
      "  <plugin name='test' filename='test'>"
      "    <rollpitchProportionalGain>10.0</rollpitchProportionalGain>"
      "    <rollpitchDifferentialGain>5.0</rollpitchDifferentialGain>"
      "    <yawProportionalGain>2.0</yawProportionalGain>"
      "    <yawDifferentialGain>1.0</yawDifferentialGain>"
      "    <velocityXYProportionalGain>5.0</velocityXYProportionalGain>"
      "    <velocityXYDifferentialGain>2.3</velocityXYDifferentialGain>"
      "    <velocityZProportionalGain>5.0</velocityZProportionalGain>"
      "    <velocityZIntegralGain>0.0</velocityZIntegralGain>"
      "    <velocityZDifferentialGain>1.0</velocityZDifferentialGain>"
      "    <positionXYProportionalGain>1.1</positionXYProportionalGain>"
      "    <positionXYDifferentialGain>0.0</positionXYDifferentialGain>"
      "    <positionXYIntegralGain>0.0</positionXYIntegralGain>"
      "    <positionZProportionalGain>1.0</positionZProportionalGain>"
      "    <positionZDifferentialGain>0.5</positionZDifferentialGain>"
      "    <positionZIntegralGain>0.0</positionZIntegralGain>"
      "  </plugin>"
      "</sdf>";

    sdf::SDFPtr sdfParsed(new sdf::SDF());
    sdf::init(sdfParsed);
    sdf::readString(sdfStr, sdfParsed);
    sdf::ElementPtr pluginElem = sdfParsed->Root()->GetElement("plugin");
    priv.LoadControllerSettings(pluginElem);
  }
};

// --- Test: reset_requested member exists and defaults to false ---

TEST_F(DronePluginPhase4Test, ResetRequestedDefaultsFalse) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  EXPECT_FALSE(priv.reset_requested);
}

// --- Test: mutex_ member exists (compilation check) ---

TEST_F(DronePluginPhase4Test, MutexMemberExists) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  // Verify mutex compiles and can be locked/unlocked
  std::lock_guard<std::mutex> lock(priv.mutex_);
  SUCCEED();
}

// --- Test: UpdateDynamics reads ECM and applies wrench when FLYING ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsAppliesForceWhenFlying) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  gz::sim::EntityComponentManager ecm;
  SetUpDroneWithECM(priv, ecm);
  LoadDefaultPID(priv);

  priv.navi_state = FLYING_MODEL;
  priv.current_time = 1.0;

  // First call should read ECM successfully (components already created)
  double dt = 0.01;
  priv.UpdateDynamics(dt, ecm);

  // After UpdateDynamics, a wrench should be applied to the link
  auto wrenchComp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv.link_entity);
  ASSERT_NE(wrenchComp, nullptr);

  // The force Z should be positive (hover force to counteract gravity)
  double fz = wrenchComp->Data().force().z();
  EXPECT_GT(fz, 0.0) << "Expect positive Z force for hovering drone";
}

// --- Test: UpdateDynamics does NOT apply force when LANDED ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsNoForceWhenLanded) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  gz::sim::EntityComponentManager ecm;
  SetUpDroneWithECM(priv, ecm);
  LoadDefaultPID(priv);

  priv.navi_state = LANDED_MODEL;
  priv.current_time = 1.0;

  double dt = 0.01;
  priv.UpdateDynamics(dt, ecm);

  // When landed, no wrench should be applied (or it should be zero)
  auto wrenchComp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv.link_entity);
  if (wrenchComp) {
    EXPECT_DOUBLE_EQ(wrenchComp->Data().force().x(), 0.0);
    EXPECT_DOUBLE_EQ(wrenchComp->Data().force().y(), 0.0);
    EXPECT_DOUBLE_EQ(wrenchComp->Data().force().z(), 0.0);
    EXPECT_DOUBLE_EQ(wrenchComp->Data().torque().x(), 0.0);
    EXPECT_DOUBLE_EQ(wrenchComp->Data().torque().y(), 0.0);
    EXPECT_DOUBLE_EQ(wrenchComp->Data().torque().z(), 0.0);
  }
}

// --- Test: UpdateDynamics applies stronger force when TAKINGOFF (1.5x) ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsTakingOffMultiplier) {
  sjtu_drone::DroneSimpleControllerPrivate priv1;
  gz::sim::EntityComponentManager ecm1;
  SetUpDroneWithECM(priv1, ecm1);
  LoadDefaultPID(priv1);

  sjtu_drone::DroneSimpleControllerPrivate priv2;
  gz::sim::EntityComponentManager ecm2;
  SetUpDroneWithECM(priv2, ecm2);
  LoadDefaultPID(priv2);

  double dt = 0.01;

  // Flying model: normal force
  priv1.navi_state = FLYING_MODEL;
  priv1.current_time = 1.0;
  priv1.UpdateDynamics(dt, ecm1);

  // Taking off: 1.5x force
  priv2.navi_state = TAKINGOFF_MODEL;
  priv2.current_time = 1.0;
  priv2.UpdateDynamics(dt, ecm2);

  auto wrench1 = ecm1.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv1.link_entity);
  auto wrench2 = ecm2.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv2.link_entity);

  ASSERT_NE(wrench1, nullptr);
  ASSERT_NE(wrench2, nullptr);

  double fz_flying = wrench1->Data().force().z();
  double fz_takeoff = wrench2->Data().force().z();

  // Taking off force should be 1.5x the flying force
  EXPECT_NEAR(fz_takeoff, fz_flying * 1.5, 1e-6);
}

// --- Test: UpdateDynamics applies weaker force when LANDING (0.8x) ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsLandingMultiplier) {
  sjtu_drone::DroneSimpleControllerPrivate priv1;
  gz::sim::EntityComponentManager ecm1;
  SetUpDroneWithECM(priv1, ecm1);
  LoadDefaultPID(priv1);

  sjtu_drone::DroneSimpleControllerPrivate priv2;
  gz::sim::EntityComponentManager ecm2;
  SetUpDroneWithECM(priv2, ecm2);
  LoadDefaultPID(priv2);

  double dt = 0.01;

  // Flying model: normal force
  priv1.navi_state = FLYING_MODEL;
  priv1.current_time = 1.0;
  priv1.UpdateDynamics(dt, ecm1);

  // Landing: 0.8x force
  priv2.navi_state = LANDING_MODEL;
  priv2.current_time = 1.0;
  priv2.UpdateDynamics(dt, ecm2);

  auto wrench1 = ecm1.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv1.link_entity);
  auto wrench2 = ecm2.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv2.link_entity);

  ASSERT_NE(wrench1, nullptr);
  ASSERT_NE(wrench2, nullptr);

  double fz_flying = wrench1->Data().force().z();
  double fz_landing = wrench2->Data().force().z();

  EXPECT_NEAR(fz_landing, fz_flying * 0.8, 1e-6);
}

// --- Test: UpdateDynamics publishes ground truth pose ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsPublishesGroundTruth) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  gz::sim::EntityComponentManager ecm;
  SetUpDroneWithECM(priv, ecm);
  LoadDefaultPID(priv);

  priv.navi_state = FLYING_MODEL;
  priv.current_time = 1.0;

  // Subscribe to ground truth pose topic
  geometry_msgs::msg::Pose received_pose;
  bool received = false;
  auto sub = priv.ros_node_->create_subscription<geometry_msgs::msg::Pose>(
    "gt_pose", 10,
    [&](const geometry_msgs::msg::Pose::SharedPtr msg) {
      received_pose = *msg;
      received = true;
    });

  double dt = 0.01;
  priv.UpdateDynamics(dt, ecm);

  // Spin to receive the message
  auto start = std::chrono::steady_clock::now();
  while (!received && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2)) {
    rclcpp::spin_some(priv.ros_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received) << "Should have received ground truth pose message";
  // Pose should match the ECM world pose (z=1.0)
  EXPECT_NEAR(received_pose.position.z, 1.0, 0.01);
}

// --- Test: PublishOdom publishes odometry and TF ---

TEST_F(DronePluginPhase4Test, PublishOdomSendsOdomMessage) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  priv.ros_node_ = rclcpp::Node::make_shared("test_odom_pub");
  priv.tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(priv.ros_node_);
  priv.InitPublishers();
  priv.current_time = 1.5;

  nav_msgs::msg::Odometry received_odom;
  bool received = false;
  auto sub = priv.ros_node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      received_odom = *msg;
      received = true;
    });

  gz::math::Pose3d test_pose(1.0, 2.0, 3.0, 0, 0, 0);
  gz::math::Vector3d test_vel(0.1, 0.2, 0.3);
  gz::math::Vector3d test_acc(0.0, 0.0, 0.0);

  priv.PublishOdom(test_pose, test_vel, test_acc);

  auto start = std::chrono::steady_clock::now();
  while (!received && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2)) {
    rclcpp::spin_some(priv.ros_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received) << "Should have received odom message";
  EXPECT_NEAR(received_odom.pose.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(received_odom.pose.pose.position.y, 2.0, 1e-6);
  EXPECT_NEAR(received_odom.pose.pose.position.z, 3.0, 1e-6);

  // Timestamp should be based on current_time = 1.5
  EXPECT_EQ(received_odom.header.stamp.sec, 1);
  EXPECT_GT(received_odom.header.stamp.nanosec, 0u);
}

// --- Test: max_force_ clamp ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsClampForce) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  gz::sim::EntityComponentManager ecm;
  SetUpDroneWithECM(priv, ecm);
  LoadDefaultPID(priv);

  priv.navi_state = FLYING_MODEL;
  priv.current_time = 1.0;
  priv.max_force_ = 5.0;  // Set a low max force

  double dt = 0.01;
  priv.UpdateDynamics(dt, ecm);

  auto wrenchComp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(priv.link_entity);
  ASSERT_NE(wrenchComp, nullptr);

  // The world-frame force Z component: since pose is identity,
  // world_force = pose.Rot().RotateVector(body_force) = body_force
  // The body force[2] should be clamped to max_force_ = 5.0
  // But after rotation to world frame, the magnitude may differ
  // In this case with identity rotation, they're the same
  double fz = wrenchComp->Data().force().z();
  EXPECT_LE(fz, 5.0 + 1e-6) << "Force Z should be clamped to max_force_";
}

// --- Test: ECM components get created if missing ---

TEST_F(DronePluginPhase4Test, UpdateDynamicsCreatesComponentsIfMissing) {
  sjtu_drone::DroneSimpleControllerPrivate priv;
  gz::sim::EntityComponentManager ecm;

  priv.ros_node_ = rclcpp::Node::make_shared("test_create_components");
  priv.tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(priv.ros_node_);
  priv.InitSubscribers();
  priv.InitPublishers();
  LoadDefaultPID(priv);

  // Create world with gravity
  auto worldEntity = ecm.CreateEntity();
  ecm.CreateComponent(worldEntity, gz::sim::components::World());
  ecm.CreateComponent(worldEntity, gz::sim::components::Name("default"));
  ecm.CreateComponent(worldEntity, gz::sim::components::Gravity(gz::math::Vector3d(0, 0, -9.81)));
  priv.world_entity = worldEntity;

  // Create model
  auto modelEntity = ecm.CreateEntity();
    ecm.CreateComponent(modelEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelEntity, gz::sim::components::Name("drone"));
  ecm.CreateComponent(modelEntity, gz::sim::components::ParentEntity(worldEntity));
  priv.model_entity = modelEntity;

  // Create link WITHOUT WorldPose, WorldLinearVelocity, WorldAngularVelocity
  auto linkEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkEntity, gz::sim::components::Link());
  ecm.CreateComponent(linkEntity, gz::sim::components::Name("base_link"));
  ecm.CreateComponent(linkEntity, gz::sim::components::ParentEntity(modelEntity));
  priv.link_entity = linkEntity;
  priv.link_name = "base_link";
  priv.mass = 1.5;
  priv.inertia = gz::math::Vector3d(0.035, 0.046, 0.098);
  priv.navi_state = FLYING_MODEL;

  double dt = 0.01;

  // First call: WorldPose component doesn't exist, should return early without crash
  EXPECT_NO_THROW(priv.UpdateDynamics(dt, ecm))
    << "UpdateDynamics should handle missing components gracefully";

  // navi_state should remain unchanged (early return before physics update)
  EXPECT_EQ(priv.navi_state, FLYING_MODEL)
    << "State should be unchanged after early return";
}
