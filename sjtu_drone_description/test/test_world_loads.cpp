// Copyright 2026 SJTU Drone Authors
// Test that the gz-sim world file is valid SDF and has expected configuration.

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include <filesystem>
#include <string>
#include <set>

#ifndef WORLD_DIR
#error "WORLD_DIR must be defined at compile time"
#endif

static const std::string kWorldPath =
    std::string(WORLD_DIR) + "/playground.world";

class WorldFileTest : public ::testing::Test {
 protected:
  sdf::Root root;
  bool parsed{false};

  void SetUp() override {
    ASSERT_TRUE(std::filesystem::exists(kWorldPath))
        << "World file not found: " << kWorldPath;
    auto errors = root.Load(kWorldPath);
    ASSERT_TRUE(errors.empty())
        << "SDF parse errors: " << errors.front().Message();
    parsed = true;
  }
};

TEST_F(WorldFileTest, FileExists) {
  EXPECT_TRUE(std::filesystem::exists(kWorldPath));
}

TEST_F(WorldFileTest, HasOneWorld) {
  ASSERT_TRUE(parsed);
  EXPECT_EQ(root.WorldCount(), 1u);
}

TEST_F(WorldFileTest, PhysicsIsDart) {
  ASSERT_TRUE(parsed);
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  // Check that at least one physics profile uses DART
  ASSERT_GT(world->PhysicsCount(), 0u);
  const auto *physics = world->PhysicsByIndex(0);
  ASSERT_NE(physics, nullptr);
  EXPECT_EQ(physics->EngineType(), "dart");
}

TEST_F(WorldFileTest, GravityIsCorrect) {
  ASSERT_TRUE(parsed);
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  auto gravity = world->Gravity();
  EXPECT_DOUBLE_EQ(gravity.X(), 0.0);
  EXPECT_DOUBLE_EQ(gravity.Y(), 0.0);
  EXPECT_DOUBLE_EQ(gravity.Z(), -9.8);
}

TEST_F(WorldFileTest, RequiredPluginsDeclared) {
  ASSERT_TRUE(parsed);
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);

  // Collect all plugin names from the world element
  std::set<std::string> found_plugins;
  auto elem = world->Element();
  ASSERT_NE(elem, nullptr);

  if (elem->HasElement("plugin")) {
    auto plugin_elem = elem->GetElement("plugin");
    while (plugin_elem) {
      if (plugin_elem->HasAttribute("name")) {
        std::string name;
        plugin_elem->GetAttribute("name")->Get(name);
        found_plugins.insert(name);
      }
      plugin_elem = plugin_elem->GetNextElement("plugin");
    }
  }

  // These system plugins are required for our drone simulation
  std::vector<std::string> required = {
      "gz::sim::systems::Physics",
      "gz::sim::systems::UserCommands",
      "gz::sim::systems::SceneBroadcaster",
      "gz::sim::systems::Contact",
      "gz::sim::systems::Imu",
      "gz::sim::systems::Sensors",
  };

  for (const auto &name : required) {
    EXPECT_TRUE(found_plugins.count(name) > 0)
        << "Missing required plugin: " << name;
  }
}

TEST_F(WorldFileTest, HasGroundPlane) {
  ASSERT_TRUE(parsed);
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  // Look for a model named ground_plane
  bool found = false;
  for (uint64_t i = 0; i < world->ModelCount(); ++i) {
    if (world->ModelByIndex(i)->Name() == "ground_plane") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "ground_plane model not found in world";
}

TEST_F(WorldFileTest, HasSunLight) {
  ASSERT_TRUE(parsed);
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  bool found = false;
  for (uint64_t i = 0; i < world->LightCount(); ++i) {
    if (world->LightByIndex(i)->Name() == "sun") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "sun light not found in world";
}
