// Copyright 2024 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.en.html

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <tinyxml2.h>

// ---- Helpers ----
static bool file_exists(const std::string &path)
{
  std::ifstream f(path);
  return f.good();
}

static std::string read_file(const std::string &path)
{
  std::ifstream f(path);
  std::stringstream buf;
  buf << f.rdbuf();
  return buf.str();
}

// ---------------------------------------------------------------------------
// 1. Source file inventory: correct files exist, obsolete ones do not
// ---------------------------------------------------------------------------
TEST(IntegrationMigration, GzPluginSourceExists)
{
  EXPECT_TRUE(file_exists(SRC_DIR "/plugin_drone_gz.cpp"))
    << "plugin_drone_gz.cpp must exist";
}

TEST(IntegrationMigration, PidControllerSourceExists)
{
  EXPECT_TRUE(file_exists(SRC_DIR "/pid_controller.cpp"));
}

TEST(IntegrationMigration, PrivateImplSourceExists)
{
  EXPECT_TRUE(file_exists(SRC_DIR "/plugin_drone_private.cpp"));
}

TEST(IntegrationMigration, ClassicPluginRemoved)
{
  EXPECT_FALSE(file_exists(SRC_DIR "/plugin_drone.cpp"))
    << "Obsolete Gazebo Classic plugin_drone.cpp should be removed";
}

// ---------------------------------------------------------------------------
// 2. World file is SDF 1.9
// ---------------------------------------------------------------------------
TEST(IntegrationMigration, WorldFileIsSdf19)
{
  const std::string world_path = std::string(WORLD_DIR) + "/playground.world";
  ASSERT_TRUE(file_exists(world_path));

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(world_path.c_str()), tinyxml2::XML_SUCCESS);

  auto *sdf = doc.FirstChildElement("sdf");
  ASSERT_NE(sdf, nullptr);
  const char *ver = sdf->Attribute("version");
  ASSERT_NE(ver, nullptr);
  EXPECT_EQ(std::string(ver), "1.9");
}

// ---------------------------------------------------------------------------
// 3. SDF model file is SDF 1.9
// ---------------------------------------------------------------------------
TEST(IntegrationMigration, ModelSdfIsSdf19)
{
  const std::string sdf_path = std::string(MODEL_DIR) + "/sjtu_drone.sdf";
  ASSERT_TRUE(file_exists(sdf_path));

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(sdf_path.c_str()), tinyxml2::XML_SUCCESS);

  auto *sdf = doc.FirstChildElement("sdf");
  ASSERT_NE(sdf, nullptr);
  const char *ver = sdf->Attribute("version");
  ASSERT_NE(ver, nullptr);
  EXPECT_EQ(std::string(ver), "1.9");
}

// ---------------------------------------------------------------------------
// 4. SDF model sensor names match bridge expectations
// ---------------------------------------------------------------------------
TEST(IntegrationMigration, SdfSensorNamesMatchBridge)
{
  const std::string sdf_path = std::string(MODEL_DIR) + "/sjtu_drone.sdf";
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.LoadFile(sdf_path.c_str()), tinyxml2::XML_SUCCESS);

  // Expected sensor names that the bridge references
  std::vector<std::string> expected = {
    "sensor_imu", "front_camera", "down_camera", "navsat_sensor", "sonar"
  };

  // Collect all <sensor name="..."> from the SDF
  std::vector<std::string> found;
  auto *sdf = doc.FirstChildElement("sdf");
  ASSERT_NE(sdf, nullptr);
  auto *model = sdf->FirstChildElement("model");
  ASSERT_NE(model, nullptr);
  auto *link = model->FirstChildElement("link");
  ASSERT_NE(link, nullptr);

  for (auto *sensor = link->FirstChildElement("sensor");
       sensor != nullptr;
       sensor = sensor->NextSiblingElement("sensor"))
  {
    const char *name = sensor->Attribute("name");
    if (name) {
      found.push_back(name);
    }
  }

  for (const auto &exp : expected) {
    EXPECT_NE(
      std::find(found.begin(), found.end(), exp),
      found.end())
      << "Expected sensor '" << exp << "' not found in SDF model";
  }
}

// ---------------------------------------------------------------------------
// 5. No Gazebo Classic artifacts in source directory
// ---------------------------------------------------------------------------
TEST(IntegrationMigration, NoGazeboClassicInGzPlugin)
{
  // The new gz plugin source should not reference gazebo::physics
  const std::string content = read_file(SRC_DIR "/plugin_drone_gz.cpp");
  EXPECT_EQ(content.find("gazebo::physics"), std::string::npos)
    << "plugin_drone_gz.cpp should not reference gazebo::physics (Classic API)";
  EXPECT_EQ(content.find("gazebo_ros::Node"), std::string::npos)
    << "plugin_drone_gz.cpp should not reference gazebo_ros::Node (Classic API)";
}
