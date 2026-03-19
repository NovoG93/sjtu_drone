// Copyright 2026 SJTU Drone Authors
// Test that the SDF model and URDF files have correct gz-sim sensors & plugin.

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#ifndef MODEL_DIR
#error "MODEL_DIR must be defined at compile time"
#endif

static const std::string kSdfPath =
    std::string(MODEL_DIR) + "/sjtu_drone.sdf";

// ---------------------------------------------------------------------------
// SDF Model Tests
// ---------------------------------------------------------------------------
class SdfModelTest : public ::testing::Test {
 protected:
  sdf::Root root;
  const sdf::Model *model{nullptr};

  void SetUp() override {
    ASSERT_TRUE(std::filesystem::exists(kSdfPath))
        << "SDF file not found: " << kSdfPath;
    auto errors = root.Load(kSdfPath);
    ASSERT_TRUE(errors.empty())
        << "SDF parse errors: " << errors.front().Message();
    model = root.Model();
    ASSERT_NE(model, nullptr) << "No model found in SDF file";
  }
};

TEST_F(SdfModelTest, ModelNameIsSjtuDrone) {
  EXPECT_EQ(model->Name(), "sjtu_drone");
}

TEST_F(SdfModelTest, HasBaseFootprintLink) {
  ASSERT_GT(model->LinkCount(), 0u);
  const auto *link = model->LinkByName("base_footprint");
  EXPECT_NE(link, nullptr) << "base_footprint link not found";
}

TEST_F(SdfModelTest, HasDronePlugin) {
  const auto &plugins = model->Plugins();
  bool found = false;
  for (const auto &p : plugins) {
    if (p.Filename().find("libplugin_drone") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "DroneSimpleController plugin not found on model";
}

TEST_F(SdfModelTest, DronePluginHasNoRosBlock) {
  // gz-sim plugins should NOT have <ros> blocks — bridging is external
  const auto &plugins = model->Plugins();
  for (const auto &p : plugins) {
    if (p.Filename().find("libplugin_drone") != std::string::npos) {
      // Check raw SDF element for absence of <ros> child
      auto elem = p.ToElement();
      ASSERT_NE(elem, nullptr);
      EXPECT_EQ(elem->FindElement("ros"), nullptr)
          << "Drone plugin should not contain <ros> block in gz-sim";
    }
  }
}

// Helper: collect all sensors from all links
static std::vector<const sdf::Sensor *> CollectSensors(
    const sdf::Model *model) {
  std::vector<const sdf::Sensor *> sensors;
  for (uint64_t i = 0; i < model->LinkCount(); ++i) {
    const auto *link = model->LinkByIndex(i);
    for (uint64_t j = 0; j < link->SensorCount(); ++j) {
      sensors.push_back(link->SensorByIndex(j));
    }
  }
  return sensors;
}

static const sdf::Sensor *FindSensor(
    const std::vector<const sdf::Sensor *> &sensors,
    const std::string &name) {
  for (const auto *s : sensors) {
    if (s->Name() == name) return s;
  }
  return nullptr;
}

TEST_F(SdfModelTest, HasImuSensor) {
  auto sensors = CollectSensors(model);
  const auto *s = FindSensor(sensors, "sensor_imu");
  ASSERT_NE(s, nullptr) << "IMU sensor 'sensor_imu' not found";
  EXPECT_EQ(s->Type(), sdf::SensorType::IMU);
}

TEST_F(SdfModelTest, HasNavSatSensor) {
  auto sensors = CollectSensors(model);
  const auto *s = FindSensor(sensors, "navsat_sensor");
  ASSERT_NE(s, nullptr) << "NavSat sensor 'navsat_sensor' not found";
  EXPECT_EQ(s->Type(), sdf::SensorType::NAVSAT);
}

TEST_F(SdfModelTest, HasFrontCamera) {
  auto sensors = CollectSensors(model);
  const auto *s = FindSensor(sensors, "front_camera");
  ASSERT_NE(s, nullptr) << "Front camera sensor not found";
  EXPECT_EQ(s->Type(), sdf::SensorType::CAMERA);
}

TEST_F(SdfModelTest, HasBottomCamera) {
  auto sensors = CollectSensors(model);
  const auto *s = FindSensor(sensors, "down_camera");
  ASSERT_NE(s, nullptr) << "Bottom camera sensor 'down_camera' not found";
  EXPECT_EQ(s->Type(), sdf::SensorType::CAMERA);
}

TEST_F(SdfModelTest, HasSonarGpuLidar) {
  auto sensors = CollectSensors(model);
  const auto *s = FindSensor(sensors, "sonar");
  ASSERT_NE(s, nullptr) << "Sonar sensor not found";
  EXPECT_EQ(s->Type(), sdf::SensorType::GPU_LIDAR);
}

TEST_F(SdfModelTest, HasContactSensor) {
  auto sensors = CollectSensors(model);
  const auto *s = FindSensor(sensors, "collision_sensor");
  ASSERT_NE(s, nullptr) << "Contact sensor 'collision_sensor' not found";
  EXPECT_EQ(s->Type(), sdf::SensorType::CONTACT);
}

TEST_F(SdfModelTest, SensorsHaveNoClassicPlugins) {
  // None of the sensors should have gazebo_ros Classic plugin children
  auto sensors = CollectSensors(model);
  for (const auto *s : sensors) {
    auto elem = s->Element();
    if (!elem) continue;
    auto pluginElem = elem->FindElement("plugin");
    while (pluginElem) {
      std::string fname;
      if (pluginElem->HasAttribute("filename")) {
        fname = pluginElem->Get<std::string>("filename");
      }
      EXPECT_TRUE(fname.find("libgazebo_ros") == std::string::npos)
          << "Sensor '" << s->Name()
          << "' still has Classic plugin: " << fname;
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }
}

TEST_F(SdfModelTest, MeshUrisPresent) {
  const auto *link = model->LinkByName("base_footprint");
  ASSERT_NE(link, nullptr);
  // At least one visual and one collision should exist
  EXPECT_GT(link->VisualCount(), 0u) << "No visuals on base_footprint";
  EXPECT_GT(link->CollisionCount(), 0u) << "No collisions on base_footprint";
}

TEST_F(SdfModelTest, SdfVersionIs1_9) {
  // Verify the file starts with sdf version 1.9 by re-reading raw text
  std::ifstream f(kSdfPath);
  ASSERT_TRUE(f.is_open());
  std::string content((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("version='1.9'"), std::string::npos)
      << "SDF version should be 1.9";
}
