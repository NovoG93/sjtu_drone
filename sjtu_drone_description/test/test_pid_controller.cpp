// Copyright 2023 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.en.html

#include <gtest/gtest.h>
#include <sdf/sdf.hh>
#include "sjtu_drone_description/pid_controller.h"

class PIDControllerTest : public ::testing::Test {
protected:
  PIDController pid;
};

// --- Constructor / default state ---

TEST_F(PIDControllerTest, DefaultConstructor) {
  // Members should be zero-initialised (or undefined). After Load(nullptr)
  // the gains get defaults, so we test that separately.
  // Just verify the object is constructable.
  SUCCEED();
}

// --- Loading gains from SDF element ---

TEST_F(PIDControllerTest, LoadWithNullSDF) {
  pid.Load(nullptr);
  // Defaults when no SDF provided
  EXPECT_DOUBLE_EQ(pid.gain_p, 5.0);
  EXPECT_DOUBLE_EQ(pid.gain_d, 1.0);
  EXPECT_DOUBLE_EQ(pid.gain_i, 0.0);
  EXPECT_DOUBLE_EQ(pid.time_constant, 0.0);
  EXPECT_DOUBLE_EQ(pid.limit, -1.0);
}

TEST_F(PIDControllerTest, LoadFromSDFElement) {
  std::string sdfStr =
    "<sdf version='1.6'>"
    "  <plugin name='test' filename='test'>"
    "    <ProportionalGain>10.0</ProportionalGain>"
    "    <DifferentialGain>2.0</DifferentialGain>"
    "    <IntegralGain>0.5</IntegralGain>"
    "    <TimeConstant>0.1</TimeConstant>"
    "    <Limit>100.0</Limit>"
    "  </plugin>"
    "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::readString(sdfStr, sdfParsed);

  sdf::ElementPtr pluginElem = sdfParsed->Root()->GetElement("plugin");
  pid.Load(pluginElem);

  EXPECT_DOUBLE_EQ(pid.gain_p, 10.0);
  EXPECT_DOUBLE_EQ(pid.gain_d, 2.0);
  EXPECT_DOUBLE_EQ(pid.gain_i, 0.5);
  EXPECT_DOUBLE_EQ(pid.time_constant, 0.1);
  EXPECT_DOUBLE_EQ(pid.limit, 100.0);
}

TEST_F(PIDControllerTest, LoadFromSDFWithPrefix) {
  std::string sdfStr =
    "<sdf version='1.6'>"
    "  <plugin name='test' filename='test'>"
    "    <rollProportionalGain>8.0</rollProportionalGain>"
    "    <rollDifferentialGain>3.0</rollDifferentialGain>"
    "    <rollIntegralGain>0.2</rollIntegralGain>"
    "    <rollTimeConstant>0.05</rollTimeConstant>"
    "    <rollLimit>50.0</rollLimit>"
    "  </plugin>"
    "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::readString(sdfStr, sdfParsed);

  sdf::ElementPtr pluginElem = sdfParsed->Root()->GetElement("plugin");
  pid.Load(pluginElem, "roll");

  EXPECT_DOUBLE_EQ(pid.gain_p, 8.0);
  EXPECT_DOUBLE_EQ(pid.gain_d, 3.0);
  EXPECT_DOUBLE_EQ(pid.gain_i, 0.2);
  EXPECT_DOUBLE_EQ(pid.time_constant, 0.05);
  EXPECT_DOUBLE_EQ(pid.limit, 50.0);
}

TEST_F(PIDControllerTest, LoadPartialSDF) {
  // Only set proportional gain; rest should be defaults
  std::string sdfStr =
    "<sdf version='1.6'>"
    "  <plugin name='test' filename='test'>"
    "    <ProportionalGain>7.0</ProportionalGain>"
    "  </plugin>"
    "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::readString(sdfStr, sdfParsed);

  sdf::ElementPtr pluginElem = sdfParsed->Root()->GetElement("plugin");
  pid.Load(pluginElem);

  EXPECT_DOUBLE_EQ(pid.gain_p, 7.0);
  EXPECT_DOUBLE_EQ(pid.gain_d, 1.0);   // default
  EXPECT_DOUBLE_EQ(pid.gain_i, 0.0);   // default
  EXPECT_DOUBLE_EQ(pid.time_constant, 0.0);
  EXPECT_DOUBLE_EQ(pid.limit, -1.0);
}

// --- Proportional behavior ---

TEST_F(PIDControllerTest, ProportionalOnly) {
  pid.Load(nullptr);  // gain_p=5, gain_d=1, gain_i=0
  // Override to isolate proportional
  pid.gain_p = 2.0;
  pid.gain_d = 0.0;
  pid.gain_i = 0.0;
  pid.reset();

  // input=1.0, x=0.0, dx=0.0, dt=0.1
  // With time_constant=0: input = (0.1*1.0 + 0*0)/(0.1+0) = 1.0
  // p = input - x = 1.0 - 0.0 = 1.0
  // output = gain_p * p = 2.0 * 1.0 = 2.0
  double out = pid.update(1.0, 0.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(out, 2.0);
}

// --- Integral behavior ---

TEST_F(PIDControllerTest, IntegralAccumulates) {
  pid.Load(nullptr);
  pid.gain_p = 0.0;
  pid.gain_d = 0.0;
  pid.gain_i = 1.0;
  pid.reset();

  double dt = 0.1;
  // Step 1: input=1, x=0 → p=1, i += dt*p = 0.1
  pid.update(1.0, 0.0, 0.0, dt);
  EXPECT_NEAR(pid.i, 0.1, 1e-9);

  // Step 2: same → i += 0.1 → i=0.2
  pid.update(1.0, 0.0, 0.0, dt);
  EXPECT_NEAR(pid.i, 0.2, 1e-9);

  // Output should be gain_i * i = 1.0 * 0.2 = 0.2
  EXPECT_NEAR(pid.output, 0.2, 1e-9);
}

// --- Derivative behavior ---

TEST_F(PIDControllerTest, DerivativeResponse) {
  pid.Load(nullptr);
  pid.gain_p = 0.0;
  pid.gain_d = 1.0;
  pid.gain_i = 0.0;
  pid.reset();

  double dt = 0.1;
  // First call: input=1, x=0, dx=0
  // dinput = (1.0 - 1.0)/(0.1) = 0  (since input becomes 1.0 immediately when time_constant=0)
  // d = dinput - dx = 0 - 0 = 0
  pid.update(1.0, 0.0, 0.0, dt);
  // With time_constant=0: dinput = (new_input - input)/(dt + tc) = (1.0 - 1.0)/0.1 = 0
  // (because input is set to new_input when tc=0)
  EXPECT_DOUBLE_EQ(pid.d, 0.0);

  // If we now change input, there will be a derivative
  // new_input=2.0, input was 1.0 from previous step
  // input = (0.1*2.0 + 0*1.0)/(0.1+0) = 2.0
  // dinput = (2.0 - 2.0)/(0.1+0) = 0  ... still 0 because tc=0 makes it instant
  // Actually let's use a time constant to see derivative behavior
  pid.reset();
  pid.time_constant = 0.1;
  // Step 1
  pid.update(1.0, 0.0, 0.0, dt);
  // input = (0.1*1.0 + 0.1*0)/(0.1+0.1) = 0.1/0.2 = 0.5
  // dinput = (1.0 - 0.5)/(0.1+0.1) = 0.5/0.2 = 2.5
  EXPECT_NEAR(pid.dinput, 2.5, 1e-9);
  // d = dinput - dx = 2.5 - 0 = 2.5
  EXPECT_NEAR(pid.d, 2.5, 1e-9);
}

// --- Update with known inputs ---

TEST_F(PIDControllerTest, UpdateFullPID) {
  pid.Load(nullptr);  // gain_p=5, gain_d=1, gain_i=0, tc=0, limit=-1
  pid.reset();

  double dt = 0.1;
  // new_input=1.0, x=0.5, dx=0.1
  // input = (0.1*1.0 + 0*0)/(0.1+0) = 1.0
  // dinput = (1.0 - 1.0)/(0.1+0) = 0
  // p = 1.0 - 0.5 = 0.5
  // d = 0.0 - 0.1 = -0.1
  // i = 0 + 0.1*0.5 = 0.05
  // output = 5*0.5 + 1*(-0.1) + 0*0.05 = 2.5 - 0.1 = 2.4
  double out = pid.update(1.0, 0.5, 0.1, dt);
  EXPECT_NEAR(out, 2.4, 1e-9);
}

// --- Limit clamping ---

TEST_F(PIDControllerTest, LimitClampsInput) {
  pid.Load(nullptr);
  pid.gain_p = 1.0;
  pid.gain_d = 0.0;
  pid.gain_i = 0.0;
  pid.limit = 2.0;
  pid.reset();

  // Input exceeds limit: should be clamped to 2.0
  pid.update(5.0, 0.0, 0.0, 0.1);
  EXPECT_NEAR(pid.input, 2.0, 1e-9);

  // Negative input exceeds limit: should be clamped to -2.0
  pid.reset();
  pid.update(-5.0, 0.0, 0.0, 0.1);
  EXPECT_NEAR(pid.input, -2.0, 1e-9);
}

// --- Reset ---

TEST_F(PIDControllerTest, ResetClearsState) {
  pid.Load(nullptr);
  pid.reset();

  // Drive some state
  pid.update(1.0, 0.0, 0.0, 0.1);
  EXPECT_NE(pid.output, 0.0);

  pid.reset();
  EXPECT_DOUBLE_EQ(pid.input, 0.0);
  EXPECT_DOUBLE_EQ(pid.dinput, 0.0);
  EXPECT_DOUBLE_EQ(pid.p, 0.0);
  EXPECT_DOUBLE_EQ(pid.i, 0.0);
  EXPECT_DOUBLE_EQ(pid.d, 0.0);
  EXPECT_DOUBLE_EQ(pid.output, 0.0);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
