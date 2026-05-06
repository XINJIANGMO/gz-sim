/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gtest/gtest.h>

#include <gz/msgs/double.pb.h>

#include <optional>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/JointForceCmd.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/ResetUtils.hh"
#include "../helpers/Util.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test fixture for ApplyJointForce system
class ApplyJointForceTestFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Tests that the ApplyJointForce accepts joint velocity commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ApplyJointForceTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointVelocityCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/apply_joint_force.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Slow update period to increase the likelihood of published data arriving
  // before iterations are completed
  server.SetUpdatePeriod(1ms);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> jointForceCmd;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        auto forceComp = _ecm.Component<components::JointForceCmd>(joint);
        if (forceComp)
        {
          jointForceCmd.push_back(forceComp->Data()[0]);
        }
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_EQ(initIters, jointForceCmd.size());
  for (const auto &jointForce : jointForceCmd)
  {
    EXPECT_NEAR(0, jointForce, TOL);
  }

  jointForceCmd.clear();

  // Publish command and check that the joint force is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_force_test/joint/j1/cmd_force");

  const double testJointForce{0.001};
  msgs::Double msg;
  msg.set_data(testJointForce);

  pub.Publish(msg);
  // Wait for the message to be published
  const std::size_t maxIters = 1000;
  for (std::size_t i = 0; i < maxIters; ++i)
  {
    server.Run(true, 1, false);
    if (std::abs(jointForceCmd.back() - testJointForce) < 1e-6)
    {
      break;
    }
  }
  EXPECT_DOUBLE_EQ(jointForceCmd.back(), testJointForce);
}

/////////////////////////////////////////////////
TEST_F(ApplyJointForceTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetStateContamination))
{
  using namespace std::chrono_literals;

  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/apply_joint_force.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1ms);

  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_force_test/joint/j1/cmd_force");

  const std::string jointName = "j1";
  std::optional<double> jointPosition;
  std::optional<double> jointVelocity;

  test::Relay testSystem;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        const auto joint = _ecm.EntityByComponents(components::Joint(),
            components::Name(jointName));
        ASSERT_NE(kNullEntity, joint);

        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }

        if (nullptr == _ecm.Component<components::JointVelocity>(joint))
        {
          _ecm.CreateComponent(joint, components::JointVelocity());
        }
      });
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        const auto joint = _ecm.EntityByComponents(components::Joint(),
            components::Name(jointName));
        ASSERT_NE(kNullEntity, joint);

        const auto positionComp =
            _ecm.Component<components::JointPosition>(joint);
        if (positionComp != nullptr && !positionComp->Data().empty())
        {
          jointPosition = positionComp->Data()[0];
        }

        const auto velocityComp =
            _ecm.Component<components::JointVelocity>(joint);
        if (velocityComp != nullptr && !velocityComp->Data().empty())
        {
          jointVelocity = velocityComp->Data()[0];
        }
      });
  server.AddSystem(testSystem.systemPtr);

  server.Run(true, 10, false);
  ASSERT_TRUE(jointPosition.has_value());
  ASSERT_TRUE(jointVelocity.has_value());
  EXPECT_TRUE(test::WaitUntil(2s, [&]
      {
        return pub.HasConnections();
      }));

  // Drive the joint away from its initial state so reset has something to
  // clean up.
  constexpr double testJointForce = 0.001;
  msgs::Double msg;
  msg.set_data(testJointForce);
  EXPECT_TRUE(pub.Publish(msg));
  std::this_thread::sleep_for(100ms);
  ASSERT_TRUE(test::StepUntil(server, 1000u, [&]
      {
        return jointPosition.has_value() && jointVelocity.has_value() &&
            (std::abs(*jointPosition) > 1e-6 ||
             std::abs(*jointVelocity) > 1e-6);
      }));
  const auto preResetPosition = *jointPosition;
  EXPECT_GT(std::abs(preResetPosition), 1e-6);

  // Stop publishing before reset; any post-reset motion now comes from stale
  // plugin state rather than a fresh transport command.
  gz::sim::test::reset::RequestAndApplyWorldReset(server, "default");

  ASSERT_TRUE(test::StepUntil(server, 500u, [&]
      {
        return jointPosition.has_value() && jointVelocity.has_value() &&
            std::abs(*jointPosition) < 1e-6 &&
            std::abs(*jointVelocity) < 1e-6;
      }));

  // Re-advertise after reset so the fresh plugin instance reconnects through
  // a clean transport publisher.
  transport::Node postResetNode;
  auto postResetPub = postResetNode.Advertise<msgs::Double>(
      "/model/joint_force_test/joint/j1/cmd_force");
  EXPECT_TRUE(test::WaitUntil(2s, [&]
      {
        return postResetPub.HasConnections();
      }));
  std::this_thread::sleep_for(300ms);
  EXPECT_TRUE(postResetPub.Publish(msg));
  std::this_thread::sleep_for(100ms);
  ASSERT_TRUE(test::StepUntil(server, 1000u, [&]
      {
        return jointPosition.has_value() && jointVelocity.has_value() &&
            (std::abs(*jointPosition) > 1e-6 ||
             std::abs(*jointVelocity) > 1e-6);
      }));
}
