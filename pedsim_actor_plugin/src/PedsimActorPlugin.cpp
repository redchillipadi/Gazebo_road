/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "PedsimActorPlugin.h"
#include <vector>
#include <thread>
#include <sstream>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedsimActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
PedsimActorPlugin::PedsimActorPlugin()
{
}

/////////////////////////////////////////////////
void PedsimActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(std::bind(&PedsimActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target
  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, 0, 0);

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Set up Gazebo transport

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  std::string topicName = "~/" + this->model->GetName() + "/target";
  this->sub = this->node->Subscribe(topicName, &PedsimActorPlugin::OnMsg, this);

  // Set up ROS transport

  std::string rosNodeName = "gazebo_client";
  std::string rosTopicName = "/" + this->model->GetName() + "/target";

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, rosNodeName, ros::init_options::NoSigintHandler);
  }

  this->rosNode.reset(new ros::NodeHandle(rosNodeName));
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>(rosTopicName, 1, boost::bind(&PedsimActorPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Subscribe to Pedsim Messages
  std::string pedsimTopic = "/pedsim_simulator/simulated_agents";
  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(pedsimTopic, 1, boost::bind(&PedsimActorPlugin::pedsimCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
  this->rosSub2 = this->rosNode->subscribe(so2);

  // Spin up the queue helper thread
  this->rosQueueThread = std::thread(std::bind(&PedsimActorPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void PedsimActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

////////////////////////////////////////////////
void PedsimActorPlugin::HandleObstacles(ignition::math::Vector3d & /*_pos*/)
{
  return;
}

/////////////////////////////////////////////////
void PedsimActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  // Normalize the direction vector.
  pos = pos.Normalize();

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // If the target yaw is too far, rotate towards it rather than snapping
  // TODO: Make the snapping rotation limit and scaling factor parameters in the SDF
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian() * 0.001);
  else
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian());

  // Always move towards target
  pose.Pos() += pos * this->velocity * dt;
  // vs Snap to target
  // pose.Pos() += this->target - pose.Pos();

  // Height must always be 1m for Actor so feet touch the ground (assuming Actor model used)
  // TODO: Make this a parameter in te SDF
  pose.Pos().Z(1.0);

  // Distance traveled is used to coordinate motion with the walking animation
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

/////////////////////////////////////////////////
void PedsimActorPlugin::pedsimCallback(const pedsim_msgs::AgentStatesConstPtr& msg)
{
  // All the pedsim messages seem to use odom

  for (auto& agent : msg->agent_states)
  {
    std::stringstream ss;
    ss << "id: " << agent.id << " type: " << agent.type << " social: " << agent.social_state << " pose: ("
      << agent.pose.position.x << ", " << agent.pose.position.y << ", " << agent.pose.position.z << ") orientation: ("
      << agent.pose.orientation.x << ", " << agent.pose.orientation.y << ", " << agent.pose.orientation.z << ", " << agent.pose.orientation.w << ") twist: Linear ("
      << agent.twist.linear.x << ", " << agent.twist.linear.y << ", " << agent.twist.linear.z << ") Angular ("
      << agent.twist.angular.x << ", " << agent.twist.angular.y << ", " << agent.twist.angular.z << ")" << std::endl;
    std::cout << ss.str();
  }
}

void PedsimActorPlugin::SetTarget(double x, double y, double z)
{
  this->target.X(x);
  this->target.Y(y);
  this->target.Z(z);
}

void PedsimActorPlugin::OnMsg(ConstVector3dPtr& msg)
{
  this->SetTarget(msg->x(), msg->y(), msg->z());
}

void PedsimActorPlugin::OnRosMsg(const geometry_msgs::Vector3ConstPtr& msg)
{
  this->SetTarget(msg->x, msg->y, msg->z);
}

void PedsimActorPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
