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

#ifndef GAZEBO_PLUGINS_PEDSIACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_PEDSIMACTORPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Quaternion.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pedsim_msgs/AgentStates.h>

namespace gazebo
{
  class GAZEBO_VISIBLE PedsimActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
  public:
    PedsimActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
  public:
    virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
  private:
    void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    // private: void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
  private:
    void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the parent actor.
  private:
    physics::ActorPtr actor;

    /// \brief Pointer to the model
  private:
    physics::ModelPtr model;

    /// \brief Pointer to the world, for convenience.
  private:
    physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
  private:
    sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
  private:
    ignition::math::Vector3d velocity;

    /// \brief List of connections
  private:
    std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
  private:
    ignition::math::Vector3d target;

    /// \brief Current target orientation
  private:
    ignition::math::Quaterniond orientation;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
  private:
    double animationFactor = 1.0;

    /// \brief ID of Pedsim Agent to duplicate
  private:
    uint64_t pedsimId = 0;

    /// \brief Whether to interpolate betewen positions (true) or to snap to new location on updates (false)
  private:
    bool interpolate = false;

    /// \brief Time of the last update.
  private:
    common::Time lastUpdate;

    /// \brief Custom trajectory info.
  private:
    physics::TrajectoryInfoPtr trajectoryInfo;

    // Function access and Gazebo API
    /// \brief External API to set target position
  public:
    void SetTarget(double x, double y, double z);

    /// @brief Set orientation using a quaternion
    /// @param x The x parameter of the quaternion
    /// @param y The y parameter of the quaternion
    /// @param z The z parameter of the quaternion
    /// @param w The w parameter of the quaternion
  public:
    void SetOrientation(double x, double y, double z, double w);

    /// \brief A node used for transport
  private:
    transport::NodePtr node;

    /// \brief A subscriber to the named topic
  private:
    transport::SubscriberPtr sub;

    /// \brief Handle the incoming message
    /// \param[in] msg Pass a vector3 message with the target location
  private:
    void OnMsg(ConstVector3dPtr &msg);

    // TODO: Add gazebo function for setting orientation quaternion and velocity if desired

    // ROS API
    /// \brief A node used for ROS transport
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
  private:
    ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
  private:
    ros::CallbackQueue rosQueue;

    /// \brief A thrad that keeps running the rosQueue
  private:
    std::thread rosQueueThread;

    /// \brief Callback used for ROS updates
  private:
    void OnRosMsg(const geometry_msgs::Vector3ConstPtr &msg);

    /// \brief Helper function to process messages
  private:
    void QueueThread();

    // Subscribe directly to PedSim

    /// \brief A ROS subscriber for pedsim messages
  private:
    ros::Subscriber rosSub2;

    /// \brief Callback for pedsim updates
  private:
    void pedsimCallback(const pedsim_msgs::AgentStatesConstPtr &msg);
  };
}
#endif
