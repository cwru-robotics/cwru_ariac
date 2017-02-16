/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include "ROSConveyorBeltPlugin.hh"

#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin);

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
  this->rosnode_->shutdown();
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    this->robotNamespace_ = _sdf->GetElement(
        "robot_namespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string topic = "conveyor/control";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  this->controlService_ = this->rosnode_->advertiseService(topic,
    &ROSConveyorBeltPlugin::OnControlCommand, this);
}

/////////////////////////////////////////////////
bool ROSConveyorBeltPlugin::OnControlCommand(
  osrf_gear::ConveyorBeltControl::Request &_req,
  osrf_gear::ConveyorBeltControl::Response &_res)
{
  gzdbg << "Control command received\n";
  this->SetVelocity(_req.state.velocity);
  _res.success = true;
  return true;
}
