/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <boost/algorithm/string/replace.hpp>
#include <string>

#include "ConveyorBeltPlugin.hh"
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)

/////////////////////////////////////////////////
ConveyorBeltPlugin::ConveyorBeltPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

//////////////////////////////////////////////////
std::string ConveyorBeltPlugin::Topic(std::string topicName) const
{
  std::string globalTopicName = "~/";
  globalTopicName += this->parentSensor->Name() + "/" + this->GetHandle() + "/" + topicName;
  boost::replace_all(globalTopicName, "::", "/");

  return globalTopicName;
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (this->updateRate > 0)
    gzdbg << "ConveyorBeltPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "ConveyorBeltPlugin running at the default update rate\n";

  if (!this->node)
  {
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->world->GetName());
  }

  if (_sdf->HasElement("belt_start_velocity"))
  {
    this->beltVelocity = _sdf->Get<double>("belt_start_velocity");
  }
  else {
    this->beltVelocity = 0.5;
  }
  gzdbg << "Using belt start velocity of: " << this->beltVelocity << " m/s\n";

  std::string controlCommandTopic;
  if (_sdf->HasElement("control_command_topic"))
  {
    controlCommandTopic = _sdf->Get<std::string>("control_command_topic");
  }
  else {
    controlCommandTopic = this->Topic("control_command");
  }
  this->controlCommandSub = this->node->Subscribe(controlCommandTopic,
      &ConveyorBeltPlugin::OnControlCommand, this);

  if (_sdf->HasElement("velocity_axis"))
  {
    this->velocityAxis = _sdf->Get<ignition::math::Vector3d>("velocity_axis");
  }
  else
  {
    gzerr << "'velocity_axis' tag not found\n";
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  double velocity;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    velocity = this->beltVelocity;
  }
  this->ActOnContactingLinks(velocity);

  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  auto prevNumberContactingLinks = this->contactingLinks.size();
  this->CalculateContactingLinks();
  if (prevNumberContactingLinks != this->contactingLinks.size()) {
    gzdbg << "Number of links ontop of belt: " << this->contactingLinks.size() << "\n";
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::ActOnContactingLinks(double velocity)
{
  ignition::math::Vector3d velocity_beltFrame = velocity * this->velocityAxis;
  auto beltPose = this->parentLink->GetWorldPose().Ign();
  math::Vector3 velocity_worldFrame = beltPose.Rot().RotateVector(velocity_beltFrame);
  for (auto linkPtr : this->contactingLinks)
  {
    if (linkPtr)
    {
      linkPtr->SetLinearVel(velocity_worldFrame);
    }
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnControlCommand(ConstHeaderPtr& _msg)
{
  double requestedVelocity = std::stod(_msg->str_id());
  gzdbg << "Received control command of: " << requestedVelocity << "\n";
  this->SetVelocity(requestedVelocity);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::SetVelocity(double velocity)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  gzdbg << "Setting velocity to: " << velocity << "\n";
  this->beltVelocity = velocity;
}
