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
#include <string>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class ConveyorController : public WorldPlugin
{
  private: transport::PublisherPtr controlPub;
  private: transport::SubscriberPtr sensorSub;
  private: physics::WorldPtr world;
  private: double beltVelocity;

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->world = _parent;
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(_parent->GetName());

    // Create a subscriber for the proximity sensor output
    std::string sensorStateChangeTopic =
      "~/laser/proximity_ray_plugin/state_change";
    sensorSub =
      node->Subscribe(sensorStateChangeTopic, &ConveyorController::OnSensorStateChange, this);

    // Create a publisher for the conveyor control commands
    std::string conveyorControlTopic =
      "~/conveyor_belt_contact/conveyor_belt_plugin/control_command";
    this->controlPub =
      node->Advertise<msgs::Header>(conveyorControlTopic);

    this->beltVelocity = 0.5;

    // Create the message
    // TODO: make custom belt control message (or add Double to standard gazebo msgs...)
    msgs::Header msg;
    bool controlCommand = true;
    double desiredVelocity = controlCommand*this->beltVelocity;
    msg.set_str_id(std::to_string(desiredVelocity));
    msgs::Set(msg.mutable_stamp(), _parent->GetSimTime());

    // Send the message
    this->controlPub->Publish(msg);
  }

  private: void OnSensorStateChange(ConstHeaderPtr& _msg)
  {
    gzdbg << "Sensor state changed.\n";
    msgs::Header msg;

    bool sensorValue = _msg->index();
    bool controlCommand;
    std::string outputFunction = _msg->str_id();
    if ("normally_open" == outputFunction) {
      controlCommand = !sensorValue;
    }
    else if ("normally_closed" == outputFunction) {
      controlCommand = sensorValue;
    }
    else {
      gzerr << "output_function can only be either normally_open or normally_closed" << std::endl;
      return;
    }
    double desiredVelocity = controlCommand*this->beltVelocity;
    msg.set_str_id(std::to_string(desiredVelocity));
    msgs::Set(msg.mutable_stamp(), this->world->GetSimTime());

    // Send the message
    this->controlPub->Publish(msg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ConveyorController)
}
