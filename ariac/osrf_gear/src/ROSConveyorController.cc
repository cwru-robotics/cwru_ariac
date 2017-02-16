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
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <string>

// ROS
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/ConveyorBeltState.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Model.h>
#include <osrf_gear/Proximity.h>
#include <ros/ros.h>

namespace gazebo
{
class ROSConveyorController : public WorldPlugin
{
  private: ros::NodeHandle* rosnode;
  private: ros::ServiceClient controlClient;
  private: ros::Subscriber proximitySensorSub;
  private: ros::Subscriber breakBeamSub;
  private: physics::WorldPtr world;
  private: double beltVelocity;

  private: ros::Subscriber logicalCameraImageSub;
  private: std::string searchModelType = "unit_box";
  private: bool modelDetected = false;

  public: ~ROSConveyorController()
  {
    this->rosnode->shutdown();
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->world = _parent;
    this->rosnode = new ros::NodeHandle("");

    // Create a subscriber for the proximity sensor output
    std::string sensorStateChangeTopic = "/ariac/proximity_sensor_changed";
    this->proximitySensorSub =
      this->rosnode->subscribe(sensorStateChangeTopic, 1000,
        &ROSConveyorController::OnSensorStateChange, this);

    // Create a subscriber for the break beam output
    std::string breakBeamStateChangeTopic = "/ariac/break_beam_changed";
    this->breakBeamSub =
      this->rosnode->subscribe(breakBeamStateChangeTopic, 1000,
        &ROSConveyorController::OnSensorStateChange, this);

    // Create a subscriber for the logical camera images
    std::string logicalCameraImageTopic = "/ariac/logical_camera";
    this->logicalCameraImageSub =
      this->rosnode->subscribe(logicalCameraImageTopic, 1000,
        &ROSConveyorController::OnLogicalCameraImage, this);

    // Create a client for the conveyor control commands
    std::string conveyorControlTopic = "/conveyor/control";
    this->controlClient = this->rosnode->serviceClient<osrf_gear::ConveyorBeltControl>(
      conveyorControlTopic);

    this->beltVelocity = 0.5;

    // Turn belt on
    this->SendControlRequest(this->beltVelocity);
  }

  private: void OnSensorStateChange(const osrf_gear::Proximity::ConstPtr &_msg)
  {
    gzdbg << "Sensor state changed\n";

    bool sensorValue = _msg->object_detected;
    bool controlCommand = !sensorValue; // on (true) or off (false)
    this->SendControlRequest(this->beltVelocity * controlCommand);
  }

  private: void SendControlRequest(double velocity)
  {
    osrf_gear::ConveyorBeltState controlMsg;
    controlMsg.velocity = velocity;
    osrf_gear::ConveyorBeltControl controlRequest;
    controlRequest.request.state = controlMsg;
    this->controlClient.call(controlRequest);
  }

  private: void OnLogicalCameraImage(const osrf_gear::LogicalCameraImage::ConstPtr &_msg)
  {
    bool modelDetected = false;
    for (osrf_gear::Model model : _msg->models)
    {
      if (model.type == this->searchModelType)
      {
        modelDetected = true;
        break;
      }
    }
    if (modelDetected != this->modelDetected)
    {
      this->SendControlRequest(!modelDetected * this->beltVelocity);
      this->modelDetected = modelDetected;
      if (!modelDetected)
        gzdbg << "Object no longer detected by logical camera\n";
      else
        gzdbg << "Object detected by logical camera\n";
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ROSConveyorController)
}
