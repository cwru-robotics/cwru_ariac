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
/*
 * Desc: Kit tray plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_KIT_TRAY_PLUGIN_HH_
#define _GAZEBO_KIT_TRAY_PLUGIN_HH_

#include <string>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>
#include <osrf_gear/ARIAC.hh>
#include "SideContactPlugin.hh"

namespace gazebo
{
  /// \brief A plugin for a contact sensor on a kit tray.
  class GAZEBO_VISIBLE KitTrayPlugin : public SideContactPlugin
  {
    /// \brief Constructor.
    public: KitTrayPlugin();

    /// \brief Destructor.
    public: virtual ~KitTrayPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Update the kit based on which models are in contact
    protected: void ProcessContactingModels();

    /// \brief Update the kit based on which models are in contact
    public: std::string DetermineModelType(const std::string &modelName);

    /// \brief Publish the Kit ROS message
    protected: void PublishKitMsg();

    /// \brief Kit which is currently on the tray
    protected: ariac::Kit currentKit;

    /// \brief ID of tray
    protected: std::string trayID;

    /// \brief ROS node handle
    protected: ros::NodeHandle *rosNode;

    /// \brief Publisher for the kit state
    protected: ros::Publisher currentKitPub;
  };
}
#endif

