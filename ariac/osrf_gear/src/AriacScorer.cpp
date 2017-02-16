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

#include <gazebo/common/Console.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>

#include "osrf_gear/AriacScorer.h"

/////////////////////////////////////////////////
AriacScorer::AriacScorer()
{
}

/////////////////////////////////////////////////
AriacScorer::~AriacScorer()
{
}

/////////////////////////////////////////////////
ariac::GameScore AriacScorer::GetGameScore()
{
  return this->gameScore;
}

/////////////////////////////////////////////////
ariac::OrderScore AriacScorer::GetCurrentOrderScore()
{
  return *this->orderScore;
}

/////////////////////////////////////////////////
void AriacScorer::Update(double timeStep)
{
  if (this->isPartTravelling)
  {
    this->gameScore.partTravelTime += timeStep;
  }

  boost::mutex::scoped_lock kitTraysLock(this->kitTraysMutex);

  if (this->newOrderReceived)
  {
    gzdbg << "New order received: " << this->newOrder.orderID << std::endl;
    this->AssignOrder(this->newOrder);
  }

  // During the competition, this environment variable will be set.
  auto v = std::getenv("ARIAC_COMPETITION");
  if (!v)
  {
    // Check score of trays in progress.
    if (this->newOrderReceived || this->newTrayInfoReceived)
    {
      this->ScoreCurrentState();
    }
  }

  this->newOrderReceived = false;
  this->newTrayInfoReceived = false;
}

/////////////////////////////////////////////////
bool AriacScorer::IsCurrentOrderComplete()
{
  return this->orderScore->isComplete();
}

/////////////////////////////////////////////////
void AriacScorer::ScoreCurrentState()
{
  gzdbg << "Scoring current state." << std::endl;
  for (const auto & item : this->kitTrays)
  {
    auto trayID = item.first;
  gzdbg << "Scoring tray: " << trayID << std::endl;
    auto tray = item.second;
    if (tray.currentKit.kitType != "")
    {
      auto trayScore = ScoreTray(tray);
          std::ostringstream logStream;
      logStream << "Score from tray '" << trayID << \
        "' with kit type '" << tray.currentKit.kitType << "': " << trayScore.total();
      ROS_INFO_STREAM(logStream.str().c_str());
      gzdbg << logStream.str().c_str() << std::endl;
    }
    else
    {
      for (const auto & item : this->currentOrder.kits)
      {
        auto kit = item.second;
        auto orderKitType = kit.kitType;
        tray.currentKit.kitType = orderKitType;
        auto trayScore = ScoreTray(tray);
        if (trayScore.total() > 0)
        {
          std::ostringstream logStream;
          logStream << "Score from tray '" << trayID \
            << "' if it were to have kit type '" << orderKitType << "': " \
            << trayScore.total();
          ROS_INFO_STREAM(logStream.str().c_str());
          gzdbg << logStream.str().c_str() << std::endl;
        }
      }
    }
  }
  gzdbg << "Finished scoring current state." << std::endl;
}

/////////////////////////////////////////////////
bool AriacScorer::GetTrayById(const ariac::TrayID_t & trayID, ariac::KitTray & kitTray)
{
  auto it = this->kitTrays.find(trayID);
  if (it == this->kitTrays.end())
  {
    gzwarn << "No known tray with ID: " << trayID << std::endl;
    return false;
  }
  kitTray = it->second;
  return true;
}

/////////////////////////////////////////////////
ariac::TrayScore AriacScorer::SubmitTray(const ariac::KitTray & tray)
{
  auto trayScore = ScoreTray(tray);
  gzdbg << "Score from tray '" << tray.trayID << "': " << trayScore.total() << std::endl;
  this->orderScore->trayScores[tray.trayID] = trayScore;
  this->orderScore->trayScores[tray.trayID].isSubmitted = true;
  return trayScore;
}

/////////////////////////////////////////////////
ariac::TrayScore AriacScorer::ScoreTray(const ariac::KitTray & tray)
{
  ariac::Kit kit = tray.currentKit;
  ariac::KitType_t kitType = tray.currentKit.kitType;
  ariac::TrayScore score;
  if (this->currentOrder.kits.find(kitType) == this->currentOrder.kits.end())
  {
    gzdbg << "No known kit type: " << kitType << std::endl;
    gzdbg << "Known kit types: " << std::endl;
    for (auto item : this->currentOrder.kits)
    {
      gzdbg << item.first << std::endl;
    }
    gzdbg << "Current order: " << this->currentOrder << std::endl;
    return score;
  }
  ariac::Kit assignedKit = this->currentOrder.kits[kitType];
  auto numAssignedObjects = assignedKit.objects.size();
  gzdbg << "Comparing the " << numAssignedObjects << " assigned objects with the current " << \
    kit.objects.size() << " objects" << std::endl;

  // Count the number of each type of assigned object
  std::map<std::string, unsigned int> assignedObjectTypeCount, currentObjectTypeCount;
  for (const auto & obj : assignedKit.objects)
  {
    if (assignedObjectTypeCount.find(obj.type) == assignedObjectTypeCount.end())
    {
      assignedObjectTypeCount[obj.type] = 0;
    }
    assignedObjectTypeCount[obj.type] += 1;
  }

  gzdbg << "Checking object counts" << std::endl;

  bool assignedObjectsMissing = false;
  for (auto & value : assignedObjectTypeCount)
  {
    auto assignedObjectType = value.first;
    auto assignedObjectCount = value.second;
    auto currentObjectCount =
      std::count_if(kit.objects.begin(), kit.objects.end(),
        [assignedObjectType](ariac::KitObject k) {return k.type == assignedObjectType;});
    gzdbg << "Found " << currentObjectCount << \
      " objects of type '" << assignedObjectType << "'" << std::endl;
    score.partPresence +=
      std::min(long(assignedObjectCount), currentObjectCount) * scoringParameters.objectPresence;
    if (currentObjectCount < assignedObjectCount)
    {
      assignedObjectsMissing = true;
    }
  }
  if (!assignedObjectsMissing)
  {
    gzdbg << "All objects on tray" << std::endl;
    score.allPartsBonus += scoringParameters.allObjectsBonusFactor * numAssignedObjects;
  }

  gzdbg << "Checking object poses" << std::endl;
  // Keep track of which assigned objects have already been 'matched' to one on the tray.
  // This is to prevent multiple objects being close to a single target pose both scoring points.
  std::vector<ariac::KitObject> remainingAssignedObjects(assignedKit.objects);

  for (const auto & currentObject : kit.objects)
  {
    for (auto it = remainingAssignedObjects.begin(); it != remainingAssignedObjects.end(); ++it)
    {
      // Only check poses of parts of the same type
      auto assignedObject = *it;
      if (assignedObject.type != currentObject.type)
        continue;

      // Check the position of the object (ignoring orientation)
      gzdbg << "Comparing pose '" << currentObject.pose << \
        "' with the assigned pose '" << assignedObject.pose << "'" << std::endl;
      gazebo::math::Vector3 posnDiff = assignedObject.pose.CoordPositionSub(currentObject.pose);
      posnDiff.z = 0;
      if (posnDiff.GetLength() > scoringParameters.distanceThresh)
        continue;
      gzdbg << "Object of type '" << currentObject.type << \
        "' in the correct position" << std::endl;
      score.partPose += scoringParameters.objectPosition;

      // Check the orientation of the object.
      gazebo::math::Quaternion objOrientation = currentObject.pose.rot;
      gazebo::math::Quaternion orderOrientation = assignedObject.pose.rot;

      // Filter objects that aren't in the appropriate orientation (loosely).
      // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
      double orientationDiff = objOrientation.Dot(orderOrientation);
      // TODO: this value can probably be derived using relationships between
      // euler angles and quaternions.
      double quaternionDiffThresh = 0.05;
      if (std::abs(orientationDiff) < (1.0 - quaternionDiffThresh))
        continue;

      // Now filter the poses based on a threshold set in radians (more user-friendly).
      double yawDiff = objOrientation.GetYaw() - orderOrientation.GetYaw();
      if (std::abs(yawDiff) > scoringParameters.orientationThresh)
        continue;

      gzdbg << "Object of type '" << currentObject.type << \
        "' in the correct orientation" << std::endl;
      score.partPose += scoringParameters.objectOrientation;

      // Once a match is found, don't permit it to be matched again
      remainingAssignedObjects.erase(it);
      break;
    }
  }

  // Check if all assigned objects have been matched to one on the tray
  if (remainingAssignedObjects.empty())
  {
    score.isComplete = true;
  }

  return score;
}

/////////////////////////////////////////////////
void AriacScorer::OnTrayInfoReceived(const osrf_gear::KitTray::ConstPtr & trayMsg)
{
  boost::mutex::scoped_lock kitTraysLock(this->kitTraysMutex);

  // Get the ID of the tray that the message is from.
  std::string trayID = trayMsg->tray;

  if (this->kitTrays.find(trayID) == this->kitTrays.end())
  {
    // This is the first time we've heard from this tray: initialize it.
    this->kitTrays[trayID] = ariac::KitTray(trayID);
  }

  // Update the state of the tray.
  // TODO: this should be moved outside of the callback
  // Do this even if the tray isn't part of the current order because maybe it
  // will be part of future orders.
  this->newTrayInfoReceived = true;
  ariac::Kit kitState;
  FillKitFromMsg(trayMsg->kit, kitState);
  this->kitTrays[trayID].UpdateKitState(kitState);
}

/////////////////////////////////////////////////
void AriacScorer::OnOrderReceived(const osrf_gear::Order::ConstPtr & orderMsg)
{
  gzdbg << "Received an order" << std::endl;
  this->newOrderReceived = true;

  ariac::Order order;
  order.orderID = orderMsg->order_id;
  for (const auto & kitMsg : orderMsg->kits)
  {
    ariac::KitType_t kitType = kitMsg.kit_type;
    ariac::Kit assignedKit;
    FillKitFromMsg(kitMsg, assignedKit);
    order.kits[kitType] = assignedKit;
  }
  this->newOrder = order;
}

/////////////////////////////////////////////////
void AriacScorer::AssignOrder(const ariac::Order & order)
{
  ariac::OrderID_t orderID = order.orderID;
  if (this->gameScore.orderScores.find(orderID) == this->gameScore.orderScores.end())
  {
    // This is a previously unseen order: start scoring from scratch
    this->gameScore.orderScores[orderID] = ariac::OrderScore();
    this->gameScore.orderScores[orderID].orderID = orderID;
  }
  this->orderScore = &this->gameScore.orderScores[orderID];

  this->currentOrder = order;
  gzdbg << "Assigned order: " << this->currentOrder << std::endl;
}

/////////////////////////////////////////////////
ariac::OrderScore AriacScorer::UnassignCurrentOrder(double timeTaken)
{
  gzdbg << "Unassigning order: " << this->currentOrder.orderID << std::endl;
  auto orderScore = *this->orderScore;
  orderScore.timeTaken = timeTaken;
  this->currentOrder.kits.clear();
  return orderScore;
}

/////////////////////////////////////////////////
void AriacScorer::FillKitFromMsg(const osrf_gear::Kit &kitMsg, ariac::Kit &kit)
{
  kit.objects.clear();
  for (const auto & objMsg : kitMsg.objects)
  {
    ariac::KitObject obj;
    obj.type = ariac::DetermineModelType(objMsg.type);
    geometry_msgs::Point p = objMsg.pose.position;
    geometry_msgs::Quaternion o = objMsg.pose.orientation;
    gazebo::math::Vector3 objPosition(p.x, p.y, p.z);
    gazebo::math::Quaternion objOrientation(o.w, o.x, o.y, o.z);
    obj.pose = gazebo::math::Pose(objPosition, objOrientation);
    kit.objects.push_back(obj);
  }
}

/////////////////////////////////////////////////
void AriacScorer::OnGripperStateReceived(const osrf_gear::VacuumGripperState &stateMsg)
{
  this->isPartTravelling = stateMsg.enabled && stateMsg.attached;
}
