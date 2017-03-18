// wsn, 10/15/17 based on CameraEstimator.cpp
// Created by shipei on 10/19/16.
//

#include <cwru_ariac/Part.h>
#include "Agv1CameraEstimator.h"

string kit_tray_string("kit_tray");

Agv1CameraEstimator::Agv1CameraEstimator(ros::NodeHandle nodeHandle, string topic) :
        nh_(nodeHandle), onAGV(totalAGVs), onBin(totalBins) {
    cameraSubscriber = nh_.subscribe(topic, 1,
                                     &Agv1CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.02;
    untraceableTolerance = 0.1;
    assignedID = 1;
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "/world";
    cameraFrame = topic.substr(topic.find_last_of("/")) + "_frame";
    //tf_listener.waitForTransform(cameraFrame, worldFrame, ros::Time(0), ros::Duration(2.0));
}

void Agv1CameraEstimator::cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
    PartSet updateView;
    ros::Time currentTime = ros::Time::now();
    double dt = currentTime.toSec() - lastTime.toSec();
    // update all exist objects
    for (auto part: inView) {
        part.pose.pose.position.x += part.linear.x * dt;
        part.pose.pose.position.y += part.linear.y * dt;
        part.pose.pose.position.z += part.linear.z * dt;
        part.pose.header.stamp = ros::Time::now();
    }
    lastTime.fromSec(currentTime.toSec());

    auto models = image_msg->models;
//    vector<double> distances;
//    for (int j = 0; j < models.size(); ++j) {
//        if (defaultParts.find(models[j].type) == defaultParts.end()) {
//            models.erase(models.begin() + j);
//            continue;
//        }
//        for (int i = j + 1; i < models.size(); ++i) {
//            double distance = euclideanDistance(models[j].pose.position, models[i].pose.position);
//            distances.push_back(distance);
//            if (distance < 0.02) {
//                models.erase(models.begin() + i);
//            }
//        }
//    }
//    sort(distances.begin(), distances.end());
    // TODO refactor to: match all exist object first, then all untraceable object then add new object
//    ROS_INFO("inView: %d, size: %d, dt = %f",(int)inView.size(), (int)image_msg->models.size(), (float)dt);
    if(updateCount == checkedCount) {
       ROS_INFO("AGV1 camera sees %d objects",(int) models.size());
    }
    for (int i = 0; i < models.size(); ++i) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = cameraFrame;
        bool flag = false;
        bool tferr = true;
        double distance;
        Part nextPart;

        nextPart.name = models[i].type;
        string part_string(nextPart.name);
        if (kit_tray_string.compare(part_string)==0)
          continue; // skip this part if it is the kit tray
        inPose.pose = models[i].pose;
        while (tferr && ros::ok()) {
            tferr = false;
            try {
                inPose.header.stamp = ros::Time(0);
                tf_listener.transformPose(worldFrame, inPose, outPose);
            } catch (tf::TransformException &exception) {
//                return;
                ROS_WARN("%s", exception.what());
                tferr = true;
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
        }
        nextPart.pose = outPose;
        if(updateCount == checkedCount) {
          //ROS_INFO("evaluating part:");
          //ROS_INFO_STREAM(nextPart);
         }
        // find object by estimated distance;
        for (auto part:inView) {
            distance = euclideanDistance(nextPart.pose.pose.position, part.pose.pose.position);
            if ((distance < distanceTolerance) && (nextPart.name == part.name)) {
                nextPart.linear.x = (nextPart.linear.x + (nextPart.pose.pose.position.x - part.pose.pose.position.x)/dt)/2;
                nextPart.linear.y = (nextPart.linear.y + (nextPart.pose.pose.position.y - part.pose.pose.position.y)/dt)/2;
                nextPart.linear.z = (nextPart.linear.z + (nextPart.pose.pose.position.z - part.pose.pose.position.z)/dt)/2;
                nextPart.traceable = true;
                nextPart.id = part.id;
                inView.erase(part);
                flag = true;
                break;
            }
        }
        // try to find untraceable object by nearest distance
        if (!flag) {
            for (auto part:inView) {
                distance = euclideanDistance(nextPart.pose.pose.position, part.pose.pose.position);
                if ((!part.traceable) && (distance < untraceableTolerance) && (nextPart.name == part.name)) {
                    if ((distance < untraceableTolerance) && (nextPart.name == part.name)) {
                        nextPart.linear.x = (nextPart.pose.pose.position.x - part.pose.pose.position.x)/dt;
                        nextPart.linear.y = (nextPart.pose.pose.position.y - part.pose.pose.position.y)/dt;
                        nextPart.linear.z = (nextPart.pose.pose.position.z - part.pose.pose.position.z)/dt;
                        nextPart.traceable = true;
                        nextPart.id = part.id;
                        inView.erase(part);
                        flag = true;
                        break;
                    }
                }
            }
        }
        // make new object untraceable
        if (!flag) {
//            ROS_INFO("add part %d : %s", assignedID, nextPart.name.c_str());
            nextPart.linear.x = 0;
            nextPart.linear.y = 0;
            nextPart.linear.z = 0;
            nextPart.traceable = false;
            nextPart.id = assignedID++;
        }
        updateView.insert(nextPart);
    }
//    for (auto part: inView) {
//        ROS_INFO("part %d : %s disappeared", part.id, part.name.c_str());
//    }
    inView.swap(updateView);
    splitLocation();
    updateCount++;
}

//only test for parts within view on conveyor or parts on AGV1:
void Agv1CameraEstimator::splitLocation() {
    onConveyor.clear(); //consider parts in view on conveyor
    onAGV[0].clear(); //only consider AGV1, not AGV2


    for (auto part: inView) {
        if (checkBound(part.pose.pose.position, conveyorBoundBox)) {
            part.location = Part::CONVEYOR;
            onConveyor.push_back(part); //assign this part to conveyor
            continue; //done w/ this part; examine next part
        }
        //part was not on conveyor, so test if it is on agv1
        if(updateCount == checkedCount) {
          //ROS_INFO("checking if part on agv:");
          //ROS_INFO_STREAM(part.pose.pose.position);
         }
        if (checkBound(part.pose.pose.position, agvBoundBox[0])) {
         if(updateCount == checkedCount) {
          //ROS_INFO("part is on AGV1:");
         }                
                part.location = Part::AGV1;
                onAGV[0].push_back(part); //assign this part to AGV1
        }
        else {
         if(updateCount == checkedCount) {
          //ROS_INFO("part is not on AGV1:");
         }  
        }
    }
}

void Agv1CameraEstimator::waitForUpdate() {
    checkedCount = updateCount;
    while (updateCount == checkedCount && ros::ok()){
        //ROS_INFO("Waiting for camera callback...");
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
}

//given a vector of expected parts/locattions on agv tray, 
bool Agv1CameraEstimator::find_dropped_part(vector <osrf_gear::KitObject> partialKit, PartList onAGV1,Part &droppedPart) {
 //try to find matches between parts viewed by camera on tray vs intended parts on tray
 vector<int> camera_index_of_kit_index;
 vector<int> kit_index_of_camera_index;
 osrf_gear::KitObject kitPart;
 int nparts_camera = onAGV1.size();
 int nparts_kit = partialKit.size();
 //build mappings from kit list to camera list, and vice versa; index value= -1 if no match
 for (int i=0;i<nparts_kit;i++) {
   camera_index_of_kit_index.push_back(-1);
 }
 for (int i=0;i<nparts_camera;i++) {
   kit_index_of_camera_index.push_back(-1);
 }
 //walk through the kit parts list:
 for (int i=0;i<nparts_kit;i++) {
   kitPart = partialKit[i];
   string kit_part_name(kitPart.type);
   geometry_msgs::Pose kit_part_pose, camera_part_pose;
   kit_part_pose = kitPart.pose;
   Part camera_part;
   for (int j=0;j<nparts_camera;j++) {

    //logic: should have kit object not yet assigned;
    //camera object not yet assigned
    //kit and camera object names match
    //kit and camera object coords are within tolerance of each other
    if(camera_index_of_kit_index[i]== -1) {  //should halt the for:i loop
     camera_part = onAGV1[j];
     camera_part_pose = camera_part.pose.pose;
     string camera_part_name(camera_part.name);

     if (camera_part_name.compare(kit_part_name)==0) {
      ROS_INFO("part names match; but check if camera object already assigned");
      if (kit_index_of_camera_index[j]== -1) {
        ROS_INFO("and this camera object is not yet paired");
             if (match_poses_on_tray(camera_part_pose,kit_part_pose)) {
               ROS_INFO("poses match within tolerance");
               ROS_INFO("pairing kit object %d with camera object %d",i,j);
               camera_index_of_kit_index[i] = j;
               kit_index_of_camera_index[j] = i;
              }
              else {
               ROS_INFO("part coords not within tolerance");
              }
       }
     }
     
    }

   }
 }
  //now find which element of camera list is "orphaned"
  for (int j=0;j<nparts_camera;j++) {
    if (kit_index_of_camera_index[j]== -1) {
      ROS_INFO("element %d of camera part list is an orphan",j);
      droppedPart = onAGV1[j];
      return true; }
   }
 ROS_WARN("did not find orphaned part on camera list");
 return false; //if found dropped part
}

bool Agv1CameraEstimator::match_poses_on_tray(geometry_msgs::Pose camera_part_pose,geometry_msgs::Pose kit_part_pose) {
     //     tray_tol_x=0.01;  //TUNE THESE
     //   tray_tol_y = 0.01;
     //   tray_tol_quat = 0.05;
  double x_cam_part = camera_part_pose.position.x;
  double x_kit_part = kit_part_pose.position.x;
  double y_cam_part = camera_part_pose.position.y;
  double y_kit_part = kit_part_pose.position.y;
  ROS_INFO("x_cam_part = %f; x_kit_part = %f",x_cam_part,x_kit_part);
  double x_err = fabs(x_cam_part-x_kit_part);
  ROS_INFO("x_err = %f",x_err);
  if (x_err>tray_tol_x) { 
     ROS_INFO("out of tolerance");
     return false;
  }
  ROS_INFO("y_cam_part = %f; y_kit_part = %f",y_cam_part,y_kit_part);
  double y_err = fabs(y_cam_part-y_kit_part);
  ROS_INFO("y_err = %f",y_err);
  if (y_err>tray_tol_y) {
    ROS_INFO("out of tolerance");
    return false;  
  }
  double quat_err;
  //TODO: repeat test for quaternion error
  ROS_INFO("poses match within tolerance");
  return true;
}
