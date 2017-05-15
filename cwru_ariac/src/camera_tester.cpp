//
// Created by shipei on 11/1/16.
//

#include <CameraEstimator.h>

int main(int argc, char** argv) {
    int index;
    if (argv[1] != NULL) {
        index = atoi(argv[1]);
    } else {
        cout << "Usage: rosrun cwru_ariac camera_tester [part_id] # the id of the part you want to keep tracking (positive integer)" << endl;
    }
    ros::init(argc, argv, "camera_tester");
    ros::NodeHandle nh;
    CameraEstimator camera(nh, "/ariac/logical_camera_1");
    while (ros::ok()) {
        camera.forceUpdate();
        auto part = findPart(camera.inView, index);
        if (part != camera.inView.end()) {
            cout << "Id: " << part->id << endl
                 << "Name: " << part->name << endl
                 << "Traceable: " << (part->traceable?"yes":"no")
                 << "Location: " << part->location << endl;
            ROS_INFO_STREAM("Pose:\n" << part->pose);
            ROS_INFO_STREAM("Linear:\n" << part->linear);
        } else {
            cout << "Id: " << atoi(argv[1]) << " is out of view" << endl;
        }
        cout << "inView size: " << camera.inView.size() << ", onConveyor size: " << camera.onConveyor.size() << endl;
        for (int i = 0; i < camera.onBin.size(); ++i) {
            cout << "onBin " << i + 1 << ": " << camera.onBin[i].size() << endl;
        }
        for (int i = 0; i < camera.onAGV.size(); ++i) {
            cout << "onAGV " << i + 1 << ": " << camera.onAGV[i].size() << endl;
        }
        cout << "onGround : " << camera.onGround.size() << endl;
    }
}
