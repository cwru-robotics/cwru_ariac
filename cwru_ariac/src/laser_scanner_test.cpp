// RN
// Laser Scanner Integration
// 15/5/17

#include <LaserScanner.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "laser_scanner_test");
    ros::NodeHandle nh; 

    LaserScanner LaserScannerTest(nh);

    ROS_INFO("laser_scanner_test..");

    while(ros::ok()){
	LaserScannerTest.ForceUpdate();
	// LaserScannerTest.publish_part();
    
    }
    return 0; 
    
}

