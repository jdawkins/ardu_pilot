#include "ardu_pilot/ardu_pilot.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "ardu_pilot_node");

    ros::NodeHandle nh = ros::NodeHandle("~");

    ArduPilot* ardu = new ArduPilot(nh);

    ardu->connect();

    ros::Rate loop_rate(1000);
    while(ros::ok()){

        ardu->readData(); // Read new Data from ArduPilot and Write to ROS

        ros::spinOnce();// Allow ROS to check for new ROS Messages and Send to ArduPilot
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate
    }

}
