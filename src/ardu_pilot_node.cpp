#include "ardu_pilot/ardu_pilot.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "ardu_pilot_node");

    string port = "/dev/ttyACM0";
    int baud = 115200;
    ardu_pilot ardu(port,baud);


    ardu.connect();

    ros::Rate loop_rate(100);
    while(ros::ok()){

        ardu.readData(); // Read new Data from ArduPilot and Write to ROS

        ros::spinOnce();// Allow ROS to check for new ROS Messages and Send to ArduPilot
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate
    }

}
