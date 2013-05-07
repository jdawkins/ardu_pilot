#include "ardu_pilot/ardu_pilot.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "ardu_pilot_node");

    string port = "/dev/ttyACM0";
    int baud = 115200;
    ardu_pilot ardu(port,baud);

    ardu.connect();

    while(ros::ok()){

        ardu.run();

    }

}
