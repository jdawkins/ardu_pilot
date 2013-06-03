#include "ardu_pilot/ardu_pilot.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "ardu_pilot_listener");

    string port = "/dev/ttyACM0";
    int baud = 115200;
    ardu_pilot ardu;

    ros::NodeHandle n;

     ros::Subscriber  tsub = n.subscribe("mav_data",1000,&ardu_pilot::receiverCallBack,&ardu);

    ros::spin();

   /* ros::Rate loop_rate(100);
    while(ros::ok()){

        ardu.run();

        loop_rate.sleep();
    }*/

}
