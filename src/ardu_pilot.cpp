#include "ardu_pilot/ardu_pilot.h"


ardu_pilot::ardu_pilot(){

    getROSParameters();

}

ardu_pilot::ardu_pilot(const string port, uint32_t baud){

    my_serial.setPort(port);
    my_serial.setBaudrate(baud);

}

void
ardu_pilot::connect(){

    cout<<"attempting to open serial port"<<endl;

 //           my_serial.open();
    try {
        my_serial.open();

    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open port "<< my_serial.getPort() << "err: " << e.what() <<endl;
    }

    if(my_serial.isOpen()){

        cout<<"Ardu_Pilot Connected on Port: "<<my_serial.getPort()<<" at Baudrate: "<<my_serial.getBaudrate()<<endl;
    }

    getROSParameters();
    configureROSComms();

}

void
ardu_pilot::configureROSComms(){

    gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_data",100);
    imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",100);
    ahrs_pub = n.advertise<geometry_msgs::Vector3>("ahrs_data",100);
    att_pub = n.advertise<geometry_msgs::Vector3>("att_data",100);

}

void
ardu_pilot::getROSParameters(){

    n.param<std::string>("port", port_, "/dev/ttyACM0");
    n.param<int>("baudrate", baud_, 115200);
}

void
ardu_pilot::run(){

    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf;

    my_serial.read(&buf,1);

    if(mavlink_parse_char(MAVLINK_COMM_0,buf,&msg,&status)){

        switch(msg.msgid){

                case MAVLINK_MSG_ID_HEARTBEAT:
                {

                }
                break;
                case MAVLINK_MSG_ID_RAW_IMU:
                {
                    mavlink_raw_imu_t imu;
                    mavlink_msg_raw_imu_decode(&msg,&imu);

                    ros_imu_msg.linear_acceleration.x = 9.81*(imu.xacc/1000);
                    ros_imu_msg.linear_acceleration.y = 9.81*(imu.yacc/1000);
                    ros_imu_msg.linear_acceleration.z = 9.81*(imu.zacc/1000);

                    ros_imu_msg.angular_velocity.x = imu.xgyro/1000;
                    ros_imu_msg.angular_velocity.y = imu.ygyro/1000;
                    ros_imu_msg.angular_velocity.z = imu.zgyro/1000;


                    imu_pub.publish(ros_imu_msg);
                }
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&msg,&gps);

                    ros_gps_msg.latitude = gps.lat/1E7;
                    ros_gps_msg.longitude = gps.lon/1E7;
                    ros_gps_msg.altitude = gps.alt/1E3;

                    gps_pub.publish(ros_gps_msg);
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    mavlink_attitude_t att;

                    mavlink_msg_attitude_decode(&msg,&att);

                    ros_att_msg.x = att.roll;
                    ros_att_msg.y = att.pitch;
                    ros_att_msg.z = att.yaw;

                    att_pub.publish(ros_att_msg);
                }
                case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                {
                    mavlink_rc_channels_raw_t rc;

                }

                case MAVLINK_MSG_ID_RAW_PRESSURE:
                {
                    mavlink_raw_pressure_t baro;


                }

                case MAVLINK_MSG_ID_COMMAND_LONG:
                // EXECUTE ACTION
                break;

                default:
                //Do nothing
                break;
        }

    }

    my_serial.flush();

}
