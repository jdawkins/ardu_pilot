#include "ardu_pilot/ardu_pilot.h"


/**
 * Takes a ROS-Mavlink-message (mavlink_ros_msg) and converts it into a Mavlink-Message (msg)
 */
static inline void createMavlinkFromROS(const Mavlink *mavlink_ros_msg, mavlink_message_t *msg)
{

    msg->checksum = mavlink_ros_msg->checksum;
    msg->magic = mavlink_ros_msg->magic;
    msg->len = mavlink_ros_msg->len;
    msg->seq = mavlink_ros_msg->seq;
    msg->sysid = mavlink_ros_msg->sysid;
    msg->compid = mavlink_ros_msg->compid;
    msg->msgid = mavlink_ros_msg->msgid;

    for(int i = 0;i<((MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8);i++){
        msg->payload64[i] = mavlink_ros_msg->payload[i];

    }

}

/**
 * Takes a Mavlink-message (mavlink_msg) and converts it into a ROS-Mavlink-Message (mavlink_ros_msg)
 */
static inline void createROSFromMavlink(const mavlink_message_t* mavlink_msg, Mavlink* mavlink_ros_msg)
{
    mavlink_ros_msg->checksum = mavlink_msg->checksum;
    mavlink_ros_msg->magic = mavlink_msg->magic;
    mavlink_ros_msg->len = mavlink_msg->len;
    mavlink_ros_msg->seq = mavlink_msg->seq;
    mavlink_ros_msg->sysid = mavlink_msg->sysid;
    mavlink_ros_msg->compid = mavlink_msg->compid;
    mavlink_ros_msg->msgid = mavlink_msg->msgid;

    for (int i = 0; i < ((MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8); i++){

       (mavlink_ros_msg->payload).push_back(mavlink_msg->payload64[i]);
    }

}

ardu_pilot::ardu_pilot(){

    getROSParameters();

}

ardu_pilot::ardu_pilot(const string port, uint32_t baud){

    my_serial.setPort(port);
    my_serial.setBaudrate(baud);

}

void
ardu_pilot::connect(){

    cout<<"attempting to Connect"<<endl;

    try {
        my_serial.open();

    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open port "<< my_serial.getPort() << "err: " << e.what() <<endl;
    }

   // cout<<"Port Status"<< my_serial.isOpen()<<endl;
    if(my_serial.isOpen()){

        cout<<"Ardu_Pilot Connected on Port: "<<my_serial.getPort()<<" at Baudrate: "<<my_serial.getBaudrate()<<endl;
    }

    getROSParameters();
    configureROSComms();

}

void
ardu_pilot::configureROSComms(){

    mav_pub = n.advertise<Mavlink>("mav_data",1000);
  //  gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_data",100);
   // imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",100);
   // ahrs_pub = n.advertise<geometry_msgs::Vector3>("ahrs_data",100);
   // att_pub = n.advertise<geometry_msgs::Vector3>("att_data",100);

    mav_sub = n.subscribe("/mav_qgc",1000,&ardu_pilot::receiverCallBack,this);

}

void
ardu_pilot::getROSParameters(){

    n.param<std::string>("port", port_, "/dev/ttyACM0");
    n.param<int>("baudrate", baud_, 115200);
}

void
ardu_pilot::readData(){
/*
    * mavlink_message_t msg;
    * int chan = 0;
    *
    *
    * while(serial.bytesAvailable > 0)
    * {
    *   uint8_t byte = serial.getNextByte();
    *   if (mavlink_parse_char(chan, byte, &msg))
    *     {
    *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
    *     }
    * }*/
    mav_msgs::Mavlink ros_mav_msg;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;


        while(my_serial.available()>0){

             my_serial.read(&byte,1);

            if(mavlink_parse_char(MAVLINK_COMM_0,byte,&msg,&status)){


                createROSFromMavlink(&msg,&ros_mav_msg);

                mav_pub.publish(ros_mav_msg);

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


                        //    imu_pub.publish(ros_imu_msg);
                        }
                        case MAVLINK_MSG_ID_GPS_RAW_INT:
                        {
                            mavlink_gps_raw_int_t gps;
                            mavlink_msg_gps_raw_int_decode(&msg,&gps);

                            ros_gps_msg.latitude = gps.lat/1E7;
                            ros_gps_msg.longitude = gps.lon/1E7;
                            ros_gps_msg.altitude = gps.alt/1E3;

                         //   gps_pub.publish(ros_gps_msg);
                        }

                        case MAVLINK_MSG_ID_ATTITUDE:
                        {
                            mavlink_attitude_t att;

                            mavlink_msg_attitude_decode(&msg,&att);

                            ros_att_msg.x = att.roll;
                            ros_att_msg.y = att.pitch;
                            ros_att_msg.z = att.yaw;

                        //    att_pub.publish(ros_att_msg);

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
        }
//    my_serial.flush();

}
void
ardu_pilot::receiverCallBack(const Mavlink &mav_msg){

    uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
    mavlink_message_t mavlink_msg;

    createMavlinkFromROS(&mav_msg, &mavlink_msg);
    mavlink_msg_to_send_buffer(buf,&mavlink_msg);

    printf("Writing Byte Stream: ");
    for(int i=0;i<sizeof(buf);i++){

        uint8_t temp = buf[i];
        printf("%02x ", (unsigned char)temp);
    }
    printf("\n");

    my_serial.write(buf,sizeof(buf));

   /* if (mavlink_msg.msgid == MAVLINK_MSG_ID_ATTITUDE){

                mavlink_attitude_t att;

                mavlink_msg_attitude_decode(&mavlink_msg,&att);

                ros_att_msg.x = att.roll;
                ros_att_msg.y = att.pitch;
                ros_att_msg.z = att.yaw;
                printf("Roll angle is : %f\n\r", att.roll);
    }*/
}
