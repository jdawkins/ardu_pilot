#include "ardu_pilot/ardu_pilot.h"


/**
 * Takes a ROS-Mavlink-message (mavlink_ros_msg) and converts it into a Mavlink-Message (msg)
 */
static inline void createMavlinkFromROS(const mavlink_msgs::Mavlink *mavlink_ros_msg, mavlink_message_t *msg)
{

    msg->checksum = mavlink_ros_msg->checksum;
    msg->magic = mavlink_ros_msg->magic;
    msg->len = mavlink_ros_msg->len;
    msg->seq = mavlink_ros_msg->seq;
    msg->sysid = mavlink_ros_msg->sysid;
    msg->compid = mavlink_ros_msg->compid;
    msg->msgid = mavlink_ros_msg->msgid;

    for(int i = 0;i<BUFFER_LENGTH;i++){
        msg->payload64[i] = mavlink_ros_msg->payload[i];

    }

}

/**
 * Takes a Mavlink-message (mavlink_msg) and converts it into a ROS-Mavlink-Message (mavlink_ros_msg)
 */
static inline void createROSFromMavlink(const mavlink_message_t* mavlink_msg, mavlink_msgs::Mavlink* mavlink_ros_msg)
{

    mavlink_ros_msg->header.frame_id = "Frame1";
    mavlink_ros_msg->header.seq = mavlink_msg->seq;
    mavlink_ros_msg->checksum = mavlink_msg->checksum;
    mavlink_ros_msg->magic = mavlink_msg->magic;
    mavlink_ros_msg->len = mavlink_msg->len;
    mavlink_ros_msg->seq = mavlink_msg->seq;
    mavlink_ros_msg->sysid = mavlink_msg->sysid;
    mavlink_ros_msg->compid = mavlink_msg->compid;
    mavlink_ros_msg->msgid = mavlink_msg->msgid;
    //mavlink_ros_msg->payload_length = BUFFER_LENGTH;

    for (int i = 0; i < BUFFER_LENGTH; i++){
        mavlink_ros_msg->payload[i] = mavlink_msg->payload64[i];
    }

}

ArduPilot::ArduPilot(){

    getROSParameters();
    configureROSComms();
}

ArduPilot::ArduPilot(NodeHandle nh){
    n_ = nh;

    getROSParameters();
    configureROSComms();
}

ArduPilot::ArduPilot(const string port, uint32_t baud){

    my_serial_.setPort(port);
    my_serial_.setBaudrate(baud);
}

void
ArduPilot::connect(){

    cout<<"Attempting to Connect Ardupilot"<<endl;

    try {
        my_serial_.open();

    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open port "<< my_serial_.getPort() << "err: " << e.what() <<endl;
    }

    // cout<<"Port Status"<< my_serial.isOpen()<<endl;
    if(my_serial_.isOpen()){

        cout<<"Ardu_Pilot Connected on Port: "<<my_serial_.getPort()<<" at Baudrate: "<<my_serial_.getBaudrate()<<endl;
    }
    else{

        ROS_ERROR("Serial Port was Unable to Open, check port settings");
    }

  //  getROSParameters();

}

void
ArduPilot::configureROSComms(){

    mav_pub_ = n_.advertise<mavlink_msgs::Mavlink>("/mav_data",1000);
    gps_pub_ = n_.advertise<sensor_msgs::NavSatFix>("/gps_data",1000);
    imu_pub_ = n_.advertise<sensor_msgs::Imu>("/imu_data",1000);
    ahrs_pub_ = n_.advertise<sensor_msgs::Imu>("/ahrs_data",1000);
    att_pub_ = n_.advertise<sensor_msgs::Imu>("/att_data",1000);
    rc_pub_ = n_.advertise<mavlink_msgs::rc_channel>("/rc_data",1000);
    pose_pub_ = n_.advertise<geometry_msgs::Pose>("/pose_data",1000);

    mav_sub_ = n_.subscribe("/mav_qgc",1000,&ArduPilot::receiverCallBack,this);

}

void
ArduPilot::getROSParameters(){

    if(n_.getParam("port",port_)){
        ROS_INFO("Got Param: %s",port_.c_str());
    }
    else{

        port_ = "/dev/ttyACM0";
        ROS_WARN("Failed to get Serial Port from Launch File, defaulting to %s \n",port_.c_str());

    }

    if(n_.getParam("baud",baud_)){

        ROS_INFO("Got Param: %d",baud_);

    }
    else{
        baud_ = 115200;
        ROS_WARN("Failed to get Serial Baudrate from Launch File, defaulting to %d \n",baud_);

    }

    my_serial_.setPort(port_.c_str());
    my_serial_.setBaudrate(baud_);

}

void
ArduPilot::readData(){

    mavlink_msgs::Mavlink ros_mav_msg;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;


    while(my_serial_.available()>0){

        my_serial_.read(&byte,1);

        if(mavlink_parse_char(MAVLINK_COMM_0,byte,&msg,&status)){

            clock_gettime(CLOCK_MONOTONIC, &current_time_);
            createROSFromMavlink(&msg,&ros_mav_msg);
            ros_mav_msg.header.stamp.sec = current_time_.tv_sec;
            ros_mav_msg.header.stamp.nsec = current_time_.tv_nsec;

            mav_pub_.publish(ros_mav_msg);

            switch(msg.msgid){

            case MAVLINK_MSG_ID_HEARTBEAT:
            {

            }
                break;
            case MAVLINK_MSG_ID_RAW_IMU: // msg ID: 27
            {
                if(imu_pub_.getNumSubscribers()>0){
                    mavlink_raw_imu_t imu;
                    mavlink_msg_raw_imu_decode(&msg,&imu);

                    ros_imu_msg_.header.stamp.sec = current_time_.tv_sec;
                    ros_imu_msg_.header.stamp.nsec = current_time_.tv_nsec;
                    //ros_imu_msg_.header.stamp = ros::Time.now();
                    ros_imu_msg_.linear_acceleration.x = 9.81*(imu.xacc/1000);
                    ros_imu_msg_.linear_acceleration.y = 9.81*(imu.yacc/1000);
                    ros_imu_msg_.linear_acceleration.z = 9.81*(imu.zacc/1000);

                    ros_imu_msg_.angular_velocity.x = imu.xgyro/1000;
                    ros_imu_msg_.angular_velocity.y = imu.ygyro/1000;
                    ros_imu_msg_.angular_velocity.z = imu.zgyro/1000;


                    imu_pub_.publish(ros_imu_msg_);
                }
            }
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT: //msg ID: 24
            {
                if(gps_pub_.getNumSubscribers()>0){
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&msg,&gps);

                    ros_gps_msg_.header.stamp.sec = current_time_.tv_sec;
                    ros_gps_msg_.header.stamp.nsec = current_time_.tv_nsec;

                    ros_gps_msg_.latitude = gps.lat/1E7;
                    ros_gps_msg_.longitude = gps.lon/1E7;
                    ros_gps_msg_.altitude = gps.alt/1E3;
                    gps_pub_.publish(ros_gps_msg_);

                    gps_common::LLtoUTM(gps.lat,gps.lon, north_, east_, utm_zone_);

                    ros_pose_msg_.position.x = north_;
                    ros_pose_msg_.position.y = east_;
                    ros_pose_msg_.position.z = -gps.alt/1E7;

                }
            }
                break;
            case MAVLINK_MSG_ID_ATTITUDE: //msg ID: 30
            {
                if(pose_pub_.getNumSubscribers()>0){
                    mavlink_attitude_t att;

                    mavlink_msg_attitude_decode(&msg,&att);

                    ros_att_msg_.header.stamp.sec = current_time_.tv_sec;
                    ros_att_msg_.header.stamp.nsec = current_time_.tv_nsec;
                    ros_att_msg_.orientation.x = att.roll;
                    ros_att_msg_.orientation.y = att.pitch;
                    ros_att_msg_.orientation.z = att.yaw;
                    att_pub_.publish(ros_att_msg_);


                    ros_pose_msg_.orientation.x = att.roll;
                    ros_pose_msg_.orientation.y = att.pitch;
                    ros_pose_msg_.orientation.z = att.yaw;

                    pose_pub_.publish(ros_pose_msg_);
                }
            }
                break;
            case MAVLINK_MSG_ID_AHRS: //msg ID: 163
            {
                mavlink_ahrs_t ahrs;

            }
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW: //msg ID: 35
            {

                if(rc_pub_.getNumSubscribers()>0){
                mavlink_rc_channels_raw_t rc;
                ros_rc_msg_.time_boot_ms = rc.time_boot_ms;
                ros_rc_msg_.chan1_raw = rc.chan1_raw;
                ros_rc_msg_.chan2_raw = rc.chan2_raw;
                ros_rc_msg_.chan3_raw = rc.chan3_raw;
                ros_rc_msg_.chan4_raw = rc.chan4_raw;
                ros_rc_msg_.chan5_raw = rc.chan5_raw;
                ros_rc_msg_.chan6_raw = rc.chan6_raw;
                ros_rc_msg_.chan7_raw = rc.chan7_raw;
                ros_rc_msg_.chan8_raw = rc.chan8_raw;
                ros_rc_msg_.rssi = rc.rssi;

                rc_pub_.publish(ros_rc_msg_);
                }
            }
                break;
           /* case MAVLINK_MSG_ID_G: //msg ID: 28
            {
                mavlink_raw_pressure_t baro;

            }
                break;  */
            case MAVLINK_MSG_ID_RAW_PRESSURE: //msg ID: 28
            {
                mavlink_raw_pressure_t baro;

            }
                break;
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
ArduPilot::receiverCallBack(const mavlink_msgs::Mavlink &mav_msg){

    mavlink_message_t msg;
    msg.msgid = mav_msg.msgid;

    static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

    //Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
    copy(mav_msg.payload.begin(), mav_msg.payload.end(), msg.payload64);

    mavlink_finalize_message_chan(&msg, mav_msg.sysid, mav_msg.compid, MAVLINK_COMM_0,
                                  mav_msg.len, mavlink_crcs[msg.msgid]);

    static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int messageLength = mavlink_msg_to_send_buffer(buffer, &msg);
    int written = my_serial_.write(buffer,messageLength);
    my_serial_.flushOutput();

 //   if (messageLength != written)
 //     fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength);

}
