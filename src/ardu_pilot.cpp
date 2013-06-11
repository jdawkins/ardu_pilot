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

    for(int i = 0;i<BUFFER_LENGTH;i++){
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

    for (int i = 0; i < BUFFER_LENGTH; i++){

       (mavlink_ros_msg->payload).push_back(mavlink_msg->payload64[i]);
    }

}

ArduPilot::ArduPilot(){
    log_imu_flag_ = false;
    log_gps_flag_ = false;
    log_ahrs_flag_ = false;
    log_att_flag_ = false;

    getROSParameters();
    configureROSComms();
}

ArduPilot::ArduPilot(NodeHandle nh){
    n_ = nh;

    log_imu_flag_ = false;
    log_gps_flag_ = false;
    log_ahrs_flag_ = false;
    log_att_flag_ = false;
    getROSParameters();
    configureROSComms();

}

ArduPilot::ArduPilot(const string port, uint32_t baud){

    log_imu_flag_ = false;
    log_gps_flag_ = false;
    log_ahrs_flag_ = false;
    log_att_flag_ = false;

    my_serial_.setPort(port);
    my_serial_.setBaudrate(baud);

}

void
ArduPilot::connect(){

    cout<<"attempting to Connect"<<endl;

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

    getROSParameters();


}

void
ArduPilot::configureROSComms(){

    mav_pub_ = n_.advertise<Mavlink>("/mav_data",1000);
    gps_pub_ = n_.advertise<sensor_msgs::NavSatFix>("/gps_data",1000);
    imu_pub_ = n_.advertise<sensor_msgs::Imu>("/imu_data",1000);
    ahrs_pub_ = n_.advertise<geometry_msgs::Vector3>("/ahrs_data",1000);
    att_pub_ = n_.advertise<geometry_msgs::Vector3>("/att_data",1000);

    mav_sub_ = n_.subscribe("/mav_qgc",1000,&ArduPilot::receiverCallBack,this);

}

void
ArduPilot::getROSParameters(){

    n_.getParam("log_imu",log_imu_flag_);
    n_.getParam("log_gps",log_gps_flag_);
    n_.getParam("log_att",log_att_flag_);
    n_.getParam("log_ahrs",log_ahrs_flag_);

    ROS_INFO("log imu stats: %d\n",log_imu_flag_);
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
    mavlink_msgs::Mavlink ros_mav_msg;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;


        while(my_serial_.available()>0){

             my_serial_.read(&byte,1);

            if(mavlink_parse_char(MAVLINK_COMM_0,byte,&msg,&status)){


                createROSFromMavlink(&msg,&ros_mav_msg);

                mav_pub_.publish(ros_mav_msg);

                switch(msg.msgid){

                        case MAVLINK_MSG_ID_HEARTBEAT:
                        {

                        }
                        break;
                        case MAVLINK_MSG_ID_RAW_IMU:
                        {
                            if(log_imu_flag_){
                            mavlink_raw_imu_t imu;
                            mavlink_msg_raw_imu_decode(&msg,&imu);

                            ros_imu_msg_.linear_acceleration.x = 9.81*(imu.xacc/1000);
                            ros_imu_msg_.linear_acceleration.y = 9.81*(imu.yacc/1000);
                            ros_imu_msg_.linear_acceleration.z = 9.81*(imu.zacc/1000);

                            ros_imu_msg_.angular_velocity.x = imu.xgyro/1000;
                            ros_imu_msg_.angular_velocity.y = imu.ygyro/1000;
                            ros_imu_msg_.angular_velocity.z = imu.zgyro/1000;


                            imu_pub_.publish(ros_imu_msg_);
                            }
                        }
                        case MAVLINK_MSG_ID_GPS_RAW_INT:
                        {
                            if(log_gps_flag_){
                                mavlink_gps_raw_int_t gps;
                                mavlink_msg_gps_raw_int_decode(&msg,&gps);

                                ros_gps_msg_.latitude = gps.lat/1E7;
                                ros_gps_msg_.longitude = gps.lon/1E7;
                                ros_gps_msg_.altitude = gps.alt/1E3;

                                gps_pub_.publish(ros_gps_msg_);
                            }
                        }

                        case MAVLINK_MSG_ID_ATTITUDE:
                        {
                            if(log_att_flag_){
                            mavlink_attitude_t att;

                            mavlink_msg_attitude_decode(&msg,&att);

                            ros_att_msg_.x = att.roll;
                            ros_att_msg_.y = att.pitch;
                            ros_att_msg_.z = att.yaw;

                            att_pub_.publish(ros_att_msg_);
                            }
                        }
                        case MAVLINK_MSG_ID_AHRS:
                        {
                            mavlink_ahrs_t ahrs;

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
ArduPilot::receiverCallBack(const Mavlink &mav_msg){

    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t mavlink_msg;

    createMavlinkFromROS(&mav_msg, &mavlink_msg);
    mavlink_msg_to_send_buffer(buf,&mavlink_msg);

    /*printf("Writing Byte Stream: ");
    for(int i=0;i<sizeof(buf);i++){

        uint8_t temp = buf[i];
        printf("%02x ", (unsigned char)temp);
    }
    printf("\n");*/

    my_serial_.write(buf,sizeof(buf));

}
