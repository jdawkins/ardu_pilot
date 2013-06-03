#ifndef ARDU_PILOT_H
#define ARDU_PILOT_H

#define MAX_BUFFER_SIZE 100

#include "serial/serial.h"
#include <string.h>
#include <iostream>
#include "mavlink/v1.0/ardupilotmega/mavlink.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavlink_msgs/Mavlink.h"


#include "ros/ros.h"

using namespace serial;
using namespace std;
using namespace boost;
using namespace ros;
using namespace mav_msgs;


// MAVLINK MESSAGE ID 24
/*struct gps_data{

    uint64_t time;      //micro seconds from epoch
    uint8_t fix_type;   //0-1: no fix, 2: 2D fix, 3: 3D fix.
    int32_t latitude;   //1E7 degrees
    int32_t longitude;  //1E7 degrees
    int32_t altitude;   //1E3 meters
    uint16_t hdop;      //HDOP horizontal dilution of position in cm (m*100)
    uint16_t vdop;      //VDOP horizontal dilution of position in cm (m*100)
    uint16_t speed;     //GPS ground speed (m/s * 100)
    uint16_t course;    //in degrees * 100, 0.0..359.99 degrees
    uint8_t num_sats;   // # of satelite visible
};

// MAVLINK MESSAGE ID 27 "RAW_IMU"
struct raw_imu_data{
    uint64_t time_usec; // microseconds since system boot
    int16_t accel_x;    // x acceleration m/s^2
    int16_t accel_y;    // y acceleration m/s^2
    int16_t accel_z;    // z acceleration m/s^2
    int16_t gyro_x;     // rotation rate about x
    int16_t gyro_y;     // rotation rate about y
    int16_t gyro_z;     // rotation rate about z
    int16_t mag_x;      // x magnetic field (raw)
    int16_t mag_y;      // y magnetic field (raw)
    int16_t mag_z;      // z magnetic field (raw)
};

//MAVLINK MESSAGE ID 30 Z-down coordinate frame
struct attitude{
    uint32_t time_boot_ms; // time since boot in ms
    float roll;         // radians
    float pitch;        // radians
    float yaw;          // radians
    float roll_rate;    // rad/s
    float pitch_rate;   // rad/s
    float yaw_rate;     // rad/s
};

// MAVLINK MESSAGE ID 33
struct pos_estimate{

    uint64_t time_boot_ms;      // time since boot in ms
    int32_t latitude;   //1E7 degrees
    int32_t longitude;  //1E7 degrees
    int32_t altitude;   //1E3 meters
    int32_t rel_altitude; // Altitdue measured from ground in mm (m*1000)
    int16_t x_vel;
    int16_t y_vel;
    int16_t z_vel;
    uint16_t heading;    //in degrees * 100, 0.0..359.99 degrees
};

struct ahrs_data{

    float roll;
    float pitch;
    float yaw;
};


// MAVLINK MESSAGE ID 29
struct barometer{

    uint32_t time_boot_ms;
    float pressure_abs;
    float pressure_rel;
    int16_t temperature;

};*/

class ardu_pilot {

public:
    ardu_pilot();
    ardu_pilot(const string port, uint32_t baud);

    void connect();
    void run();
    void readData();


    sensor_msgs::Imu ros_imu_msg;
    sensor_msgs::NavSatFix ros_gps_msg;
    sensor_msgs::NavSatStatus ros_gps_fix_msg;
    geometry_msgs::Vector3 ros_ahrs_msg;
    geometry_msgs::Vector3 ros_att_msg;





    void getROSParameters();
    void configureROSComms();
    void receiverCallBack(const Mavlink &mav_msg);
   // ~ardu_pilot();
private:


    void parseData();


    string port_;
    int baud_;
    string data_buffer;
    Serial my_serial;

    mavlink_status_t last_status;


    ros::NodeHandle n;
    ros::Publisher imu_pub;
    ros::Publisher gps_pub;
    ros::Publisher ahrs_pub;
    ros::Publisher att_pub;
    ros::Publisher mav_pub;
    ros::Subscriber mav_sub;

};


#endif
