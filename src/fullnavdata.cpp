/** TODO MIT **/

#include "fullnavdata.h"

#include <vector>
#include <utility>
#include <boost/log/trivial.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "gnuplot_iostream.h"

using boost::asio::ip::udp;

fullnavdata::fullnavdata()
    :_spinlock(ATOMIC_FLAG_INIT)
{
}

void fullnavdata::init(std::string ip, int senderPort)
{
    std::cout << "INIT" << std::endl;
    _navdata_socket.reset(new udp::socket(_ioService, udp::v4()));
    _navdata_socket->bind(udp::endpoint(udp::v4(), senderPort));
    std::cout << "bind ok" << std::endl;


    udp::resolver resolver(_ioService);
    udp::resolver::query query(ip, std::to_string(FULL_NAVDATA_DEFAULT_PORT));

    std::cout << "resolving " << query.host_name() << ":" << query.service_name() << std::endl;

    endpoint = *resolver.resolve(query);
    //lol
    std::cout << "sending to " << endpoint.address() << ":" << endpoint.protocol().protocol() << std::endl;

    size_t sent = _navdata_socket->send_to(boost::asio::buffer("hello"), endpoint);

    std::cout << sent << " bytes sent" << std::endl;

    // Start receiving beautiful, full, informative (and not dumbed-down by Parrot) navdata ;)
    std::cout << "ASYNC WAITING" << std::endl;
    _navdata_socket->async_receive_from(
            boost::asio::buffer(_navdata_buf),
            _navdata_sender_endpoint,
            boost::bind(&fullnavdata::navdataPacketReceived,
                        this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred
            )
    );
    /*
    while(1) {
        size_t read = _navdata_socket->receive_from(
                boost::asio::buffer(_navdata_buf, FULL_NAVDATA_MAX_SIZE),
                _navdata_sender_endpoint
        );
        navdataPacketReceived(boost::system::error_code(), read);
    }
     */
    std::cout << _navdata_sender_endpoint << std::endl;

    //_g->showonscreen();
}


void fullnavdata::navdataPacketReceived(const boost::system::error_code &error, size_t bytes_transferred)
{
    while(_spinlock.test_and_set(std::memory_order_acquire));

    std::cout <<"doot"<< std::endl;
    _gotNavdata = true;
    if(bytes_transferred < 1000)
    {
        return;
    }

    /*
    double pitch;
    double roll;
    double yaw;
    double height;
    double height_ultrasonic;
    double pressure;
    double vbat;
    double latitude;
    double longitude;
    double gps_sats;
    double gps_altitude;
    double bat_percent;
    double speed_x;
    double speed_y;
    double speed_z;
    double gps_accuracy;
    double gps_num_svs;
    double gps_eph, gps_epv;
    */

    //warning: data type size must not be over 8 bytes
    double sensor_acc_raw_x_m_s2;
    double sensor_acc_raw_y_m_s2;
    double sensor_acc_raw_z_m_s2;

    double sensor_gyro_raw_x_rad_s;
    double sensor_gyro_raw_y_rad_s;
    double sensor_gyro_raw_z_rad_s;

    double sensor_mag_raw_x_mG;
    double sensor_mag_raw_y_mG;
    double sensor_mag_raw_z_mG;

    double gyro_filt_x_rad_s;
    double gyro_filt_y_rad_s;
    double gyro_filt_z_rad_s;

    double sensor_ultrasound_height_m;

    double speed_body_x_m_s;
    double speed_body_y_m_s;
    double speed_body_z_m_s;

    double test_x;
    double test_y;
    double test_z;


    int one = 1496; //speed_ned_ref_x

    std::vector<std::pair<double *, int>> data_table = { // Firmware 4.0.6
            {&sensor_acc_raw_x_m_s2, 1240},         //ok
            {&sensor_acc_raw_y_m_s2, 1248},         //ok
            {&sensor_acc_raw_z_m_s2, 1256},         //ok

            {&sensor_gyro_raw_x_rad_s, 1520},       //ok
            {&sensor_gyro_raw_y_rad_s, 1528},       //ok
            {&sensor_gyro_raw_z_rad_s, 1536},       //ok

            {&sensor_mag_raw_x_mG, 1392},
            {&sensor_mag_raw_y_mG, 1400},
            {&sensor_mag_raw_z_mG, 1408},

            {&gyro_filt_x_rad_s, 632},              //ok
            {&gyro_filt_y_rad_s, 640},              //ok
            {&gyro_filt_z_rad_s, 648},              //ok

            {&sensor_ultrasound_height_m, 1424},    //ok

            {&speed_body_x_m_s, 1520},              //ok
            {&speed_body_y_m_s, 1528},              //ok
            {&speed_body_z_m_s, 1536},              //ok

            {&test_x, one},
            {&test_y, one+FULL_NAVDATA_DATASIZE},
            {&test_z, one+(2*FULL_NAVDATA_DATASIZE)}

    };

/*
    double d;
    memcpy(&d, _navdata_buf.data() + 1440, FULL_NAVDATA_DATASIZE);

    double t;
    memcpy(&t, _navdata_buf.data() + 1336, FULL_NAVDATA_DATASIZE);

    int64_t tt;
    memcpy(&tt, _navdata_buf.data() + 1336, FULL_NAVDATA_DATASIZE);
*/

    for(const std::pair<double *, int> &element : data_table)
    {
        memcpy(
                element.first,
                _navdata_buf.data() + element.second,
                FULL_NAVDATA_DATASIZE
        );
    }
    _accelerometer_raw = Eigen::Vector3f(
            (float)sensor_acc_raw_x_m_s2,
            (float)sensor_acc_raw_y_m_s2,
            (float)sensor_acc_raw_z_m_s2
    );

    _gyroscope_raw = Eigen::Vector3f(
            (float)sensor_gyro_raw_x_rad_s,
            (float)sensor_gyro_raw_y_rad_s,
            (float)sensor_gyro_raw_z_rad_s
    );
    _magnetometer_raw = Eigen::Vector3f(
            (float)sensor_mag_raw_x_mG,
            (float)sensor_mag_raw_y_mG,
            (float)sensor_mag_raw_z_mG
    );
    _gyroscope_filt = Eigen::Vector3f(
            (float)gyro_filt_x_rad_s,
            (float)gyro_filt_y_rad_s,
            (float)gyro_filt_z_rad_s
    );

    _test = Eigen::Vector3f(
            (float)test_x,
            (float)test_y,
            (float)test_z
    );

    _body_speed = Eigen::Vector3f(
            (float)speed_body_x_m_s,
            (float)speed_body_y_m_s,
            (float)speed_body_z_m_s
    );


    _height_ultrasonic = (float) sensor_ultrasound_height_m;

    // Listen for next packet
    _spinlock.clear(std::memory_order_release);
    _navdata_socket->async_receive_from(boost::asio::buffer(_navdata_buf), _navdata_sender_endpoint, boost::bind(&fullnavdata::navdataPacketReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void fullnavdata::runUpdateLoop() {
    _ioService.run();
}


void fullnavdata::startReceive() {
    if(_updater == nullptr){
        _updater = new boost::thread(&fullnavdata::runUpdateLoop, this);
    }
}
/// ****** LOCKER
void fullnavdata::lock(){
    while(_spinlock.test_and_set(std::memory_order_acquire));
}

void fullnavdata::release(){
    _spinlock.clear(std::memory_order_release);
}

/// ****** GETTERS

bool fullnavdata::receivedData() {
    return _gotNavdata;
}

Eigen::Vector3f fullnavdata::get_accelerometer_raw() const {
    return _accelerometer_raw;
}

Eigen::Vector3f fullnavdata::get_gyroscope_raw() const {
    return _gyroscope_raw;
}

float fullnavdata::get_height_ultrasonic() const {
    return _height_ultrasonic;
}

Eigen::Vector3f fullnavdata::get_body_speed() const {
    return _body_speed;
}

Eigen::Vector3f fullnavdata::get_magnetometer_raw() const {
    return _magnetometer_raw;
}

Eigen::Vector3f fullnavdata::get_gyroscope_filt() const {
    return _gyroscope_filt;
}

Eigen::Vector3f fullnavdata::get_test() const {
    return _test;
}
