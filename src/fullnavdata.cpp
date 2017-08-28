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
    :_accelerometerPlot(new Gnuplot()),
     _gyroPlot(new Gnuplot())
{
    *_accelerometerPlot << "set title \"Accelerometer (m/s^2)\"\n";
    *_gyroPlot << "set title \"Gyroscope (rad/s)\"\n";
}

void fullnavdata::init(std::string ip)
{
    std::cout << "INIT" << std::endl;
    _navdata_socket.reset(new udp::socket(_ioService, udp::v4()));
    _navdata_socket->bind(udp::endpoint(udp::v4(), FULL_NAVDATA_PORT));

    udp::resolver resolver(_ioService);
    udp::resolver::query query(ip, std::to_string(FULL_NAVDATA_PORT));
    endpoint = *resolver.resolve(query);
    //lol

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
    if(bytes_transferred < 1000)
    {
        return;
    }

    _gotNavdata = true;

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
    double sensor_acc_raw_x_m_s2;
    double sensor_acc_raw_y_m_s2;
    double sensor_acc_raw_z_m_s2;

    double sensor_gyro_raw_x_rad_s;
    double sensor_gyro_raw_y_rad_s;
    double sensor_gyro_raw_z_rad_s;

    double sensor_ultrasound_height_m;

    /*std::vector<std::pair<double *, int>> data_table = { // Firmware 3.2
            {&roll, 61},
            {&pitch, 62},
            {&yaw, 63},
            {&height, 76},
            {&height_ultrasonic, 15},
            {&pressure, 9},
            {&vbat, 58},
            {&latitude, 22},
            {&longitude, 23},
            {&gps_altitude, 24},
            {&bat_percent, 59},
            {&speed_x, 73},
            {&speed_y, 74},
            {&speed_z, 75},
            {&gps_accuracy, 27},
            {&gps_num_svs, 28},
            {&gps_eph, 32},
            {&gps_epv, 34}
    };*/

    std::vector<std::pair<double *, int>> data_table = { // Firmware 4.0.6
            {&sensor_acc_raw_x_m_s2, 1240},
            {&sensor_acc_raw_y_m_s2, 1248},
            {&sensor_acc_raw_z_m_s2, 1256},

            {&sensor_gyro_raw_x_rad_s, 1352},
            {&sensor_gyro_raw_y_rad_s, 1360},
            {&sensor_gyro_raw_z_rad_s, 1368},

            {&sensor_ultrasound_height_m, 1424}

    };
    for(const std::pair<double *, int> &element : data_table)
    {
        memcpy(element.first, _navdata_buf.data() + element.second, FULL_NAVDATA_DATASIZE);
    }
/*
    _navdata.altitude = height;
    _navdata.attitude = Eigen::Vector3f(pitch, roll, yaw);
    _navdata.batterystatus = bat_percent / 100.0f;
    _navdata.full = true;
    _navdata.gps_altitude = gps_altitude;
    _navdata.gps_sats = gps_num_svs;
    _navdata.latitude = latitude;
    _navdata.linearvelocity = Eigen::Vector3f(speed_x, speed_y, speed_z);
    _navdata.longitude = longitude;

    _navdata.full_navdata.ultrasound_height = height_ultrasonic;
    _navdata.full_navdata.pressure = pressure;
    _navdata.full_navdata.vbat = vbat;
    _navdata.full_navdata.gps_accuracy = gps_accuracy;
    _navdata.full_navdata.gps_eph = gps_eph;
    _navdata.full_navdata.gps_epv = gps_epv;
*/

    if(_acceX.size() > 150){
        _acceX.pop_front();
        _acceY.pop_front();
        _acceZ.pop_front();
        _gyroX.pop_front();
        _gyroY.pop_front();
        _gyroZ.pop_front();
    }

    _acceX.push_back(sensor_acc_raw_x_m_s2);
    _acceY.push_back(sensor_acc_raw_y_m_s2);
    _acceZ.push_back(sensor_acc_raw_z_m_s2);

    _gyroX.push_back(sensor_gyro_raw_x_rad_s);
    _gyroY.push_back(sensor_gyro_raw_y_rad_s);
    _gyroZ.push_back(sensor_gyro_raw_z_rad_s);

    /*
    _g->reset_all();

    _g->plot_x(_x, "acce x");
    _g->plot_x(_y, "acce y");
    _g->plot_x(_z, "acce z");
    */

    /************/

    //gp << "set yrange [-1:1]\n";



    *_accelerometerPlot << "plot '-' binary" << _accelerometerPlot->binFmt1d(_acceX, "array") << "with lines title \"acce_x\",";
    *_accelerometerPlot << "'-' binary" << _accelerometerPlot->binFmt1d(_acceY, "array") << "with lines title \"acce_y\",";
    *_accelerometerPlot << "'-' binary" << _accelerometerPlot->binFmt1d(_acceZ, "array") << "with lines title \"acce_z\"\n";
    _accelerometerPlot->sendBinary1d(_acceX);
    _accelerometerPlot->sendBinary1d(_acceY);
    _accelerometerPlot->sendBinary1d(_acceZ);
    _accelerometerPlot->flush();



    *_gyroPlot << "plot '-' binary" << _gyroPlot->binFmt1d(_gyroX, "array") << "with lines title \"gyro_x\",";
    *_gyroPlot << "'-' binary" << _gyroPlot->binFmt1d(_gyroY, "array") << "with lines title \"gyro_y\",";
    *_gyroPlot << "'-' binary" << _gyroPlot->binFmt1d(_gyroZ, "array") << "with lines title \"gyro_z\"\n";
    _gyroPlot->sendBinary1d(_gyroX);
    _gyroPlot->sendBinary1d(_gyroY);
    _gyroPlot->sendBinary1d(_gyroZ);
    _gyroPlot->flush();

        /***********************/


    //std::cout << sensor_ultrasound_height_m << std::endl;
    // Listen for next packet
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

fullnavdata::~fullnavdata() {
    delete _accelerometerPlot;
}
/*
shared_ptr<navdata> fullnavdata::getNavdata()
{
    if(!_gotNavdata)
    {
        return nullptr;
    }

    shared_ptr<navdata> navdata = make_shared<bebop::navdata>();

    *navdata = _navdata; // Copy local navdata

    return navdata;
}*/