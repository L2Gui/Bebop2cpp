/** TODO MIT **/

#ifndef BEBOP_FULLNAVDATA_H
#define BEBOP_FULLNAVDATA_H

#include <array>
#include <memory>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <deque>

#include <Eigen/Dense>

#define FULL_NAVDATA_MAX_SIZE 4096
#define FULL_NAVDATA_DATASIZE 8

const int FULL_NAVDATA_DEFAULT_PORT	= 56789;

class Gnuplot;


class fullnavdata
{
public:
    fullnavdata();
    void init(std::string ip, int senderPort= FULL_NAVDATA_DEFAULT_PORT);

    void startReceive();

    bool receivedData();

    Eigen::Vector3f get_accelerometer_raw() const;
    Eigen::Vector3f get_gyroscope_raw() const;
    float get_height_ultrasonic() const;
    Eigen::Vector3f get_speed() const;

    void lock();
    void release();

private:

    std::atomic_flag _spinlock;

    Eigen::Vector3f _accelerometer_raw;
    Eigen::Vector3f _gyroscope_raw;
    std::atomic<float> _height_ultrasonic;
    Eigen::Vector3f _speed;


    /*******/

    std::deque<double> lolx;
    std::deque<double> loly;
    std::deque<double> lolz;
    Gnuplot* _accelerometerPlot;
    //Gnuplot* _gyroPlot;

    /***********/

    void navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred);

    std::unique_ptr<boost::asio::ip::udp::socket> _navdata_socket = nullptr;
    boost::asio::ip::udp::endpoint endpoint;

    boost::asio::ip::udp::endpoint _navdata_sender_endpoint;

    std::array<char, FULL_NAVDATA_MAX_SIZE> _navdata_buf;

    bool _gotNavdata = false;

    boost::asio::io_service _ioService;
    boost::thread *_updater = nullptr;

    void runUpdateLoop();

};
#endif //BEBOP_FULLNAVDATA_H
