/** TODO MIT **/

#ifndef BEBOP_FULLNAVDATA_H
#define BEBOP_FULLNAVDATA_H

#include <array>
#include <memory>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <deque>

class Gnuplot;

#define FULL_NAVDATA_MAX_SIZE 4096
#define FULL_NAVDATA_DATASIZE 8

const int FULL_NAVDATA_PORT	= 56789;

class fullnavdata
{
public:
    fullnavdata();
    void init(std::string ip);

    void startReceive();

    ~fullnavdata();


private:
    std::deque<double> _acceX, _acceY, _acceZ;
    std::deque<double> _gyroX, _gyroY, _gyroZ;

    Gnuplot* _accelerometerPlot;
    Gnuplot* _gyroPlot;

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
