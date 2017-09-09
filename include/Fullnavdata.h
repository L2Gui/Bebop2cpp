#ifndef BEBOP_FULLNAVDATA_H
#define BEBOP_FULLNAVDATA_H


#include <boost/asio.hpp>

namespace boost{
    class thread;
}

#include <deque>

#include <Eigen/Dense>
#include "Drone.h"

#define FULL_NAVDATA_MAX_SIZE 4096
#define FULL_NAVDATA_DATASIZE 8

const int FULL_NAVDATA_DEFAULT_PORT	= 56789;

class Fullnavdata
{
public:
    Fullnavdata();
    ~Fullnavdata();

    //Fullnavdata class can't be copied! there is only 1 physical drone associated with an instance
    Fullnavdata(Fullnavdata const &) = delete;
    void operator=(Fullnavdata const &fullnavdata) = delete;

    /**
     * Send a message to the navdata server on the drone in order for it to start sending data.
     * You need to check "receivedData" to be sure the navdata server is running
     */
    void startReceive();

    /**
     * Gets information regarding the reception of navigation data
     * @return true if the computer received navdata, false otherwise
     */
    bool receivedData() const;

    /**
     * Gets the accelerometer raw values in m/s²
     * @return a eigen vector3f with (x,y,z)
     */
    Eigen::Vector3f get_accelerometer_raw() const;
    /**
     * Gets the gyroscope raw values in rad/s
     * @return a eigen vector3f with (x,y,z)
     */
    Eigen::Vector3f get_gyroscope_raw() const;
    /**
     * Gets the magnetometer raw values in mG
     * @return a eigen vector3f with (x,y,z)
     */
    Eigen::Vector3f get_magnetometer_raw() const;
    /**
     * Gets the gyroscope filtered values in rad/s
     * It is suspected that the Bebop2 uses some king of kalman filter with the magnetometer
     * @return a eigen vector 3f with x,y,z)
     */
    Eigen::Vector3f get_gyroscope_filt() const;

    /**
     * Gets the height deduced from the sonic sensor in m
     * @warn only works when the drone is flyings
     * @return the deduced height
     */
    float get_height_ultrasonic() const;

    /**
     * Gets the speed of the drone in m/s
     * @return a eigen vector3f with (x,y,z)
     */
    Eigen::Vector3f get_body_speed() const;
    /**
     * Gets the uptime in us corresponding to the time where data were collected on the drone
     * @return
     */
    int64_t get_sent_drone_uptime() const;
    /**
     * Gets the timestamp in us corresponding to the time where the computer received the data
     * @return
     */
    int64_t get_received_time_computer() const;

    /**
     * Locks the reception of navdata. Use it when you want to collect several data
     */
    void lock();
    /**
     * Releases the reception of navdata. Use it when you finished collecting several data
     */
    void release();
    /**
     * Gets a value that is curently being tested, you may not want to use it
     * @return some eigen vector3f
     */
    Eigen::Vector3f get_test() const;

protected:

    /**
     * Initialise the connexion between the computer and the drone using the provided settings
     * @param ip Ip address of the drone
     * @param senderPort port to initialise the connexion (computer will receive navdata on that port)
     */
    void init(std::string ip, int senderPort= FULL_NAVDATA_DEFAULT_PORT);
private:

    std::atomic_flag _spinlock;

    Eigen::Vector3f _accelerometer_raw;
    Eigen::Vector3f _gyroscope_raw;
    Eigen::Vector3f _magnetometer_raw;
    Eigen::Vector3f _gyroscope_filt;
    std::atomic<float> _height_ultrasonic;
    Eigen::Vector3f _body_speed;
    Eigen::Vector3f _test;

    int64_t _sent_drone_uptime;
    int64_t _received_time_computer;


    void navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred);

    std::unique_ptr<boost::asio::ip::udp::socket> _navdata_socket = nullptr;
    boost::asio::ip::udp::endpoint endpoint;

    boost::asio::ip::udp::endpoint _navdata_sender_endpoint;

    std::array<char, FULL_NAVDATA_MAX_SIZE> _navdata_buf;

    bool _gotNavdata = false;

    boost::asio::io_service _ioService;
    boost::thread *_updater = nullptr;

    void runUpdateLoop();

    friend bool Drone::useFullNavdata();
};
#endif //BEBOP_FULLNAVDATA_H
