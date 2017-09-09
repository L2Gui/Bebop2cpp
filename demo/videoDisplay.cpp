#include <vector>
#include "Drone.h"
#include <math.h>
#include <boost/asio/io_service.hpp>
#include <Fullnavdata.h>
#include <gnuplot_iostream.h>
#include <deque>


/*
 * PRIVATE HEADER
 */

#define DRONE_IP                    "10.42.0.10"
#define DRONE_MAX_ALTITUDE          1.0
#define DRONE_MAX_HORIZONTAL_SPEED  0.3
#define DRONE_MAX_VERTICAL_SPEED    0.3
#define LAND_AFTER_LAST_WAYPOINT    true
#define CALIBRATION_FILE            "res/calib_bd2.xml"
#define HULLPROTECTIONON            true
#define LOOK_FOR_CHESSBOARD         true

/*
 * UTILITY FUNCTIONS
 */
/**
 * Returns:
 *      value if value is inside [min; max],
 *      min if value < min,
 *      max if value > max
 * @param value
 * @param min
 * @param max
 * @return
 */
int interval(int value, int min, int max){
    if( min < value) {
        if( value < max){
            return value;
        }else{
            return max;
        }
    }else{
        return min;
    }
}

/**
 * Returns value if value is greater than +epsilon or lower than -epsilon. Otherwise, 0 is returned
 * @param value
 * @param epsilon
 * @return
 */
float valueIfAboveEpsilon(float value, float epsilon){
    if(0 < value and value < epsilon){
        return 0;
    }
    if( -epsilon < value and value < 0){
        return 0;
    }
    return value;

}

/*
 * MAIN
 */
int main(){

    /// ***************************************************************************** CONNECT TO AND CONFIGURE THE DRONE
    Drone d;//(DRONE_IP);

    assert(d.connect());

    // Waiting for the drone to be ready
    while(!d.isRunning()){ sleep(1); }

    std::string droneIp = d.getIpAddress();

    assert(d.setMaxAltitude(DRONE_MAX_ALTITUDE));
    assert(d.setMaxHorizontalSpeed(DRONE_MAX_HORIZONTAL_SPEED));
    assert(d.setMaxVerticalSpeed(DRONE_MAX_VERTICAL_SPEED));

    std::cout << "INITIALISATION IS OK" << std::endl;

    assert(d.blockingStartStreaming());
    std::cout << "STREAMING IS OK" << std::endl;

    // Flat trim at start. The drone MUST be on the ground at that time
    if(d.blockingFlatTrim()) {
        std::cout << "FLAT TRIM IS OK" << std::endl;
    }else{
        std::cerr << "FLAT TRIM NOT OK" << std::endl;
        return 1;
    }

    assert(d.setHullPresence(HULLPROTECTIONON));
    assert(d.setVideoAutorecord(false));

    // Initialising the openCV camera
    d.blockingInitCam();

    cv::Mat frame;
    bool proceed = true;

    int tilt = d.getDefaultTilt();
    int pan = d.getDefaultPan();

    int default_tilt = tilt;
    int default_pan = pan;

    cv::namedWindow(droneIp);

    while(proceed)
    {
        frame = d.retrieveLastFrame();
        if(frame.data != NULL) {

            /// ********************************************************************************************** IHM STUFF

            cv::putText(frame, "Q: quit, NUMPAD (8,2 - 4,6 - 5): orientation", cv::Point(10,465), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,255,255), 2);
            cv::putText(frame, "Q: quit, NUMPAD (8,2 - 4,6 - 5): orientation", cv::Point(10,465), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 1);

            cv::putText(frame, std::to_string(d.getBatteryLvl()), cv::Point(800, 30),
                        cv::QT_FONT_NORMAL, 1, cv::Scalar(255, 0, 255), 3, 8);


            cv::imshow(droneIp, frame);
        }


        /// ****************************************************************************************** IHM KEY REACTIONS

        char k = (char)cv::waitKey(10);
        switch(k){
            case 'q':
                proceed = false;
                break;
            case '8':
                ++tilt;
                d.rotateCamera(tilt, pan);
                break;
            case '6':
                ++pan;
                d.rotateCamera(tilt, pan);
                break;
            case '2':
                --tilt;
                d.rotateCamera(tilt, pan);
                break;
            case '4':
                --pan;
                d.rotateCamera(tilt, pan);
                break;
            case '5':
                pan = default_pan;
                tilt = default_tilt;
                d.rotateCamera(tilt, pan);
            default:
                break;
        }
    }

    return 0;
}
