#include <vector>
#include "Drone.h"
#include <math.h>
#include <boost/asio/io_service.hpp>
#include <fullnavdata.h>
#include <gnuplot_iostream.h>
#include <deque>


/*
 * PRIVATE HEADER
 */

#define DRONE_IP                    "10.42.0.10"
#define CALIBRATION_FILE            "res/calib_bd2.xml"
#define HULLPROTECTIONON            true

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


    /// ******************************************************************************************** CONFIGURE WAYPOINTS
    std::vector<cv::Point3d> steps = {
            cv::Point3d(-1000, 0, 1500),
            cv::Point3d(1000, 0, 1500),
            cv::Point3d(-500, 0, 2000),
            cv::Point3d(500, 0, 2000),
            cv::Point3d(0, 0, 1500),
            cv::Point3d(0, 0, 1000)
    };

    int stepCurrent= 0;
    int stepsCount = (int) steps.size();

    cv::Point3d wishedPosition(steps[0]);

    /// ***************************************************************************** CONNECT TO AND CONFIGURE THE DRONE
    Drone d;//("10.42.0.11");
    assert(d.connect());

    // Waiting for the drone to be ready
    while(!d.isRunning()){ sleep(1); }

    std::string droneIp = d.getIpAddress();

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

    // Camera calibration file
    cv::FileStorage fs(CALIBRATION_FILE, cv::FileStorage::READ);

    if(!fs.isOpened()){
        std::cout << "Can't open calibration file!" << std::endl;
        exit(1);
    }

    cv::Mat camMatrix, distCoeff;
    float unit;

    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeff;
    fs["square_size"] >> unit;
    fs.release();


    // Chessboard specifications
    int chess_x = 9;
    int chess_y = 6;
    int ref_pt = 22;

    cv::Mat model_pts = cv::Mat::zeros(chess_x * chess_y, 3, CV_32FC1);

    int i = 0;
    for(int y = -(ref_pt/chess_x); y < chess_y - (ref_pt/chess_x); ++y){
        for(int x = -(ref_pt%chess_x); x < chess_x - ref_pt%chess_x; ++x){
            model_pts.at<float>(i, 0) = x;
            model_pts.at<float>(i, 1) = y;
            ++i;
        }
    }

    // Axis to be displayed on the video (it helps to see if the calibration is ~correct or not)
    std::vector<cv::Point3d> axis;
    axis.push_back(cv::Point3d(3, 0, 0));
    axis.push_back(cv::Point3d(0, 3, 0));
    axis.push_back(cv::Point3d(0, 0, -3));


    cv::Mat rvec, tvec;

    cv::Point3d distFromWished(0,0,0);
    cv::Point3d dist(0,0,0);

    cv::Mat frame;

    bool FOLLOW = false;    // if TRUE, the drone will follow the chessboard
    bool proceed = true;    // if TRUE, the program will interact with the drone
    bool first_time = true; // if TRUE, the program will use the previous chessboard location as a hint

    double camTilt, prevCamTilt = INT_MAX/2;

    cv::namedWindow(droneIp);

    Eigen::initParallel();
    assert(d.useFullNavdata());
    sleep(1);
    assert(d.isUsingFullNavdata());

    Gnuplot gp;
    gp << "set title \"Gyroscope (filt)\"\n";
    gp << "set ylabel \"rad/s\"\n";
    std::deque<float> x1;
    std::deque<float> y1;
    std::deque<float> z1;
    Eigen::Vector3f vec;


    Gnuplot gp2;
    gp2 << "set title \"Accelerometer (raw)\"\n";
    gp2 << "set ylabel \"m/s²\"\n";
    std::deque<float> x2;
    std::deque<float> y2;
    std::deque<float> z2;
    Eigen::Vector3f vec2;


    Gnuplot gp3;
    gp3 << "set title \"Magnetometer (raw)\"\n";
    gp3 << "set ylabel \"mG\"\n";
    std::deque<float> x3;
    std::deque<float> y3;
    std::deque<float> z3;
    Eigen::Vector3f vec3;

    while(true)
    {
        if (x1.size() > 150) {
            x1.pop_front();
            y1.pop_front();
            z1.pop_front();
        }

        if (x2.size() > 150) {
            x2.pop_front();
            y2.pop_front();
            z2.pop_front();
        }

        if (x3.size() > 150) {
            x3.pop_front();
            y3.pop_front();
            z3.pop_front();
        }

        d._navdata.lock();
        vec = d._navdata.get_gyroscope_filt();
        vec2 = d._navdata.get_accelerometer_raw();
        vec3 = d._navdata.get_magnetometer_raw();
        d._navdata.release();


        x1.push_back(vec(0));
        y1.push_back(vec(1));
        z1.push_back(vec(2));


        x2.push_back(vec2(0));
        y2.push_back(vec2(1));
        z2.push_back(vec2(2));

        x3.push_back(vec3(0));
        y3.push_back(vec3(1));
        z3.push_back(vec3(2));

        gp << "plot '-' binary" << gp.binFmt1d(x1, "array") << "with lines title \"x\",";
        gp << "'-' binary" << gp.binFmt1d(y1, "array") << "with lines title \"y\",";
        gp << "'-' binary" << gp.binFmt1d(z1, "array") << "with lines title \"z\"\n";

        gp.sendBinary1d(x1);
        gp.sendBinary1d(y1);
        gp.sendBinary1d(z1);

        gp.flush();


        gp2 << "plot '-' binary" << gp2.binFmt1d(x2, "array") << "with lines title \"x\",";
        gp2 << "'-' binary" << gp2.binFmt1d(y2, "array") << "with lines title \"y\",";
        gp2 << "'-' binary" << gp2.binFmt1d(z2, "array") << "with lines title \"z\"\n";

        gp2.sendBinary1d(x2);
        gp2.sendBinary1d(y2);
        gp2.sendBinary1d(z2);

        gp2.flush();


        gp3 << "plot '-' binary" << gp3.binFmt1d(x3, "array") << "with lines title \"x\",";
        gp3 << "'-' binary" << gp3.binFmt1d(y3, "array") << "with lines title \"y\",";
        gp3 << "'-' binary" << gp3.binFmt1d(z3, "array") << "with lines title \"z\"\n";

        gp3.sendBinary1d(x3);
        gp3.sendBinary1d(y3);
        gp3.sendBinary1d(z3);

        gp3.flush();

        usleep(100000);
    }

    return 0;
}
