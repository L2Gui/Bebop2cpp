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
#define DRONE_MAX_ALTITUDE          1.0
#define DRONE_MAX_HORIZONTAL_SPEED  0.3
#define DRONE_MAX_VERTICAL_SPEED    0.3
#define LAND_AFTER_LAST_WAYPOINT    true
#define CALIBRATION_FILE            "res/calib_bd2.xml"
#define HULLPROTECTIONON            true
#define LOOK_FOR_CHESSBOARD         false

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

    /// *********************************** VIDEO ANALYSIS INITIALISATION. TODO SHOULD BE INSIDE A METHOD OF DRONE CLASS

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

    /** **/
    std::cout << "GO NAVDATA" << std::endl;
    Eigen::initParallel();
    assert(d.useFullNavdata());
    sleep(1);
    assert(d.isUsingFullNavdata());
    std::cout << "YATA" << std::endl;

    Gnuplot gp;
    gp << "set title \"RAW\"\n";
    std::deque<float> lolx;
    std::deque<float> loly;
    std::deque<float> lolz;
    Eigen::Vector3f vec;


    Gnuplot gp2;
    gp2 << "set title \"CORRECTED\"\n";
    std::deque<float> lolx2;
    std::deque<float> loly2;
    std::deque<float> lolz2;
    Eigen::Vector3f vec2;
    /*
    while(1){
        if(lolx.size() > 150){
            lolx.pop_front();
            loly.pop_front();
            lolz.pop_front();
        }

        vec = d._navdata.get_gyroscope_raw();


        lolx.push_back(vec(0));
        loly.push_back(vec(1));
        lolz.push_back(vec(2));

        gp << "plot '-' binary" << gp.binFmt1d(lolx, "array") << "with lines title \"acce_x\",";
        gp << "'-' binary" << gp.binFmt1d(loly, "array") << "with lines title \"acce_y\",";
        gp << "'-' binary" << gp.binFmt1d(lolz, "array") << "with lines title \"acce_z\"\n";
        gp.sendBinary1d(lolx);
        gp.sendBinary1d(loly);
        gp.sendBinary1d(lolz);
        gp.flush();

        usleep(15000);
    }
     */

/*
    while(1){
        sleep(1);
    }
    return 0;
*/
    /** **/
    while(proceed)
    {
        frame = d.retrieveLastFrame();
        if(frame.data != NULL) {

            std::vector<cv::Point2d> corners;
            bool chess_board = false;

            /***********************************************************************************************/


            if (lolx.size() > 150) {
                lolx.pop_front();
                loly.pop_front();
                lolz.pop_front();
            }
/*
            if (lolx2.size() > 150) {
                lolx2.pop_front();
                loly2.pop_front();
                lolz2.pop_front();
            }
*/
        //    d._navdata.lock();
            vec = d._navdata->get_gyroscope_filt();
      //      vec2 = d._navdata.get_body_speed();
    //        d._navdata.release();


            lolx.push_back(vec(0));
            loly.push_back(vec(1));
            lolz.push_back(vec(2));

/*
            lolx2.push_back(vec2(0));
            loly2.push_back(vec2(1));
            lolz2.push_back(vec2(2));
*/

            gp << "plot '-' binary" << gp.binFmt1d(lolx, "array") << "with lines title \"body x\",";
            gp << "'-' binary" << gp.binFmt1d(loly, "array") << "with lines title \"body y\",";
            gp << "'-' binary" << gp.binFmt1d(lolz, "array") << "with lines title \"body z\"\n";/*,";
            gp << "'-' binary" << gp.binFmt1d(lolx2, "array") << "with lines title \"NED x\",";
            gp << "'-' binary" << gp.binFmt1d(loly2, "array") << "with lines title \"NED y\",";
            gp << "'-' binary" << gp.binFmt1d(lolz2, "array") << "with lines title \"NED z\"\n";
            */
            gp.sendBinary1d(lolx);
            gp.sendBinary1d(loly);
            gp.sendBinary1d(lolz);
            /*
            gp.sendBinary1d(lolx2);
            gp.sendBinary1d(loly2);
            gp.sendBinary1d(lolz2);
            */
            gp.flush();


/*
            gp2 << "plot '-' binary" << gp2.binFmt1d(lolx2, "array") << "with lines title \"x\",";
            gp2 << "'-' binary" << gp2.binFmt1d(loly2, "array") << "with lines title \"y\",";
            gp2 << "'-' binary" << gp2.binFmt1d(lolz2, "array") << "with lines title \"z\"\n";
            gp2.sendBinary1d(lolx2);
            gp2.sendBinary1d(loly2);
            gp2.sendBinary1d(lolz2);
            gp2.flush();
*/
            /***********************************************************************************************/



            if (LOOK_FOR_CHESSBOARD) {
                cv::findChessboardCorners(frame, cv::Size(chess_x, chess_y), corners);
                //cv::cornerSubPix(frame, corners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria)


                if (chess_board) {
                    /// **************************************************************************** DECORATIONS FOR THE IHM
                    /*for(int i = 0; i < corners.size(); ++i){
                        cv::circle(frame, corners[i], 5, cv::Scalar(0, 0, 255));
                    }*/

                    cv::circle(frame, corners[ref_pt], 20, cv::Scalar(0, 255, 0));

                    cv::solvePnP(model_pts, corners, camMatrix, distCoeff, rvec, tvec, !first_time);

                    cv::Mat rotM;
                    std::vector<cv::Point2d> imgPts;
                    cv::projectPoints(axis, rvec, tvec, camMatrix, distCoeff, imgPts);

                    cv::line(frame, corners[ref_pt], imgPts[0], cv::Scalar(255, 0, 0), 5);
                    cv::line(frame, corners[ref_pt], imgPts[1], cv::Scalar(0, 255, 0), 5);
                    cv::line(frame, corners[ref_pt], imgPts[2], cv::Scalar(0, 0, 255), 5);


                    /// ****************************************************************** GETTING THE POSITION OF THE DRONE
                    cv::Rodrigues(rvec, rotM);

                    cv::Mat camPos = rotM.t() * tvec;
                    camPos *= unit;

                    dist.x = camPos.at<double>(0, 0);
                    dist.y = camPos.at<double>(1, 0);
                    dist.z = camPos.at<double>(2, 0);

                    distFromWished = dist - wishedPosition;


                    cv::putText(frame, "Dist from chessboard", cv::Point(10, 30), cv::QT_FONT_NORMAL, 1,
                                cv::Scalar(0, 0, 0), 2, 8);
                    cv::putText(frame, std::to_string((int) dist.x), cv::Point(10, 80), cv::QT_FONT_NORMAL, 1,
                                cv::Scalar(255, 0, 0), 2, 8);
                    cv::putText(frame, std::to_string((int) dist.y), cv::Point(10, 130), cv::QT_FONT_NORMAL, 1,
                                cv::Scalar(0, 255, 0), 2, 8);
                    cv::putText(frame, std::to_string((int) dist.z), cv::Point(10, 180), cv::QT_FONT_NORMAL, 1,
                                cv::Scalar(0, 0, 255), 2, 8);

                    if (FOLLOW) {

                        cv::putText(frame, "Dist from wished", cv::Point(10, 230), cv::QT_FONT_NORMAL, 1,
                                    cv::Scalar(0, 0, 0), 2, 8);
                        cv::putText(frame, std::to_string((int) distFromWished.x), cv::Point(10, 280), cv::QT_FONT_NORMAL,
                                    1, cv::Scalar(255, 0, 0), 2, 8);
                        cv::putText(frame, std::to_string((int) distFromWished.y), cv::Point(10, 330), cv::QT_FONT_NORMAL,
                                    1, cv::Scalar(0, 255, 0), 2, 8);
                        cv::putText(frame, std::to_string((int) distFromWished.z), cv::Point(10, 380), cv::QT_FONT_NORMAL,
                                    1, cv::Scalar(0, 0, 255), 2, 8);


                        // AskedDx, y, z are values according to the drone axis
                        float askedDx = (float) (distFromWished.z / 1000);
                        float askedDy = -(float) (distFromWished.x / 1000);
                        float askedDz = -(float) (distFromWished.y / 1000);

                        askedDx = valueIfAboveEpsilon(askedDx, 0.1);
                        askedDy = valueIfAboveEpsilon(askedDy, 0.1);
                        askedDz = valueIfAboveEpsilon(askedDz, 0.1);

                        cv::putText(frame, "Asked Dy", cv::Point(100, 280), cv::QT_FONT_NORMAL, 1, cv::Scalar(0, 0, 0), 2,
                                    8); //x
                        cv::putText(frame, "Asked Dz", cv::Point(100, 330), cv::QT_FONT_NORMAL, 1, cv::Scalar(0, 0, 0), 2,
                                    8); //y
                        cv::putText(frame, "Asked Dx", cv::Point(100, 380), cv::QT_FONT_NORMAL, 1, cv::Scalar(0, 0, 0), 2,
                                    8); //z

                        cv::putText(frame, std::to_string(askedDy), cv::Point(300, 280), cv::QT_FONT_NORMAL, 1,
                                    cv::Scalar(255, 0, 0), 2, 8);
                        cv::putText(frame, std::to_string(askedDz), cv::Point(300, 330), cv::QT_FONT_NORMAL, 1,
                                    cv::Scalar(0, 255, 0), 2, 8);
                        cv::putText(frame, std::to_string(askedDx), cv::Point(300, 380), cv::QT_FONT_NORMAL, 1,
                                    cv::Scalar(0, 0, 255), 2, 8);

                        /// **************************************************** TILT OF THE CAMERA TO FOLLOW THE CHESSBOARD
                        double rotate = atan(tvec.at<double>(0, 0) / tvec.at<double>(2, 0));
                        cv::Point center(frame.cols / 2, frame.rows / 2);

                        cv::line(frame, center - cv::Point(0, 100), center + cv::Point(0, 100), cv::Scalar(0, 0, 0), 2);
                        cv::line(frame, center - cv::Point(0, 100), center + cv::Point(0, 100), cv::Scalar(255, 255, 255),
                                 1);

                        cv::line(frame, center - cv::Point(100, 0), center + cv::Point(100, 0), cv::Scalar(0, 0, 0), 2);
                        cv::line(frame, center - cv::Point(100, 0), center + cv::Point(100, 0), cv::Scalar(255, 255, 255),
                                 1);

                        //rotate = valueIfAboveEpsilon(rotate, 0.01);

                        camTilt = atan(tvec.at<double>(1, 0) / tvec.at<double>(2, 0));

                        camTilt *= -57.9;

                        camTilt -= 13; // DEFAULT

                        if (abs((int) (camTilt - prevCamTilt)) > 1) {
                            std::cout << "NEW TILT " << camTilt << std::endl;
                            //TODO Changing camera tilt probably reduces position precision since it's numerical
                            // stabilisation with a fish eye and not optical stabilisation
                            d.rotateCamera((float) camTilt, 0);
                            prevCamTilt = camTilt;
                        }
                        /// ************************************************************************* PROPER MOVE BY COMMAND

                        d.moveBy(askedDx, askedDy, askedDz, (float) rotate);

                        /// ************************** IF THE DRONE REACHED THE CURRENT WAYPOINT, GO TO THE NEXT ONE OR LAND

                        if (askedDx == 0 and askedDy == 0) {
                            ++stepCurrent;
                            if (stepCurrent >= stepsCount) {
                                std::cout << "ALL WAYPOINTS REACHED" << std::endl;
                                if (LAND_AFTER_LAST_WAYPOINT) {
                                    std::cout << "ALL WAYPOINTS LAND" << std::endl;
                                    d.land();
                                }
                            } else {
                                std::cout << "WAYPOINT " << stepCurrent << "/" << stepsCount << " REACHED" << std::endl;
                                wishedPosition = steps[stepCurrent];
                            }
                        }
                    }

                    first_time = false;
                } else {
                    /// ***************************************************************************** NO CHESSBOARD DETECTED
                    cv::putText(frame,
                                "STOP EVERYTHIG, NO CHESSBOARD !",
                                cv::Point(10, 300),
                                cv::QT_FONT_NORMAL,
                                1,
                                cv::Scalar(0, 0, 255),
                                3,
                                8
                    );
                    d.moveBy(0, 0, 0, 0);
                    // Better without. TODO look around to find the board if it's not in the sight of the drone
                    //d.rotateCamera(-13.0f, 0);
                    first_time = true;
                }
            }
            /// ********************************************************************************************** IHM STUFF

            std::string help("T: takeoff, L: land, F: follow");
            help.append(FOLLOW ? " (on)" : " (off)");
            help.append(", Q: emergency, S: stop");
            cv::putText(frame, help, cv::Point(10,465), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,255,255), 2);
            cv::putText(frame, help, cv::Point(10,465), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 1);

            cv::putText(frame, std::to_string(d.getBatteryLvl()), cv::Point(800, 30),
                        cv::QT_FONT_NORMAL, 1, cv::Scalar(255, 0, 255), 3, 8);


            cv::imshow(droneIp, frame);
        }


        /// ****************************************************************************************** IHM KEY REACTIONS

        char k = (char)cv::waitKey(10);
        switch(k){
            case 'l':
                d.land();
                break;
            case 'q':
                d.emergency();
                proceed = false;
                break;
            case 's':
                std::cout << "STOP" << std::endl;
                d.modifyYaw(0);
                d.modifyRoll(0);
                d.modifyPitch(0);
                d.modifyAltitude(0);
                assert(d.moveBy(0, 0, 0, 0));
                FOLLOW = false;
                break;
            case 'f':
                FOLLOW = !FOLLOW;
                if(!FOLLOW){
                    assert(d.moveBy(0,0,0,0));
                }
                break;
            case 't':
                d.takeOff();
                break;
                /// *********************************************************************************** IHM "CHEAT" KEYS
            case 'u':
                std::cout << "move 0.5m up" << std::endl;
                assert(d.moveBy(0, 0, -0.5f, 0));
                break;
            case 'd':
                std::cout << "move 0.5m down" << std::endl;
                assert(d.moveBy(0, 0, 0.5f, 0));
                break;
            case 'y':
                std::cout << "move 0.5m forward" << std::endl;
                assert(d.moveBy(0.5f, 0, 0, 0));
                break;
            case 'h':
                std::cout << "move 0.5m backward" << std::endl;
                assert(d.moveBy(-0.5f, 0, 0, 0));
                break;
            case 'j':
                std::cout << "move 0.5m right" << std::endl;
                assert(d.moveBy(0, 0.5f, 0, 0));
                break;
            case 'g':
                std::cout << "move 0.5m left" << std::endl;
                assert(d.moveBy(0, -0.5f, 0, 0));
                break;
            case '+':
                std::cout << "turn 90° right" << std::endl;
                assert(d.moveBy(0, 0, 0, 1.57f));
                break;
            case '-':
                std::cout << "turn 90° left" << std::endl;
                assert(d.moveBy(0, 0, 0, -1.57f));
                break;

            case 'w':
                d._navdata->lock();
                std::cout << "data locked" << std::endl;
                break;
            case 'x':
                d._navdata->release();
                std::cout << "data released" << std::endl;
                break;
            default:
                break;
        }
    }

    return 0;
}
