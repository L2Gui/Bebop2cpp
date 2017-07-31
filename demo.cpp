#include <vector>
#include "Drone.h"
#include <math.h>


#define DRONE_IP "10.42.0.11"

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

float valueIfAboveEpsilon(float value, float epsilon){
    if(0 < value and value < epsilon){
        return 0;
    }
    if( -epsilon < value and value < 0){
        return 0;
    }
    return value;

}

int main(){

    cv::Point3d step1(-1000, -50, 1500);
    cv::Point3d step2(1000, -50, 1500);
    cv::Point3d step3(0, -50, 1500);


    cv::Point3d wishedPosition(step1);

    Drone d(DRONE_IP);

    assert(d.isValid());

    d.connect();
    //assert(d.connect());


    while(!d.isRunning()){
        sleep(1);
    }

    assert(d.setMaxAltitude(1.0f));


    std::cout << "INITIALISATION IS OK" << std::endl;



    assert(d.startStreaming());

    std::cout << "STREAMING IS OK" << std::endl;

    if(d.blockingFlatTrim()) {
        std::cout << "FLAT TRIM IS OK" << std::endl;

    }else{
        std::cerr << "FLAT TRIM NOT OK" << std::endl;
        return 1;
    }
/*
    initStream();
*/

    cv::VideoCapture cap(d.getVideoPath());
    while(!cap.isOpened()){
        sleep(1);
        cap = cv::VideoCapture(d.getVideoPath());
    }

    /**
     * UGLY CP FROM CPOS ***********************************************************************************************
     */


    cv::FileStorage fs("calib_bd2.xml", cv::FileStorage::READ);

    if(!fs.isOpened()){
        std::cout << "Can't open calibration file!" << std::endl;
        exit(1);
    }

    cv::Mat camMatrix, distCoeff;
    float square_size;

    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeff;
    fs["square_size"] >> square_size;
    fs.release();

    //std::cout << camMatrix << std::endl << std::endl;
    //std::cout << distCoeff << std::endl << std::endl;
    //std::cout << square_size << std::endl << std::endl;

    int chess_x = 9;
    int chess_y = 6;
    int ref_pt = 22;

    /**
     * Chess board pts
     */

    cv::Mat model_pts = cv::Mat::zeros(chess_x * chess_y, 3, CV_32FC1);

    int i = 0;
    for(int y = -(ref_pt/chess_x); y < chess_y - (ref_pt/chess_x); ++y){
        for(int x = -(ref_pt%chess_x); x < chess_x - ref_pt%chess_x; ++x){
            model_pts.at<float>(i, 0) = x;
            model_pts.at<float>(i, 1) = y;
            ++i;
        }
    }

    /**
     * Axis
     */
    std::vector<cv::Point3d> axis;
    axis.push_back(cv::Point3d(3, 0, 0));
    axis.push_back(cv::Point3d(0, 3, 0));
    axis.push_back(cv::Point3d(0, 0, -3));

    bool first_time = true;
    cv::Mat rvec, tvec;
    /**
     * *****************************************************************************************************************
     */
    cv::Point3d distFromWished(0,0,0);
    cv::Point3d dist(0,0,0);

    cv::Mat tmp;
    cv::Mat frame;
    bool FOLLOW = false;



    d.setMaxHorizontalSpeed(0.3f);
    d.setMaxVerticalSpeed(0.3f);
    //d.setMaxRotationSpeed()


    int step = 1;


    cv::namedWindow(DRONE_IP);

    double camTilt, prevCamTilt = INT_MAX/2;

    while(true)
    {
        cap >> tmp;
        if(tmp.data != NULL) {
            // last image only
            while (1) {
                if(tmp.data != NULL) {
                    tmp.copyTo(frame);
                }else{
                    break;
                }
                cap >> tmp;
            }
            /**
             * UGLY CP FROM CPOS AGAIN *********************************************************************************
             */
            std::vector<cv::Point2d> corners;
            bool chess_board = cv::findChessboardCorners(frame, cv::Size(chess_x, chess_y), corners);
            //cv::cornerSubPix(frame, corners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria)
            if(chess_board) {
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



                cv::Rodrigues(rvec, rotM);

                cv::Mat camPos = rotM.t() * tvec;
                camPos *= square_size;
                dist.x = camPos.at<double>(0,0);
                dist.y = camPos.at<double>(1,0);
                dist.z = camPos.at<double>(2,0);

                distFromWished = dist - wishedPosition;


                cv::putText(frame, "Dist from chessboard", cv::Point(10,30), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2, 8);
                cv::putText(frame, std::to_string((int)dist.x), cv::Point(10,80), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,0,0), 2, 8);
                cv::putText(frame, std::to_string((int)dist.y), cv::Point(10,130), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,255,0), 2, 8);
                cv::putText(frame, std::to_string((int)dist.z), cv::Point(10,180), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,255), 2, 8);

                if(FOLLOW) {

                    cv::putText(frame, "Dist from wished", cv::Point(10, 230), cv::QT_FONT_NORMAL, 1,
                                cv::Scalar(0, 0, 0), 2, 8);
                    cv::putText(frame, std::to_string((int) distFromWished.x), cv::Point(10, 280), cv::QT_FONT_NORMAL,
                                1, cv::Scalar(255, 0, 0), 2, 8);
                    cv::putText(frame, std::to_string((int) distFromWished.y), cv::Point(10, 330), cv::QT_FONT_NORMAL,
                                1, cv::Scalar(0, 255, 0), 2, 8);
                    cv::putText(frame, std::to_string((int) distFromWished.z), cv::Point(10, 380), cv::QT_FONT_NORMAL,
                                1, cv::Scalar(0, 0, 255), 2, 8);


                    // ASKED = DRONE AXIS
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

                    double rotate = atan(tvec.at<double>(0,0)/tvec.at<double>(2,0));
                    cv::Point center(frame.cols/2, frame.rows/2);

                    cv::line(frame, center - cv::Point(0, 100), center + cv::Point(0, 100), cv::Scalar(0,0,0), 2);
                    cv::line(frame, center - cv::Point(0, 100), center + cv::Point(0, 100), cv::Scalar(255,255,255), 1);

                    cv::line(frame, center - cv::Point(100, 0), center + cv::Point(100, 0), cv::Scalar(0,0,0), 2);
                    cv::line(frame, center - cv::Point(100, 0), center + cv::Point(100, 0), cv::Scalar(255,255,255), 1);

                    //rotate = valueIfAboveEpsilon(rotate, 0.01);

                    camTilt = atan(tvec.at<double>(1,0)/tvec.at<double>(2,0));

                    camTilt *= -57.9;

                    camTilt -= 13; // DEFAULT

                    if( abs((int)(camTilt - prevCamTilt)) > 1 ) {
                        std::cout << "NEW TILT " << camTilt << std::endl;
                        //TODO Changing camera tilt probably reduces position precision since it's numerical
                        // stabilisation with a fish eye and not optical stabilisation
                        d.rotateCamera((float) camTilt, 0);
                        prevCamTilt = camTilt;
                    }


                    if(askedDx == 0 and askedDy == 0){
                        if(step == 1){
                            wishedPosition = step2;
                            std::cout << "STEP1 ok" << std::endl;
                        }else if(step == 2) {
                            wishedPosition = step3;
                            std::cout << "STEp2 ok" << std::endl;
                        }else{
                            std::cout << "LAND" << std::endl;
                            d.land();
                        }
                        step++;
                    }
                    d.moveBy(askedDx,askedDy, askedDz, (float)rotate);

                    /*
                    /// OLD WAY WITH PITCH ROLL AND ALT COMMANDS
                    int askedRoll = (int)(distFromWished.x/30);
                    int askedPitch = (int)(distFromWished.z/30);
                    int askedAlt = (int)(distFromWished.y/50);

                    askedRoll = interval(askedRoll, -10, 10);
                    askedAlt = interval(askedAlt, -10, 10);
                    askedPitch = interval(askedPitch, -10, 10);

                    cv::putText(frame, "Asked roll", cv::Point(100,280), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2, 8);
                    cv::putText(frame, "Asked altit", cv::Point(100,330), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2, 8);
                    cv::putText(frame, "Asked pitch", cv::Point(100,380), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2, 8);

                    cv::putText(frame, std::to_string(askedRoll), cv::Point(300,280), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,0,0), 2, 8);
                    cv::putText(frame, std::to_string(askedAlt), cv::Point(300,330), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,255,0), 2, 8);
                    cv::putText(frame, std::to_string(askedPitch), cv::Point(300,380), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,255), 2, 8);

                    //d.modifyRoll((int8_t) askedRoll);
                    //d.modifyAltitude((int8_t) askedAlt);
                    //d.modifyPitch((int8_t) askedPitch);
                    */
                }

                first_time = false;
            }else{
                cv::putText(frame, "STOP EVERYTHIG, NO CHESSBOARD !", cv::Point(10,300), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,255), 3, 8);
                d.moveBy(0,0,0,0);
                //d.rotateCamera(-13.0f, 0);
                first_time = true;
            }
            /**
             * *********************************************************************************************************
             */
            std::string help("T: takeoff, L: land, F: follow");
            help.append(FOLLOW ? " (on)" : " (off)");
            help.append(", Q: emergency, S: stop");
            cv::putText(frame, help, cv::Point(10,465), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,255,255), 2);
            cv::putText(frame, help, cv::Point(10,465), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 1);

            cv::putText(frame, std::to_string(d.getBatteryLvl()), cv::Point(800, 30),
                        cv::QT_FONT_NORMAL, 1, cv::Scalar(255, 0, 255), 3, 8);


            cv::imshow(DRONE_IP, frame);
        }
        char k = (char)cv::waitKey(10);

        if(k == 'l') {
            d.land();
        }else
        if(k == 'q') {
            d.emergency();
            break;
        }else
        if(k == 'f') {
            FOLLOW = !FOLLOW;
            if(!FOLLOW){
                assert(d.moveBy(0,0,0,0));
            }
        }else
        if(k == 't') {
            d.takeOff();
        }else
        if(k == 'u'){
            std::cout << "move 0.5m up" << std::endl;
            assert(d.moveBy(0, 0, -0.5f, 0));
        }else
        if(k == 'd'){
            std::cout << "move 0.5m down" << std::endl;
            assert(d.moveBy(0, 0, 0.5f, 0));
        }else
        if(k == 'y'){
            std::cout << "move 0.5m forward" << std::endl;
            assert(d.moveBy(0.5f, 0, 0, 0));
        }else
        if(k == 'h'){
            std::cout << "move 0.5m backward" << std::endl;
            assert(d.moveBy(-0.5f, 0, 0, 0));
        }else
        if(k == 'j'){
            std::cout << "move 0.5m right" << std::endl;
            assert(d.moveBy(0, 0.5f, 0, 0));
        }else
        if(k == 'g'){
            std::cout << "move 0.5m left" << std::endl;
            assert(d.moveBy(0, -0.5f, 0, 0));
        }else
        if(k == 's'){
            std::cout << "STOP" << std::endl;
            d.modifyYaw(0);
            d.modifyRoll(0);
            d.modifyPitch(0);
            d.modifyAltitude(0);
            assert(d.moveBy(0, 0, 0, 0));
        }

    }

    d.land();

    return 0;
}
