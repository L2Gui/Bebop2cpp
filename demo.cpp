#include <vector>
#include "Drone.h"

#include "streaming.h"


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

int main(){

    cv::Point3d wishedPosition(0, -20, 1500);

    Drone d;

    assert(d.isValid());

    assert(d.connect());


    while(!d.isRunning()){
        sleep(1);
    }


    std::cout << "EVERYTHING IS OK" << std::endl;


    //d.takeOff();
    //d.land();
/*
    assert(d.takeOff());

    sleep(5);

    assert(d.modifyPitch(15));
    sleep(2);

    assert(d.modifyPitch(0));
    sleep(5);

    assert(d.land());
*/

    assert(d.startStreaming());

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

    //std::cout << model_pts << std::endl;
    /*
    std::vector<cv::Point3d> model_pts;
    for(int x = -(ref_pt%chess_x); x < chess_x - ref_pt%chess_x; ++x){
        for(int y = -(ref_pt/chess_x); y < chess_y - (ref_pt/chess_x); ++y){
            model_pts.push_back(cv::Point3d(x, y, 0.));
        }
    }+
    */
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
                for(int i = 0; i < corners.size(); ++i){
                    cv::circle(frame, corners[i], 5, cv::Scalar(0, 0, 255));
                }
                cv::circle(frame, corners[ref_pt], 10, cv::Scalar(0, 255, 0));


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
                dist.x = -camPos.at<double>(0,0);
                dist.y = camPos.at<double>(1,0);
                dist.z = camPos.at<double>(2,0);

                distFromWished = dist - wishedPosition;


                cv::putText(frame, "Dist from chessboard", cv::Point(10,30), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2, 8);
                cv::putText(frame, std::to_string((int)dist.x), cv::Point(10,80), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,0,0), 2, 8);
                cv::putText(frame, std::to_string((int)dist.y), cv::Point(10,130), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,255,0), 2, 8);
                cv::putText(frame, std::to_string((int)dist.z), cv::Point(10,180), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,255), 2, 8);



                cv::putText(frame, "Dist from wished", cv::Point(10,230), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,0), 2, 8);
                cv::putText(frame, std::to_string((int)distFromWished.x), cv::Point(10,280), cv::QT_FONT_NORMAL, 1, cv::Scalar(255,0,0), 2, 8);
                cv::putText(frame, std::to_string((int)distFromWished.y), cv::Point(10,330), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,255,0), 2, 8);
                cv::putText(frame, std::to_string((int)distFromWished.z), cv::Point(10,380), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,255), 2, 8);


                cv::putText(frame, std::to_string(d.getBatteryLvl()), cv::Point(800,30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255,0,255), 3, 8);

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

                d.modifyRoll(askedRoll);
                d.modifyAltitude(askedAlt);
                d.modifyPitch(askedPitch);

                first_time = false;
            }else{
                cv::putText(frame, "STOP EVERYTHIG, NO CHESSBOARD !", cv::Point(10,300), cv::QT_FONT_NORMAL, 1, cv::Scalar(0,0,255), 3, 8);

                d.modifyRoll(0);
                d.modifyAltitude(0);
                d.modifyPitch(0);

                first_time = true;
            }
            /**
             * *********************************************************************************************************
             */
            cv::imshow("frame", frame);
        }
        char k = (char)cv::waitKey(10);
        if(k == 'l') {
            break;
        }if(k == 'q') {
            d.emergency();
            break;
        }
    }

    d.land();

    return 0;
}
