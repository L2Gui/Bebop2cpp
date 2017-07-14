#include "Drone.h"

int main(){

    Drone d;

    assert(d.isValid());

    assert(d.connect());

    while(!d.isRunning()){
        sleep(1);
    }
/*
    assert(d.takeOff());

    sleep(1);

    assert(d.emergency());
*/

    assert(d.startStreaming());

    //cv::VideoCapture cap("./bebop.sdp");

    while(1)
        sleep(1);
    /*
    assert(cap.isOpened());

    while(true)
    {
        cv::Mat frame;
        cap >> frame;
        cv::imshow("frame", frame);
        cv::waitKey(1);
    }
     */
    return 1;
}