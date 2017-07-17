#include "Drone.h"

#include "streaming.h"

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

    assert(d.startStreamingME());



    std::cout << "EVERYTHING IS OK" << std::endl;


/*
    initStream();
*/

    cv::VideoCapture cap(d.getVideoPath());
    while(!cap.isOpened()){
        cap = cv::VideoCapture(d.getVideoPath());
    }

    while(true)
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.data != NULL){
            cv::imshow("frame", frame);
        }

        cv::waitKey(1);
    }

    while(1)
        sleep(1);

    return 0;
}