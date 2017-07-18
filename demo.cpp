#include "Drone.h"

#include "streaming.h"

int main(){


    Drone d;

    assert(d.isValid());

    assert(d.connect());


    while(!d.isRunning()){
        sleep(1);
    }



    assert(d.startStreamingME());



    std::cout << "EVERYTHING IS OK" << std::endl;


/*
    initStream();
*/

    cv::VideoCapture cap(d.getVideoPath());
    while(!cap.isOpened()){
        cap = cv::VideoCapture(d.getVideoPath());
    }
/*

    assert(d.takeOff());

    sleep(1);

    assert(d.emergency());
*/
    cv::Mat tmp;
    cv::Mat frame;
    while(true)
    {
        cap >> tmp;
        if(tmp.data != NULL) {
            /**
             * getting the last image
             */
            while (1) {
                if(tmp.data != NULL) {
                    tmp.copyTo(frame);
                }else{
                    break;
                }
                cap >> tmp;
            }
            cv::imshow("frame", frame);
        }

        cv::waitKey(1);
    }

    while(1)
        sleep(1);

    return 0;
}