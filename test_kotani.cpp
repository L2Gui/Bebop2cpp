//
// Created by ardrone on 17/08/17.
//
#include <stdio.h>
#include <iostream>
#include "Drone.h"
#include <math.h>

using namespace std;

int main(int argc, char *argv){
    int q=10;
    printf("hello1- %d\n", q);
    std::cout << "hello2- " <<  q << std::endl;
    cout << "hello3- " <<  q << endl;

    Drone d;

    bool check;
    check = d.connect();
    if (check == true) {
        
        while(false == d.isRunning()){
            sleep(1);
        }
/* //TAKEOFF
        d.blockingTakeOff();
        std::cout << "Success  Takeoff" << std::endl;
        d.blockingLand();
        std::cout << "Success  Landing" << std::endl;
 */
/* //TAKEOFF MOVE LAND
        d.blockingTakeOff();
        std::cout << "Success  Takeoff" << std::endl;
        d.moveBy(1,0,0,0);
        std::cout << "Success  Move" << std::endl;
        sleep(3);
        d.blockingLand();
        std::cout << "Success  Landing" << std::endl;
*/
/* //FLATTRIM TAKEOFF ROTATE LAND
        d.blockingFlatTrim();
        std::cout << "Success  Flat Trim" << std::endl;
        d.blockingTakeOff();
        std::cout << "Success  Takeoff" << std::endl;
        d.moveBy(0,0,0,M_PI/2.0);
        std::cout << "Success  Turn" << std::endl;
        sleep(3);
        d.blockingLand();
        std::cout << "Success  Landing" << std::endl;
*/
        d.blockingFlatTrim();
        std::cout << "Success  Flat Trim" << std::endl;
        d.blockingStartStreaming();
        d.blockingInitCam();

        ///////////
        bool proceed = true;
   //     cv::Mat frame(300, 400, 0);
        //cv::Mat frame = d.retrieveLastFrame();

        while(proceed) {
            cv::Mat frame = d.retrieveLastFrame();
            if(frame.empty() == true){
                continue;
            }
            cv::imshow("tuto1", frame);

            char k = (char) cv::waitKey(10);
            switch (k) {
                case 'q':
                    std::cout << "EMERGENCY BUTTON" << std::endl;
                    proceed = false;
                    break;
            }
        }
        ////////////////

        d.stopStreaming();
    }
    else {
        return 1;
    }

    return 0;
}