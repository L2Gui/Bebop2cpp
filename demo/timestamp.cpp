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
#define CALIBRATION_FILE            "res/calib_bd2.xml"
#define HULLPROTECTIONON            true

/*
 * MAIN
 */
int main(){


    /// ***************************************************************************** CONNECT TO AND CONFIGURE THE DRONE
    Drone d;
    assert(d.connect());

    // Waiting for the drone to be ready
    while(!d.isRunning()){ sleep(1); }

    std::cout << "INITIALISATION IS OK" << std::endl;

    // Flat trim at start. The drone MUST be on the ground at that time
    if(d.blockingFlatTrim()) {
        std::cout << "FLAT TRIM IS OK" << std::endl;
    }else{
        std::cerr << "FLAT TRIM NOT OK" << std::endl;
        return 1;
    }

    assert(d.setHullPresence(HULLPROTECTIONON));

    assert(d.setVideoAutorecord(false));
    //assert(d.stopStreaming());

    assert(d.useFullNavdata());
    sleep(1);
    assert(d.isUsingFullNavdata());

    Gnuplot gp;
    gp << "set title \"Sent timestamp\"\n";
    std::deque<double> uptime;
    int64_t tmp_up;
    int64_t prev_tmp_up;
    double delta_time_up;


    Gnuplot gp2;
    gp2 << "set title \"Received timestamp\"\n";
    std::deque<double> rt;
    int64_t tmp_rt;
    int64_t prev_tmp_rt;
    double delta_time_rt;

    while(true)
    {
        if (uptime.size() > 150) {
            uptime.pop_front();
            rt.pop_front();
        }

        prev_tmp_up = tmp_up;
        prev_tmp_rt = tmp_rt;
        d.getFullNavdata()->lock();
        tmp_up = d._navdata->get_sent_drone_uptime();
        tmp_rt = d._navdata->get_received_time_computer();
        d._navdata->release();

        delta_time_rt = tmp_rt - prev_tmp_rt;
        delta_time_up = tmp_up - prev_tmp_up;

        std::cout << prev_tmp_rt << " -> " << tmp_rt << " = " << delta_time_rt << std::endl;
        std::cout << prev_tmp_up << " -> " << tmp_up << " = " << delta_time_rt << std::endl;

        uptime.push_back(delta_time_up);
        rt.push_back(delta_time_rt);
        gp << "plot '-' binary" << gp.binFmt1d(uptime, "array") << "with lines title \"ST\"\n";

        gp.sendBinary1d(uptime);

        gp.flush();


        gp2 << "plot '-' binary" << gp2.binFmt1d(rt, "array") << "with lines title \"RT\"\n";

        gp2.sendBinary1d(rt);

        gp2.flush();

        // 100Hz
        //sleep(1);
        usleep(25 * 1000);
    }

    return 0;
}
