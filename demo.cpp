#include "Drone.h"

int main(){
    Drone d;

    assert(d.isValid());

    assert(d.connect());

    while(!d.isRunning());

    assert(d.takeOff());

    sleep(1);

    assert(d.emergency());
}