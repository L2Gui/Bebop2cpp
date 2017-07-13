#ifndef BEBOP_DRONE_H
#define BEBOP_DRONE_H

#include <atomic>
#include <iostream>

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"
// CHANGE TO THE NAME OF THE BEBOP
#define TAG "BEBOP-CONTROLLER"

#define BEBOP_DEFAULT_IP_ADDRESS "192.168.42.1"
#define BEBOP_DEFAULT_DISCOVERY_PORT 44444

class Drone {

    public:
    Drone(const std::string& ipAddress = BEBOP_DEFAULT_IP_ADDRESS, unsigned int discoveryPort = BEBOP_DEFAULT_DISCOVERY_PORT);
    /**
     * Return true if connexion is established, false otherwise
     */
    bool connect();
    bool isConnected();
    bool isValid();
    bool isStopped();
    bool isStarting();
    bool isRunning();
    bool isPaused();
    bool isStopping();

    /// COMMANDS
    bool takeOff();
    bool land();
    bool emergency();
    /**
     * Modify the altitude of the drone, adding <value> to the current one.
     * @param value the value to add in [TODO find unit]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyAltitude(int8_t value);
    /**
     * Modify the yaw of the drone, adding <value> to the current one.
     * A positive value should turn the head of the drone towards the right
     * A Negative value should turn the head of the drone towards the left
     * @param value the value to add in [TODO find unit]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyYaw(int8_t value);
    /**
     * Modify the pitch of the drone, adding <value> to the current one.
     * A positive value should move the drone forward
     * A Negative value should move the drone backward
     * @param value the value to add in [TODO find unit]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyPitch(int8_t value);
    /**
     * Modify the roll of the drone, adding <value> to the current one.
     * A positive value should roll the drone on the right
     * A Negative value should roll the drone on the left
     * @param value the value to add in [TODO find unit]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyRoll(int8_t value);

    protected:
    static void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *drone);
    static void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *drone);


    void cmdSensorStateListChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdFlyingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary);

    private:
    std::atomic<bool> _isConnected;
    std::atomic<bool> _isValid;

    private:

    //TODO _
    char _fifo_dir[sizeof(FIFO_DIR_PATTERN)] = FIFO_DIR_PATTERN;
    char _fifo_name[128] = "";
    ARSAL_Sem_t _stateSem;
    ARCONTROLLER_Device_t *_deviceController;
    //eARCONTROLLER_ERROR error;
    eARCONTROLLER_DEVICE_STATE _deviceState;
    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE _flyingState;

};


#endif //BEBOP_DRONE_H
