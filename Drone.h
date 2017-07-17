#ifndef BEBOP_DRONE_H
#define BEBOP_DRONE_H

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARNetwork/ARNetwork.h>
#include <libARController/ARCONTROLLER_Frame.h>
}

#include <atomic>
#include <iostream>


#include <opencv/cv.hpp>

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"
// CHANGE TO THE NAME OF THE BEBOP
#define TAG "BEBOP-CONTROLLER"

#define BEBOP_DEFAULT_IP_ADDRESS "192.168.42.1"
#define BEBOP_DEFAULT_DISCOVERY_PORT 44444
#define BEBOP_DEFAULT_C2D_PORT 54321 // should be read from Json
#define BEBOP_DEFAULT_D2C_PORT 43210

class Drone {

public:
    Drone(const std::string& ipAddress = BEBOP_DEFAULT_IP_ADDRESS,
          unsigned int discoveryPort = BEBOP_DEFAULT_DISCOVERY_PORT,
          unsigned int c2dPort = BEBOP_DEFAULT_C2D_PORT,
          unsigned int d2cPort = BEBOP_DEFAULT_D2C_PORT);
    ~Drone();
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
    bool isStreaming();
    bool errorStream();

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

    bool startStreamingME();
    //bool startStreamingEXPL();


    std::string getVideoPath();

protected:
    static void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *drone);
    static void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *drone);


    void cmdSensorStateListChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdFlyingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary);
    void cmdStreamingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

    static eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void *customData);

private:
    /// Utility functions
    bool ardiscoveryConnect();

    /*
    bool startNetwork();
    void stopNetwork();

    static eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *drone, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
     */
    static eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *drone);
    static eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *drone);

    //static void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *drone);

    static eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *customData);
    /// Attributes
    std::string _ip;
    int _discoveryPort;
    std::atomic<bool> _isConnected;
    std::atomic<bool> _isValid;

    char _fifo_dir[sizeof(FIFO_DIR_PATTERN)] = FIFO_DIR_PATTERN;
    char _fifo_name[128] = "";
    //TODO _
    FILE *videoOut = NULL;
    int frameNb = 0;
    ARSAL_Sem_t _stateSem;
    ARCONTROLLER_Device_t *_deviceController;

    /// Video
    int _d2cPort;
    int _c2dPort;
    ARNETWORKAL_Manager_t *_alManager;
    ARNETWORK_Manager_t *_netManager;
    ARSAL_Thread_t _rxThread;
    ARSAL_Thread_t _txThread;
    ARSAL_Thread_t _videoTxThread;
    ARSAL_Thread_t _videoRxThread;

    //eARCONTROLLER_ERROR error;
    eARCONTROLLER_DEVICE_STATE _deviceState;
    std::atomic<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE> _flyingState;
    std::atomic<eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED> _streamingState;

};


#endif //BEBOP_DRONE_H
