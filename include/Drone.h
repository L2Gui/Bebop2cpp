#ifndef BEBOP_DRONE_H
#define BEBOP_DRONE_H

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARNetwork/ARNetwork.h>
#include <libARController/ARCONTROLLER_Frame.h>
#include <libARController/ARCONTROLLER_Dictionary.h>
#include <libARController/ARCONTROLLER_Frame.h>
}

#include <atomic>
#include <iostream>

#include <opencv/cv.hpp>
#include <fullnavdata.h>

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"

// TODO CHANGE TO THE NAME OR THE ADDRESS OF THE BEBOP
#define TAG "BEBOP-CONTROLLER"

#define BEBOP_DEFAULT_IP_ADDRESS "192.168.42.1"
#define BEBOP_DEFAULT_DISCOVERY_PORT 44444
#define BEBOP_DEFAULT_C2D_PORT 54321 // should be read from Json
#define BEBOP_DEFAULT_D2C_PORT 43210

/**
 * DRONE RELEVANT DOCUMENTATION
 * http://developer.parrot.com/docs/reference/all
 * http://developer.parrot.com/docs/reference/bebop
 */
class Drone {

public:
    Drone(const std::string& ipAddress = BEBOP_DEFAULT_IP_ADDRESS,
          unsigned int discoveryPort = BEBOP_DEFAULT_DISCOVERY_PORT,
          unsigned int c2dPort = BEBOP_DEFAULT_C2D_PORT,
          unsigned int d2cPort = BEBOP_DEFAULT_D2C_PORT);
    ~Drone();

    /// ************************************************************************************************* GENERAL STATUS
    /**
     * Connect to the drone
     * @return true if connexion is established, false otherwise
     */
    bool connect();
    /**
     * Check if the drone is connected
     * @return true if the drone is connected, false otherwise
     */
    bool isConnected();
    /**
     * Check if the drone is valid. Call that method after instantiation to make sure everything went well
     * @return true if the drone is valid, false otherwise
     */
    bool isValid();
    /**
     * Check if the drone last status was stopped
     * @return true or false
     */
    bool isStopped();
    /**
     * Check if the drone last status was starting (not able to process command yet)
     * @return true or false
     */
    bool isStarting();
    /**
     * Check if the drone last status was running (can process command)
     * @return true or false
     */
    bool isRunning();
    /**
     * Check if the drone last status was paused TODO check what that state actually is
     * @return true or false
     */
    bool isPaused();
    /**
     * Check if the drone last status was stopping (not able to process command)
     * @return true or false
     */
    bool isStopping();
    /**
     * Get the battery level in percentage
     * @return the last battery level the drone sent
     */
    int getBatteryLvl();

    /// ************************************************************************************************** FLYING STATUS
    /**
     * Check if the drone is in flying state
     * @return true or false
     */
    bool isFlying();
    /**
     * Check if the drone is in flying hovering
     * @return true or false
     */
    bool isHovering();
    /**
     * Check if the drone is in landed state
     * @return true or false
     */
    bool isLanded();


    /// *********************************************************************************************** STREAMING STATUS
    /**
     * Check if the drone is curently streaming
     * @return true or false
     */
    bool isStreaming();
    /**
     * Check if the drone encountered an issue while streaming (can be used to determine if the drone is simply not
     * streaming or if it had an issue while streaming)
     * @return true or false
     */
    bool errorStream();


    /// ********************************************************************************** DRONE LIMITS & CONFIGURATIONS

    /**
     * Get the Ip address of the drone
     * @return the ip address
     */
    std::string getIpAddress();
    /**
     * Set the max altitude the drone can reach
     * @param value in meters
     * @return true if the command was well sent, false otherwise
     */
    bool setMaxAltitude(float value);
    /**
     * Get the max altitude the drone can reach
     * @return the value, in meters
     */
    float getMaxAltitude();


    /**
     * Set the max tilt value
     * @param value in degrees
     * @return true if the command was well sent, false otherwise
     */
    bool setMaxTilt(float value);
    /**
     * Get the max tilt value
     * @return the value, in degrees
     */
    float getMaxTilt();

    /**
     * Set the max vertical speed
     * @param value in m/s
     * @return true if the command was well sent, false otherwise
     */
    bool setMaxVerticalSpeed(float value);
    /**
     * Get the max vertical speed value
     * @return the value, in m/s
     */
    float getMaxVerticalSpeed();

    /**
     * Set the max horizontal speed
     * @param value in m/s
     * @return true if the command was well sent, false otherwise
     */
    bool setMaxHorizontalSpeed(float value);

    /**
     * Set the max rotation speed
     * @param value in degrees/s
     * @return true if the command was well sent, false otherwise
     */
    bool setMaxRotationSpeed(float value);
    /**
     * Get the max rotation speed value
     * @return the value, in degrees/s
     */
    float getMaxRotationSpeed();

    /**
     * Set the presence of the hull protection. Helps with the onboard computations
     * @param isPresent
     * @return true if the command was well sent, false otherwise
     */
    bool setHullPresence(bool isPresent);

    /**
     * Returns the current knowledge regarding the hull protection of the drone
     * @return true if the drone thinks the hull protection is present, false otherwise
     */
    bool isHullProtectionOn();

    /**
     * Execute a flat trim if the drone is in landed state.
     * @warning the function is blocking, it will only returns once the drone executed the flat trim of if the drone is
     * not in landing state.
     * @return true if a flat trim was successfully executed. False otherwise
     */
    bool blockingFlatTrim();

    /// ************************************************************************************************ NAVDATA GETTERS

    /**
     * Start using full navdata instead of Parrot 5Hz and limited navdata
     */
    bool useFullNavdata();

    /**
     * Returns true if full navdata are used for that drone, false otherwise
     * @return true or false
     */
    bool isUsingFullNavdata();
    /**
     * Get the last roll value received from the drone (expect the value to change 5 times per second)
     * @return Roll value in degrees
     */
    float getRoll();
    /**
     * Get the last pitch value received from the drone (expect the value to change 5 times per second)
     * @return Pitch value in degrees
     */
    float getPitch();
    /**
     * Get the last yaw value received from the drone (expect the value to change 5 times per second)
     * @return Yaw value in degrees
     */
    float getYaw();

    /**
     * Get the last values of the gyroscope received from the drone (expect the value to change 5 times per second)
     * @return a cv::Point3f as (Roll, Pitch, Yaw), all in degrees
     */
    cv::Point3f getGyro();
    /**
     * Get the last speedX value received from the drone (expect the value to change 5 times per second)
     * @return SpeedX value in m/s
     */
    float getSpeedX();
    /**
     * Get the last speedY value received from the drone (expect the value to change 5 times per second)
     * @return SpeedY value in m/s
     */
    float getSpeedY();
    /**
     * Get the last speedZ value received from the drone (expect the value to change 5 times per second)
     * @return SpeedZ value in m/s
     */
    float getSpeedZ();

    /**
     * Get the last values of the accelerometer received from the drone (expect the value to change 5 times per second)
     * @return a cv::Point3f as (speedX, speedY, speedZ), all in m/s
     */
    cv::Point3f getAccelero();

    /// ******************************************************************************************************* COMMANDS
    /**
     * Give to the drone the order to take off.
     * @return true if the order was well sent, false otherwise
     */
    bool takeOff();
    /**
     * Give to the drone the order to take off.
     * @warning this function is blocking, it will only returns when the drone actually took of
     * @return true if the order was well sent and the drone actually took off, false otherwise
     */
    bool blockingTakeOff();
    /**
     * Give to the drone the order to land.
     * @return true if the order was well sent, false otherwise
     */
    bool land();

    /**
     * Give to the drone the order to land.
     * @warning this function is blocking, it will only returns when the drone actually landed
     * @return true if the order was well sent and the drone actually landed, false otherwise
     */
    bool blockingLand();
    /**
     * Give to the drone the order to shut down all motor functions immediatly.
     * @return true if the order was well sent, false otherwise
     */
    bool emergency();
    /**
     * Modify the altitude of the drone, adding <value> to the current one.
     * @param value the value to apply as signed percentage of the max vertical speed setting, in range [-100, 100]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyAltitude(int8_t value);
    /**
     * Modify the yaw of the drone, adding <value> to the current one.
     * A positive value should turn the head of the drone towards the right
     * A Negative value should turn the head of the drone towards the left
     * @param value the value to apply as signed percentage of the max yaw rotation speed setting, in range [-100, 100]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyYaw(int8_t value);
    /**
     * Modify the pitch of the drone, adding <value> to the current one.
     * A positive value should move the drone forward
     * A Negative value should move the drone backward
     * @param value the value to apply as signed percentage of the max pitch/roll setting, in range [-100, 100]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyPitch(int8_t value);
    /**
     * Modify the roll of the drone, adding <value> to the current one.
     * A positive value should roll the drone on the right
     * A Negative value should roll the drone on the left
     * @param value the value to apply as signed percentage of the max pitch/roll setting, in range [-100, 100]
     * @return true if command was successfully send, false otherwise
     */
    bool modifyRoll(int8_t value);

    /**
     * Move the drone to a relative position and rotate heading by a given angle.
     * Moves are relative to the current drone orientation, (drone’s reference).
     * Also note that the given rotation will not modify the move (i.e. moves are always rectilinear).
     * @param dX wanted displacement on the front axis in meters
     * @param dY wanted displacement on the right axis in meters
     * @param dZ wanted displacement on the down axis in meters
     * @param dPsi wanted rotation of the head of the drone in radians
     * @return true if the command was well sent, false otherwise
     */
    bool moveBy(float dX, float dY, float dZ, float dPsi);

    /// ************************************************************************************************** CAMERA STUFFS

    /**
     * Rotate the camera
     * @param tilt value in degrees
     * @param pan value in degrees
     * @return true if the command was well sent, false otherwise
     */
    bool rotateCamera(float tilt, float pan);
    /**
     * Send the drone the order to start streaming
     * @warning the function is blocking, it will only returns once the drone actually started the stream or encountered
     * an issue.
     * @return true if the straming is started, false otherwise
     */
    bool blockingStartStreaming();

    /**
     * Initialise the camera computer-side (please call blockingStartStreaming first or the function won't return)
     * @warning blocking function
     */
    void blockingInitCam();

    /**
     * Recover last frame or an empty frame if latest frame has already been retrieved
     * @return cv::Mat
     */
    cv::Mat retrieveLastFrame();
    /**
     * Stops streaming
     * @warning the function is blocking, it will only returns once the drone actually stopped the stream or encountered
     * an issue.
     * @return true if the straming is stropped, false otherwise
     */
    bool stopStreaming();

    //bool startStreamingEXPL();

    /**
     * Get the ugly file path where the streaming is outputed
     * @return path of the video (H264 codec format)
     */
    std::string getVideoPath();

    /**
     * Set the video autorecord mode
     * If true, the video will be recorded automatically when the drone takes off and stopped after landing
     * @warning if using autorecord with the blackbox navdata, memory might be full in no time
     * @param autoRecord
     * @param storageId storageID on the drone, default is 0
     * @return true if command was successfully send, false otherwise
     */
    bool setVideoAutorecord(bool autoRecord, uint8_t storageId = 0);

    /**
     * Get the video autorecord status
     * @return true if the autorecord mode is set, false otherwise
     */
    bool isVideoAutorecordOn();

    /**
     * Get the video autorecord storage id (relevent only if autorecord mode is set)
     * @return storageId
     */
    uint8_t getAutorecordStorageId();

    fullnavdata _navdata;
protected:
    /// *********************************************************************************** RETRIEVE DATA FROM THE DRONE
    static void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *drone);
    static void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey,
                                 ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                 void *drone);

    void cmdSensorStateListChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdFlyingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary);
    void cmdStreamingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdPositionChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary);
    void cmdSpeedChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary);
    void cmdAttitudeChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
    void cmdMagnetoCalibrationNeedChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

    void cmdRelativeMovementChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

    void cmdVideoResolutionChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

    void cmdHullProtectionPresenceChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

    void cmdAutorecordModeChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

    void cmdFlatTreamChangedRcv();

    /// ********************************************************************************************* DIRTY CAMERA STUFF
    static eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void *customData);
    static eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *customData);

private:
    /// ********************************************************************************* PRIVATE FUNCTIONS & ATTRIBUTES
    bool ardiscoveryConnect();

    /// *********************************************************************************************** JSON CONFIG ****
    static eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx,
                                                                       uint32_t *dataTxSize,
                                                                       void *drone);
    static eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx,
                                                                          uint32_t dataRxSize,
                                                                          char *ip,
                                                                          void *drone);

    //static void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *drone);

    /// ************************************************************************************************ ATTRIBUTES ****
    std::string _ip;
    int _discoveryPort;
    std::atomic<bool> _isConnected;
    std::atomic<bool> _isValid;
    std::atomic<int> _batteryLvl;

    std::atomic<float> _speedX;
    std::atomic<float> _speedY;
    std::atomic<float> _speedZ;
    std::atomic_flag _lockGyro;

    std::atomic<double> _latitude;
    std::atomic<double> _longitude;
    std::atomic<double> _altitude;
    std::atomic_flag _lockAccelero;

    std::atomic<float> _roll;
    std::atomic<float> _pitch;
    std::atomic<float> _yaw;

    std::atomic<bool> _trimLock;

    std::atomic<bool> _hullProtectionPresence;

    cv::VideoCapture _camera;

    int frameNb = 0;
    ARSAL_Sem_t _stateSem;
    ARCONTROLLER_Device_t *_deviceController;
    ARCOMMANDS_Decoder_t *_commandsDecoder;

    /// ********************************************************************************** VIDEO RELATED ATTRIBUTES ****
    char _file_dir_name[sizeof(FIFO_DIR_PATTERN)] = FIFO_DIR_PATTERN;
    char _file_name[128] = "";

    FILE* _videoOut = NULL;
    int _d2cPort;
    int _c2dPort;

    //fullnavdata _navdata;
    std::atomic<bool> _usingFullNavdata;

    std::atomic<bool> _autorecordEnabled;
    std::atomic<uint8_t> _autorecordStorageId;

    /// ********************************************************************************* STATES RELATED ATTRIBUTES ****
    //eARCONTROLLER_ERROR error;
    eARCONTROLLER_DEVICE_STATE _deviceState;
    std::atomic<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE> _flyingState;
    std::atomic<eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED> _streamingState;

};


#endif //BEBOP_DRONE_H
