/**
 * Includes
 */

#include "Drone.h"

#include <errno.h>
#include <vector>

#include "Fullnavdata.h"

/**
 * Some more define to clean out
 */
#define BD_CLIENT_STREAM_PORT 55004
#define BD_CLIENT_CONTROL_PORT 55005

/**
 * Methods
 */

/// ************************************************************************************************************* PUBLIC
Drone::Drone(const std::string& ipAddress, unsigned int discoveryPort, unsigned int c2dPort, unsigned int d2cPort):
        _deviceController(NULL),
        _deviceState(ARCONTROLLER_DEVICE_STATE_MAX),
        _ip(ipAddress),
        _discoveryPort(discoveryPort),
        _c2dPort(c2dPort),
        _d2cPort(d2cPort),
        _usingFullNavdata(false),
        _navdata(new Fullnavdata()),
        _spinlock_camera(ATOMIC_FLAG_INIT)
{
    bool failed = false;
    ARDISCOVERY_Device_t *device = NULL;
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    _lockGyro.clear();
    _lockAccelero.clear();


    if (mkdtemp(_file_dir_name) == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkdtemp failed.");
        _isValid = false;
        return;
    }
    printf("DIRNAME = %s\n", _file_dir_name);
    snprintf(_file_name, sizeof(_file_name), "%s/%s", _file_dir_name, FIFO_NAME);

    _videoOut = fopen(_file_name, "wb+");
    if(_videoOut == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
        _isValid = false;
        return;
    }

    ARSAL_Sem_Init (&(_stateSem), 0, 0);

    /**
     * Exchanging JSON information with the drone
     */
    // TODO actually, there is no usage of it right now and, as is, it makes multidrone not working.
    //this->ardiscoveryConnect();


    /**
     *  Getting Network controller
     */
    if (!failed)
    {
        /* start */
        // No use right now and probably ever since libarcontroller takes care of networking stuffs
        //failed = this->startNetwork();
    }
    /**
     * Creating device controller
     */
    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
    device = ARDISCOVERY_Device_New (&errorDiscovery);
    if (errorDiscovery == ARDISCOVERY_OK) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
        errorDiscovery = ARDISCOVERY_Device_InitWifi (device,
                                                      ARDISCOVERY_PRODUCT_BEBOP_2,
                                                      "bebop2",
                                                      _ip.c_str(),
                                                      discoveryPort
        );

        if (errorDiscovery != ARDISCOVERY_OK)
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
        }
    }

    // create a device controller
    if (!failed)
    {
        _deviceController = ARCONTROLLER_Device_New (device, &error);

        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
            failed = 1;
        }

        if (!failed)
        {
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
            ARDISCOVERY_Device_Delete (&device);

        }
    }

    // add the state change callback to be informed when the device controller starts, stops...
    if (!failed)
    {
        error = ARCONTROLLER_Device_AddStateChangedCallback (_deviceController, stateChanged, this);

        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
            failed = 1;
        }
    }

    // add the command received callback to be informed when a command has been received from the device
    if (!failed)
    {
        error = ARCONTROLLER_Device_AddCommandReceivedCallback (_deviceController, commandReceived, this);

        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add callback failed.");
            failed = 1;
        }
    }
/*  // NEEDED FOR OLDER SDK VERSIONS
    if(!failed){
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set arcommands decoder ... ");
        eARCOMMANDS_DECODER_ERROR decError = ARCOMMANDS_DECODER_OK;
        _commandsDecoder = ARCOMMANDS_Decoder_NewDecoder(&decError);

        //ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCb(_deviceController.)
    }
*/


    // add the frame received callback to be informed when a streaming frame has been received from the device
    if (!failed)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
        error = ARCONTROLLER_Device_SetVideoStreamCallbacks (_deviceController,
                                                             decoderConfigCallback,
                                                             didReceiveFrameCallback,
                                                             NULL,
                                                             this
        );

        if (error != ARCONTROLLER_OK)
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error: %s", ARCONTROLLER_Error_ToString(error));
        }
    }
    _isValid = !failed;
    _isConnected = false;
}


Drone::~Drone() {
    if(_deviceController != NULL and _isConnected and !isStopped()) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

        eARCONTROLLER_ERROR error = ARCONTROLLER_Device_Stop(_deviceController);

        if (error == ARCONTROLLER_OK) {
            // wait state update update
            ARSAL_Sem_Wait(&(_stateSem));
        }
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
    ARCONTROLLER_Device_Delete (&_deviceController);

    ARSAL_Sem_Destroy (&(_stateSem));

    unlink(_file_name);
    rmdir(_file_dir_name);

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Drone successfully destroyed");

    if(_navdata != NULL){
        delete _navdata;
    }
}

bool Drone::connect()
{
    eARCONTROLLER_ERROR error;
    if (_isValid and !_isConnected)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
        error = ARCONTROLLER_Device_Start (_deviceController);

        if (error != ARCONTROLLER_OK)
        {
            _isConnected = false;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
        }else {
            _isConnected = true;
        }
    }

    if(_isConnected){
        ARSAL_Sem_Wait (&(_stateSem));

        _deviceState = ARCONTROLLER_Device_GetState (_deviceController, &error);

        if ((error != ARCONTROLLER_OK) || (_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING))
        {
            _isConnected = false;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", _deviceState);
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
        }
    }

    // desable autorecord
    //_deviceController->aRDrone3->sendPictureSettingsVideoAutorecordSelection(_deviceController->aRDrone3, (uint8_t)0, (uint8_t)0);

    // RESOLOUTION
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-Setting video resulution to 480p...");
    error = _deviceController->aRDrone3->sendPictureSettingsVideoResolutions(
            _deviceController->aRDrone3,
            ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEORESOLUTIONS_TYPE_REC1080_STREAM480
    );

    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error setting video resulution to 480p");
    }
    bool res = _isValid and _isConnected and !isStopped();
    return res;
}

/// *********************************************************************************************************** COMMANDS
bool Drone::takeOff() {
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingTakeOff(_deviceController->aRDrone3);

    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending takeoff command");
    }
    return error == ARCONTROLLER_OK;
}

bool Drone::blockingTakeOff(){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingTakeOff(_deviceController->aRDrone3);

    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending takeoff command");
    }
    if(error == ARCONTROLLER_OK){
        while(
                (_flyingState != ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING) &&
                (_flyingState != ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING));
        return true;
    }
    return false;

}
bool Drone::land(){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingLanding(_deviceController->aRDrone3);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending land command");
    }
    return error == ARCONTROLLER_OK;
}

bool Drone::blockingLand(){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingLanding(_deviceController->aRDrone3);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending land command");
    }
    if(error == ARCONTROLLER_OK){
        while(
                (_flyingState != ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED) &&
                (_flyingState != ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_EMERGENCY_LANDING));
        return true;
    }
    return false;
}
bool Drone::emergency(){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingEmergency(_deviceController->aRDrone3);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending emergency command");
    }
    return error == ARCONTROLLER_OK;
}
bool Drone::modifyAltitude(int8_t value){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->setPilotingPCMDGaz(_deviceController->aRDrone3, value);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending PCMDGaz command with %d", value);
    }
    return error == ARCONTROLLER_OK;
}
bool Drone::modifyYaw(int8_t value){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->setPilotingPCMDYaw(_deviceController->aRDrone3, value);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending PCMDYaw command with %d", value);
    }
    return error == ARCONTROLLER_OK;
}
bool Drone::modifyPitch(int8_t value){
    eARCONTROLLER_ERROR error1 = _deviceController->aRDrone3->setPilotingPCMDPitch(_deviceController->aRDrone3, value);
    if(error1 != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending PCMDPitch command with %d", value);
    }


    eARCONTROLLER_ERROR error2 = _deviceController->aRDrone3->setPilotingPCMDFlag(_deviceController->aRDrone3, 1);
    if(error2 != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending PCMDFlag command with 1");
    }
    return error1 == ARCONTROLLER_OK and error2 == ARCONTROLLER_OK;
}
bool Drone::modifyRoll(int8_t value){
    eARCONTROLLER_ERROR error1 = _deviceController->aRDrone3->setPilotingPCMDRoll(_deviceController->aRDrone3, value);
    if(error1 != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending PCMDRoll command with %d", value);
    }


    eARCONTROLLER_ERROR error2 = _deviceController->aRDrone3->setPilotingPCMDFlag(_deviceController->aRDrone3, 1);
    if(error2 != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending PCMDFlag command with 1");
    }
    return error1 == ARCONTROLLER_OK and error2 == ARCONTROLLER_OK;
}

bool Drone::moveBy(float dX, float dY, float dZ, float dPsi){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingMoveBy(
            _deviceController->aRDrone3,
            dX,
            dY,
            dZ,
            dPsi
    );
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending sendPilotingMoveBy command with %f, %f, %f, %f",
                    dX,
                    dY,
                    dZ,
                    dPsi
        );
    }
    return error == ARCONTROLLER_OK;
}

bool Drone::rotateCamera(float tilt, float pan){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendCameraOrientation(
            _deviceController->aRDrone3, (uint8_t) tilt, (uint8_t) pan);

    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending sendCameraOrientationV2 command with %f, %f",
                    tilt,
                    pan
        );
    }
    return error == ARCONTROLLER_OK;
}


bool Drone::blockingStartStreaming() {
    _deviceController->aRDrone3->sendMediaStreamingVideoEnable(_deviceController->aRDrone3, 1);

    while(!isStreaming() && !errorStream());
    return !errorStream();
}

bool Drone::stopStreaming() {
    _deviceController->aRDrone3->sendMediaStreamingVideoEnable(_deviceController->aRDrone3, 0);

    while(isStreaming() && !errorStream());

    return !errorStream();
}

bool Drone::blockingFlatTrim() {
    if(isLanded()) {
        _trimLock.store(true);
        _deviceController->aRDrone3->sendPilotingFlatTrim(_deviceController->aRDrone3);
        while (_trimLock);
        return true;
    }
    return false;
}

bool Drone:: setMaxAltitude(float value){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingSettingsMaxAltitude(
            _deviceController->aRDrone3,
            value);

    return error == ARCONTROLLER_OK;
}


bool Drone::setMaxVerticalSpeed(float value) {
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingSettingsSetAutonomousFlightMaxVerticalSpeed(
            _deviceController->aRDrone3,
            value
    );

    return error == ARCONTROLLER_OK;
}


bool Drone::setMaxHorizontalSpeed(float value) {
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingSettingsSetAutonomousFlightMaxHorizontalSpeed(
            _deviceController->aRDrone3,
            value
    );

    return error == ARCONTROLLER_OK;
}

bool Drone::setMaxRotationSpeed(float value) {
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingSettingsSetAutonomousFlightMaxRotationSpeed(
            _deviceController->aRDrone3,
            value
    );

    return error == ARCONTROLLER_OK;
}



/*
bool Drone::startStreamingEXPL()
{
    bool sentStatus = true;
    uint8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Streaming Begin");

    // Send Streaming begin command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(_netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, this, arnetworkCmdCallback, 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = false;
    }

    return sentStatus;
}
*/

/// ************************************************************************************************************ GETTERS
bool Drone::isConnected() const {
    return _isConnected;
}

bool Drone::isValid() const {
    return _isValid;
}

bool Drone::isStopped() const {
    return _deviceState == ARCONTROLLER_DEVICE_STATE_STOPPED;
}
bool Drone::isStarting() const {
    return _deviceState == ARCONTROLLER_DEVICE_STATE_STARTING;
}
bool Drone::isRunning() const {
    return _deviceState == ARCONTROLLER_DEVICE_STATE_RUNNING;
}
bool Drone::isPaused() const {
    return _deviceState == ARCONTROLLER_DEVICE_STATE_PAUSED;
}
bool Drone::isStopping() const {
    return _deviceState == ARCONTROLLER_DEVICE_STATE_STOPPING;
}

bool Drone::isFlying() const {
    return _flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING;
}
bool Drone::isHovering() const {
    return _flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING;
}
bool Drone::isLanded() const {
    return _flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED;
}

bool Drone::errorStream() const {
    return _streamingState == ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_ERROR;
}
bool Drone::isStreaming() const {
    return  _streamingState == ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_ENABLED;
}


int Drone::getBatteryLvl() const {
    return _batteryLvl;
}

const std::string Drone::getVideoPath() const {
    return _file_name;
}

float Drone::getRoll() const {
    return _roll;
}

float Drone::getPitch() const {
    return _pitch;
}

float Drone::getYaw() const {
    return _yaw;
}
cv::Point3f Drone::getGyro() {
    while(_lockGyro.test_and_set(std::memory_order_acquire)); // spinlock
    cv::Point3f res(_roll, _pitch, _yaw);
    _lockGyro.clear(std::memory_order_release);
    return res;
}

float Drone::getSpeedX() const {
    return _speedX;
}

float Drone::getSpeedY() const {
    return _speedY;
}

float Drone::getSpeedZ() const {
    return _speedZ;
}
cv::Point3f Drone::getAccelero() {
    while(_lockAccelero.test_and_set(std::memory_order_acquire)); // spinlock
    cv::Point3f res(_speedX, _speedY, _speedZ);
    _lockAccelero.clear(std::memory_order_release);
    return res;
}
/// ********************************************************************************************************** PROTECTED
/**
 * STATIC
 * @param commandKey
 * @param elementDictionary
 * @param drone (void*)(Drone d)
 */
void Drone::commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey,
                             ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                             void *drone)
{
    Drone* d = (Drone*)drone;
    if (d->_deviceController == NULL)
        return;

    if(elementDictionary == NULL)
        return;

    /***************************** SHOULD BE WRAPPED INSIDE PRIVATE METHODS */
    if ((commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED) && (elementDictionary != NULL))
    {
        ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
        ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
        HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
        if (element != NULL)
        {
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED_SOFTWARE, arg);
            if (arg != NULL)
            {
                char * software = arg->value.String;
                std::cout << "SOFTWARE " << software << std::endl;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED_HARDWARE, arg);
            if (arg != NULL)
            {
                char * hardware = arg->value.String;
                std::cout << "hardware " << hardware << std::endl;
            }
        }
    }
    // DEPRECATED BUT WORKS
    if ((commandKey == ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION) && (elementDictionary != NULL))
    {
        ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
        ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
        HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
        if (element != NULL)
        {
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION_TILT, arg);
            if (arg != NULL)
            {
                float tilt = arg->value.I8;
                //std::cout << "(deprecated) TILT " << tilt << std::endl;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION_PAN, arg);
            if (arg != NULL)
            {
                float pan = arg->value.I8;
                //std::cout << "(deprecated) PAN " << pan << std::endl;
            }
        }
    }
    // THE WAY TO DO IT... BUT DOES NOT WORK
    if ((commandKey == ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATIONV2) && (elementDictionary != NULL))
    {
        ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
        ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
        HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
        if (element != NULL)
        {
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATIONV2_TILT, arg);
            if (arg != NULL)
            {
                float tilt = arg->value.Float;
                std::cout << "TILT " << tilt << std::endl;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATIONV2_PAN, arg);
            if (arg != NULL)
            {
                float pan = arg->value.Float;
                std::cout << "PAN " << pan << std::endl;
            }
        }
    }

    if ((commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED) && (elementDictionary != NULL))
    {
        ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
        ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
        HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
        if (element != NULL)
        {
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_FOV, arg);
            if (arg != NULL)
            {
                float fov = arg->value.Float;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_PANMAX, arg);
            if (arg != NULL)
            {
                float panMax = arg->value.Float;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_PANMIN, arg);
            if (arg != NULL)
            {
                float panMin = arg->value.Float;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_TILTMAX, arg);
            if (arg != NULL)
            {
                float tiltMax = arg->value.Float;
                std::cout << "NEW TILT MAX" << tiltMax << std::endl;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_TILTMIN, arg);
            if (arg != NULL)
            {
                float tiltMin = arg->value.Float;
                std::cout << "NEW TILT MIN" << tiltMin << std::endl;
            }
        }
    }

    if ((commandKey == ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_DEFAULTCAMERAORIENTATIONV2) && (elementDictionary != NULL))
    {
        std::cout << "doot ****************************************" << std::endl;
        ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
        ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
        HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
        if (element != NULL)
        {
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_DEFAULTCAMERAORIENTATIONV2_TILT, arg);
            if (arg != NULL)
            {
                float tilt = arg->value.Float;
                std::cout << "CENTER TILT " << tilt << std::endl;
            }
            HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_DEFAULTCAMERAORIENTATIONV2_PAN, arg);
            if (arg != NULL)
            {
                float pan = arg->value.Float;
            }
        }
    }
    /************************* SHOULD BE WRAPPED INSIDE PRIVATE METHODS */

    // if the command received is a battery state changed
    switch(commandKey) {
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
            d->cmdBatteryStateChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED:
            d->cmdSensorStateListChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED:
            d->cmdFlyingStateChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED:
            d->cmdStreamingStateChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED:
            d->cmdPositionChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED:
            d->cmdSpeedChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED:
            d->cmdAttitudeChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_CALIBRATIONSTATE_MAGNETOCALIBRATIONREQUIREDSTATE:
            d->cmdMagnetoCalibrationNeedChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND:
            //std::cout << "OOOOOOOOOOOK" << std::endl;
            d->cmdRelativeMovementChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED:
            d->cmdVideoResolutionChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLATTRIMCHANGED:
            d->cmdFlatTreamChangedRcv();
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_HULLPROTECTIONCHANGED:
            d->cmdHullProtectionPresenceChanged(elementDictionary);
        case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEOAUTORECORDCHANGED:
            d->cmdAutorecordModeChanged(elementDictionary);
        default:
            break;
    }
}

void Drone::stateChanged (eARCONTROLLER_DEVICE_STATE newState,
                          eARCONTROLLER_ERROR error,
                          void *drone)
{
    Drone* d = (Drone*) drone;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);

    switch (newState)
    {
        case ARCONTROLLER_DEVICE_STATE_STOPPED:
            ARSAL_Sem_Post (&(d->_stateSem));
            //stop
            std::cout << "I STOPPED... DUNNO WHY." << std::endl;

            break;

        case ARCONTROLLER_DEVICE_STATE_RUNNING:
            ARSAL_Sem_Post (&(d->_stateSem));
            break;

        default:
            break;
    }

    d->_deviceState = newState;
}

void Drone::cmdSensorStateListChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary)
{
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *dictElement = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *dictTmp = NULL;

    eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME sensorName = ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_MAX;
    int sensorState = 0;

    // get the command received in the device controller
    HASH_ITER(hh, elementDictionary, dictElement, dictTmp) {
        // get the Name
        HASH_FIND_STR (dictElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME, arg);
        if (arg == NULL) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorName is NULL");
            continue;
        }

        sensorName = (eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME) arg->value.I32;

        // get the state
        HASH_FIND_STR (dictElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORSTATE, arg);
        if (arg == NULL) {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorState is NULL");
            continue;
        }

        sensorState = arg->value.U8;
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "sensorName %d ; sensorState: %d", sensorName, sensorState);
    }
}

void Drone::cmdStreamingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary)
{
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;

    // get the command received in the device controller
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        // get the value
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED, arg);

        if (arg != NULL)
        {
            _streamingState = (eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED) arg->value.I32;
        }
    }
}


void Drone::cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary)
{

    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;

    if (elementDictionary == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
        return;
    }

    // get the command received in the device controller
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);

    if (singleElement == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
        return;
    }

    // get the value
    HASH_FIND_STR (singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);

    if (arg == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
        return;
    }
    _batteryLvl.store((int)(arg->value.U8));
}

void Drone::cmdFlyingStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary)
{
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;

    // get the command received in the device controller
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        // get the value
        HASH_FIND_STR (element->arguments,
                       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE,
                       arg
        );

        if (arg != NULL)
        {
            _flyingState = (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE) arg->value.I32;
        }
    }
}

void Drone::cmdPositionChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_LATITUDE, arg);
        if (arg != NULL)
        {
            _latitude.store(arg->value.Double);
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_LONGITUDE, arg);
        if (arg != NULL)
        {
            _longitude.store(arg->value.Double);
            //std::cout << "LONGITUDE " << arg->value.Double << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_ALTITUDE, arg);
        if (arg != NULL)
        {
            _altitude.store(arg->value.Double);
            //std::cout << "ALTITUDE " << arg->value.Double << std::endl;
        }
    }
}
void Drone::cmdSpeedChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        while(_lockAccelero.test_and_set(std::memory_order_acquire));
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDX, arg);
        if (arg != NULL)
        {
            _speedX.store(arg->value.Float);
            //std::cout << "SPEED X " << arg->value.Double << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDY, arg);
        if (arg != NULL)
        {
            _speedY.store(arg->value.Float);
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDZ, arg);
        if (arg != NULL)
        {
            _speedZ.store(arg->value.Float);
        }
        _lockAccelero.clear(std::memory_order_release);
    }
}

void Drone::cmdAttitudeChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        while(_lockGyro.test_and_set(std::memory_order_acquire));
        //std::cout << "LOCK >" << std::endl;
        //NB: roll pitch yaw are all received at the same time
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL, arg);
        if (arg != NULL)
        {
            _roll.store(arg->value.Float);
            //std::cout << "roll " << arg->value.Float << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH, arg);
        if (arg != NULL)
        {
            _pitch.store(arg->value.Float);
            //std::cout << "pitch " << arg->value.Float << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_YAW, arg);
        if (arg != NULL)
        {
            _yaw.store(arg->value.Float);
            //std::cout << "yaw " << arg->value.Float << std::endl;
        }
        _lockGyro.clear(std::memory_order_release);
        //std::cout << "< /LOCK" << std::endl;
    }
}

void Drone::cmdMagnetoCalibrationNeedChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CALIBRATIONSTATE_MAGNETOCALIBRATIONREQUIREDSTATE_REQUIRED, arg);
        if (arg != NULL)
        {
            uint8_t required = arg->value.U8;
            if(required == 1){
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Calibration IS REQUIRED !");
            }else{
                ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Calibration is not required !");
            }
        }
    }
}

void Drone::cmdRelativeMovementChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DX, arg);
        if (arg != NULL)
        {
            float dX = arg->value.Float;
            //std::cout << "DRONE COMMAND MOVEBY dX: " << dX << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DY, arg);
        if (arg != NULL)
        {
            float dY = arg->value.Float;
            //std::cout << "DRONE COMMAND MOVEBY dY: " << dY << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DZ, arg);
        if (arg != NULL)
        {
            float dZ = arg->value.Float;
            //std::cout << "DRONE COMMAND MOVEBY dZ: " << dZ << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DPSI, arg);
        if (arg != NULL)
        {
            float dPsi = arg->value.Float;
            //std::cout << "DRONE COMMAND MOVEBY DPSI: " << dPsi << std::endl;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR, arg);
        if (arg != NULL)
        {
            eARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR error = eARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR(arg->value.I32);
            //std::cout << "DRONE COMMAND MOVEBY ERROR: " << error << std::endl;
        }
    }
}


void Drone::cmdVideoResolutionChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED_TYPE, arg);
        if (arg != NULL)
        {
            eARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED_TYPE type = eARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED_TYPE(arg->value.I32);
            switch(type) {
                case ARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED_TYPE_REC1080_STREAM480:
                    std::cout << "STREAM 480" << std::endl;
                    break;

                case ARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED_TYPE_REC720_STREAM720:
                    std::cout << "STREAM 720" << std::endl;
                    break;
                default:
                    std::cout << "SHOULD NOT HAPPEND" << std::endl;
            }

        }
    }
}
void Drone::cmdHullProtectionPresenceChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_HULLPROTECTIONCHANGED_PRESENT, arg);
        if (arg != NULL)
        {
            _hullProtectionPresence = (bool)arg->value.U8;
        }
    }
}

void Drone::cmdAutorecordModeChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary){
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
    HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
    if (element != NULL)
    {
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEOAUTORECORDCHANGED_ENABLED, arg);
        if (arg != NULL)
        {
            _autorecordEnabled = (bool)arg->value.U8;
        }
        HASH_FIND_STR (element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEOAUTORECORDCHANGED_MASS_STORAGE_ID, arg);
        if (arg != NULL)
        {
            _autorecordStorageId = arg->value.U8;
        }
    }
}

void Drone::cmdFlatTreamChangedRcv(){
    _trimLock.store(false);
}

char* CODECBUFFER;
int CODEBUFFERLEN;

eARCONTROLLER_ERROR Drone::decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void *drone)
{
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "DECODE CONFIG");
    Drone* d = (Drone*)drone;
    if (d->_videoOut != NULL)
    {
        if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264)
        {
            //TODO FIND OUT IF DECODING IS NEEDED
            if (true)
            {
                CODEBUFFERLEN = codec.parameters.h264parameters.spsSize+codec.parameters.h264parameters.ppsSize;
                CODECBUFFER = (char*) malloc(sizeof(char) * CODEBUFFERLEN);

                memcpy(CODECBUFFER, codec.parameters.h264parameters.spsBuffer, codec.parameters.h264parameters.spsSize);
                memcpy(CODECBUFFER + codec.parameters.h264parameters.spsSize, codec.parameters.h264parameters.ppsBuffer, codec.parameters.h264parameters.ppsSize);

                fwrite(codec.parameters.h264parameters.spsBuffer, codec.parameters.h264parameters.spsSize, 1, d->_videoOut);
                fwrite(codec.parameters.h264parameters.ppsBuffer, codec.parameters.h264parameters.ppsSize, 1, d->_videoOut);

                fflush (d->_videoOut);
            }
        }

    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "_videoOut is NULL !!!!!! 605.");
    }

    return ARCONTROLLER_OK;
}

/// PRIVATE

/// *************************************************************************************** JSON CONFIG FILE. NOT USED v
bool Drone::ardiscoveryConnect ()
{
    bool failed = false;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- ARDiscovery Connection");

    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = ARDISCOVERY_Connection_New (ARDISCOVERY_Connection_SendJsonCallback, ARDISCOVERY_Connection_ReceiveJsonCallback, this, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while creating discoveryData : %s", ARDISCOVERY_Error_ToString(err));
        failed = true;
    }

    if (!failed)
    {
        eARDISCOVERY_ERROR err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, _discoveryPort, _ip.c_str());
        if (err != ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while opening discovery connection : %s", ARDISCOVERY_Error_ToString(err));
            failed = true;
        }
    }

    ARDISCOVERY_Connection_Delete(&discoveryData);

    return failed;
}
eARDISCOVERY_ERROR Drone::ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *drone)
{
    Drone* self = (Drone*) drone;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataTx != NULL) && (dataTxSize != NULL))
    {
        *dataTxSize = sprintf((char *)dataTx, "{ \"%s\": %d,\n \"%s\": \"%s\",\n \"%s\": \"%s\",\n \"%s\": %d,\n \"%s\": %d }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, self->_d2cPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "BebopDroneStartStream",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "Unix",
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY, BD_CLIENT_STREAM_PORT,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY, BD_CLIENT_CONTROL_PORT) + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}

eARDISCOVERY_ERROR Drone::ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx,
                                                                      uint32_t dataRxSize,
                                                                      char *ip,
                                                                      void *drone)
{
    Drone* self = (Drone*) drone;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataRx != NULL) && (dataRxSize != 0))
    {
        char *json = (char*) malloc(dataRxSize + 1);
        strncpy(json, (char *)dataRx, dataRxSize);
        json[dataRxSize] = '\0';

        //read c2dPort ...

        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ReceiveJson:%s ", json);

        free(json);
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}
/// *************************************************************************************** JSON CONFIG FILE. NOT USED ^

eARCONTROLLER_ERROR Drone::didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *drone)
{
    Drone* self = (Drone*) drone;

    if(frame->isIFrame){
        //TODO use ffmpeg or something here to decode properly the next frames without the dirty file
        std::cout << "FULL iFrame received" << std::endl;
    }
    //uint8_t data

    //ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "GOT A FRAME");
/*
    uint8_t* pxls = (uint8_t*) malloc(sizeof(uint8_t) * (frame->used + CODEBUFFERLEN));
    memcpy(pxls, CODECBUFFER, CODEBUFFERLEN);
    memcpy(pxls + CODEBUFFERLEN, frame->data,  frame->used);

    cv::Mat picture = cv::imdecode(std::vector<uint8_t>(pxls, pxls + frame->used + CODEBUFFERLEN), CV_LOAD_IMAGE_COLOR);
    //cv::Mat picture(856, 480, CV_8U, frame->data);

    if(picture.data != NULL) {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "YATA");
        std::cout << "OK " << picture.rows << " " << picture.cols << ":" << picture.data << std::endl;
        cv::imshow("MDR", picture);
    }else{
        std::cout << "KO " << picture.rows << " " << picture.cols << ":" << picture.data << std::endl;
    }
*/
    /**/
    if (self->_videoOut != NULL)
    {
        if (frame != NULL)
        {
            //TODO detect if user requested camera ?
            if (true)
            {
                fwrite(frame->data, frame->used, 1, self->_videoOut);

                fflush (self->_videoOut);
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is NULL.");
        }
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "_videoOut is NULL.");
    }
/**/
    return ARCONTROLLER_OK;
}

void Drone::blockingInitCam() {

    while(_spinlock_camera.test_and_set(std::memory_order_acquire));
    //Not pretty
    _camera = cv::VideoCapture(_file_name);
    while(!_camera.isOpened()){
        sleep(1);
        _camera = cv::VideoCapture(_file_name);
    }
    _spinlock_camera.clear(std::memory_order_release);
}

cv::Mat Drone::retrieveLastFrame() {
    //Not pretty
    cv::Mat tmp, frame;

    while(_spinlock_camera.test_and_set(std::memory_order_acquire));
    _camera >> tmp;
    _spinlock_camera.clear(std::memory_order_release);

    if(tmp.data == NULL) {
        return tmp;
    }
    // We only want the last image, so we drop the previous ones.
    do {
        tmp.copyTo(frame);
        while(_spinlock_camera.test_and_set(std::memory_order_acquire));
        _camera >> tmp;
        _spinlock_camera.clear(std::memory_order_release);
    }while(tmp.data != NULL);

    return frame;
}

bool Drone::setHullPresence(bool isPresent) {

    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendSpeedSettingsHullProtection(
            _deviceController->aRDrone3,
            (uint8_t)isPresent
    );


    return error == ARCONTROLLER_OK;
}

bool Drone::isHullProtectionOn() const {
    return _hullProtectionPresence;
}

bool Drone::setVideoAutorecord(bool autoRecord, uint8_t storageId) {

    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPictureSettingsVideoAutorecordSelection(
            _deviceController->aRDrone3,
            (uint8_t)autoRecord,
            (uint8_t)storageId
    );



    return error == ARCONTROLLER_OK;
}

bool Drone::isVideoAutorecordOn() const{
    return _autorecordEnabled;
}

uint8_t Drone::getAutorecordStorageId() const {
    return _autorecordStorageId;
}

std::string Drone::getIpAddress() const {
    return _ip;
}

bool Drone::useFullNavdata() {
    if(not _usingFullNavdata) {

        try{
            _navdata->init(_ip, FULL_NAVDATA_DEFAULT_PORT);
        }
        catch(const boost::system::system_error &e)
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, "TODO",
                        "Bind on port %d failed, trying random port",
                        FULL_NAVDATA_DEFAULT_PORT
            );

            try{
                _navdata->init(_ip, 0);
            }
            catch(const boost::system::system_error &e)
            {
                std::cout<<e.what()<<std::endl;
                ARSAL_PRINT(ARSAL_PRINT_WARNING, "TODO",
                            "Bind on random port failed. Full navdata is not available"
                );
            }
        }


        _navdata->startReceive();
        return true;
    }
}

bool Drone::isUsingFullNavdata() const{
    return _navdata->receivedData();
}

Fullnavdata *Drone::getFullNavdata() const {
    return _navdata;
}
