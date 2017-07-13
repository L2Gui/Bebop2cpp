#include <iostream>
#include "Drone.h"

/// PUBLIC
Drone::Drone(const std::string& ipAddress, unsigned int discoveryPort):
        _deviceController(NULL),
        _deviceState(ARCONTROLLER_DEVICE_STATE_MAX)
{
    bool failed = false;
    ARDISCOVERY_Device_t *device = NULL;
    pid_t child = 0;
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;


    if (mkdtemp(_fifo_dir) == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkdtemp failed.");
        _isValid = false;
        return;
    }
    snprintf(_fifo_name, sizeof(_fifo_name), "%s/%s", _fifo_dir, FIFO_NAME);

    if(mkfifo(_fifo_name, 0666) < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
        _isValid = false;
        return;
    }

    ARSAL_Sem_Init (&(_stateSem), 0, 0);

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");

    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;
    device = ARDISCOVERY_Device_New (&errorDiscovery);

    if (errorDiscovery == ARDISCOVERY_OK) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
        errorDiscovery = ARDISCOVERY_Device_InitWifi (device, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2", ipAddress.c_str(), discoveryPort);

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

    _isValid = !failed;
}

bool Drone::connect()
{
    eARCONTROLLER_ERROR error;
    if (_isValid && !_isConnected)
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
    bool res = _isValid && _isConnected && !isStopped();
    return res;
}

/// COMMANDS
bool Drone::takeOff() {
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingTakeOff(_deviceController->aRDrone3);

    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending takeoff command");
    }
    return error == ARCONTROLLER_OK;
}
bool Drone::land(){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingLanding(_deviceController->aRDrone3);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending takeoff command");
    }
    return error == ARCONTROLLER_OK;
}
bool Drone::emergency(){
    eARCONTROLLER_ERROR error = _deviceController->aRDrone3->sendPilotingEmergency(_deviceController->aRDrone3);
    if(error != ARCONTROLLER_OK){
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "-error sending takeoff command");
    }
    return error == ARCONTROLLER_OK;
}
/// GETTERS
bool Drone::isConnected() {
    return _isConnected;
}

bool Drone::isValid() {
    return _isValid;
}

bool Drone::isStopped(){
    return _deviceState == ARCONTROLLER_DEVICE_STATE_STOPPED;
}
bool Drone::isStarting(){
    return _deviceState == ARCONTROLLER_DEVICE_STATE_STARTING;
}
bool Drone::isRunning() {
    return _deviceState == ARCONTROLLER_DEVICE_STATE_RUNNING;
}
bool Drone::isPaused(){
    return _deviceState == ARCONTROLLER_DEVICE_STATE_PAUSED;
}
bool Drone::isStopping(){
    return _deviceState == ARCONTROLLER_DEVICE_STATE_STOPPING;
}
/// PROTECTED
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

    // if the command received is a battery state changed
    switch(commandKey) {
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
            d->cmdBatteryStateChangedRcv(elementDictionary);
            break;
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED:
            d->cmdSensorStateListChangedRcv(elementDictionary);
            break;
        default:
            break;
    }
}

/**
 * STATIC
 * @param newState
 * @param error
 * @param drone (void*)(Drone d)
 */
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

    if (elementDictionary == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
        return;
    }

    // get the command received in the device controller
    HASH_ITER(hh, elementDictionary, dictElement, dictTmp) {
        // get the Name
        HASH_FIND_STR (dictElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME, arg);
        if (arg != NULL) {
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

    // update UI
    std::cout << "New battery lvl " << (int)(arg->value.U8) << std::endl;
}

/// PRIVATE