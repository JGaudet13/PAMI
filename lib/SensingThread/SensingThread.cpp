#include "SensingThread.h"


static void sensingTaskWrapper(void* pvParameters) {
    SensingThread* instance = (SensingThread*)pvParameters;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms target

    for (;;) {
        // 1. Run the hardware updates
        instance->encoder.update();
        instance->ultrasonic.update();


        // 2. Update the shared data structure
        // We update individual fields to ensure the volatile struct is refreshed
        instance->_currentData.left_speed  = instance->encoder.getLeftSpeed();
        instance->_currentData.right_speed = instance->encoder.getRightSpeed();
        instance->_currentData.obstacle    = instance->ultrasonic.get_obj_distance();
        instance->_currentData.LS_error    = instance->linesensor.readSensorSquared();

        // 3. Wait for the next 10ms mark
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
SensingThread::SensingThread() 
    : encoder(18, 8, 10, 9, 78.0, 35.0), // Pass your specific pins/data here
      ultrasonic(),                        // US_Manager seems okay with default
      linesensor(0.5)                         // If LineSensor needs pins, add them here too
{
    _taskHandle = NULL;
    _currentData = {0.0, 0.0, 0.0, 0.0};
}

void SensingThread::setup_sensors() {
    encoder.init();
    ultrasonic.init();
    linesensor.init();
    linesensor.calibrateLineSensor();
    // Initialize data to zero
    _currentData = {0.0, 0.0, 0.0, 0.0};
}

void SensingThread::start_sensing_thread() {
    // Only start if not already running
    if (_taskHandle == NULL) {
        xTaskCreatePinnedToCore(
            sensingTaskWrapper,   // Function
            "SensingTask",        // Name
            4096,                 // Stack Size
            this,                 // Pass this object as parameter
            2,                    // Priority (Medium)
            &_taskHandle,         // Store handle in the object
            0                     // Pin to Core 0
        );
    }
}

void SensingThread::stop_sensing_thread() {
    if (_taskHandle != NULL) {
        vTaskDelete(_taskHandle);
        _taskHandle = NULL;
    }
}

sensorInfoPID SensingThread::sensor_values() {
    // Return a local copy of the volatile struct
    return _currentData;
}