#pragma once
#include <Arduino.h>
#include "QE_Manager.h"
#include "US_Manager.h"
#include "LineSensor.h"
struct sensorInfoPID {
  double left_speed;
  double right_speed;
  double obstacle;
  double LS_error;
};




  class SensingThread {
    public:
      QE_Manager encoder;
      US_Manager ultrasonic;
      LineSensor linesensor;
      sensorInfoPID _currentData;
      SensingThread();
      void setup_sensors();
      void start_sensing_thread();
      void stop_sensing_thread();
      sensorInfoPID sensor_values();
  
    private:
      TaskHandle_t _taskHandle = NULL; // Moved inside the class
  };
