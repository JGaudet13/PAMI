#pragma once
#include <Ultrasonic.h>
#include <Arduino.h>
class US_Manager {

  public:
    US_Manager();
    void init();
    void update();
    float get_obj_distance();
    
    Ultrasonic ultrasonic;   
    float distance;
    bool obstacleDetected = false;
    float alpha;
    unsigned long lastUpdateTime;

};