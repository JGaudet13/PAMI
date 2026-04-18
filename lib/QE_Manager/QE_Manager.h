#include <Arduino.h>
#include "QuadratureEncoder.h"

class QE_Manager {
  public:
    QuadratureEncoder right_enc;
    QuadratureEncoder left_enc;
    bool updateDistance = false;
    double right_speed = 0;
    double left_speed = 0;
    double baseLength;
    double wheelDiam;
    
    double distanceRight = 0;
    double distanceLeft = 0;
    QE_Manager(int rA, int rB, int lA, int lB, double base, double diam);
    void init();
    void update();
    
    double getRotSpeed();
    double getTotalSpeed();

    double getRightSpeed();
    double getLeftSpeed();
    
    void startDistanceCounter();
    double getDistanceRight();
    double getDistanceLeft();
    void resetDistanceCounter();

};