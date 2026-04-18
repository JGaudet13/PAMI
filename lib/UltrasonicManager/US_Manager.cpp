#include "US_Manager.h"

US_Manager::US_Manager() : distance(0), alpha(0.2) {
        ultrasonic = Ultrasonic(12, 13, 0.0343, 4000);     
}
void US_Manager::init() {
    ultrasonic.init();
    Serial.println("Ultrasonic Manager re-initialized.");
    Serial.println("Ultrasonic Manager initialized with trigPin: " + String(ultrasonic.trigPin) + " and echoPin: " + String(ultrasonic.echoPin));
    Serial.println("Timeout for echo: " + String(ultrasonic.timeoutEcho) + " microseconds");
    Serial.println("Speed of sound: " + String(ultrasonic.vitesseSon) + " cm/us");
    Serial.println("Initial distance: " + String(distance) + " cm");
    Serial.println("Alpha (smoothing factor): " + String(alpha));
    Serial.println("Ultrasonic Manager setup complete.");
}
void US_Manager::update() {
    float newDistance = ultrasonic.read();
    if (newDistance > 0) { 
        distance = alpha * newDistance + (1 - alpha) * distance; // Simple moving average filter 
        obstacleDetected = true; 
    }
    else {
        obstacleDetected = false;
    }
}


float US_Manager::get_obj_distance() {
    if(obstacleDetected) {
        return distance;
    }
    return -1;  // Meaningful value indicating no obstacle detected
}
