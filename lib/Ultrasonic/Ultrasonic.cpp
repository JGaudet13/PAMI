#include <Arduino.h>
#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trigPin, int echoPin, float vitesseSon, unsigned long timeoutEcho)
    : trigPin(trigPin), echoPin(echoPin), vitesseSon(vitesseSon), timeoutEcho(timeoutEcho) {}

void Ultrasonic::init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float Ultrasonic::read() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duree = pulseIn(echoPin, HIGH, timeoutEcho);
    if (duree == 0) {
        return -1.0;
    }
    return (duree * vitesseSon) / 2.0;
}
