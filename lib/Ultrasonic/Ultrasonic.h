class Ultrasonic {
  

  public:
    
    int trigPin;
    int echoPin;
    float vitesseSon;
    unsigned long timeoutEcho;
    Ultrasonic(int trigPin = 12, int echoPin = 13, float vitesseSon = 0.0343, unsigned long timeoutEcho = 30000);
    void init();
    float read();
};