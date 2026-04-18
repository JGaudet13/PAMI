#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "QuadratureEncoder.h"
#include "esp_timer.h"
#include "freertos/task.h"
//this holds the info about a motor controller
struct MotorControllerInfo {
  int _pin1;
  int _pin2;
  int _pwmPin;
  
  int encoderPin1;
  int encoderPin2;
  
  pcnt_unit_t unit;

  int resolution;
  double wheel_diameter;

  double KP;
  double KI;
  double min_pwm;
  double max_rps;
  int _channel;
};

class MotorController {
  private:
    TaskHandle_t _controlTaskHandle = NULL;
    int _pin1;
    int _pin2;
    int _pwmPin;

    QuadratureEncoder encoder;
    int resolution;
    double wheel_diameter;

    double KP;
    double KI;
    double min_pwm;
    double max_rps;
    int _channel;
    
    bool running;
    bool direction; // true for forward, false for backward
    double speed_target;
    double error_sum;

    double speed_;

    void updateTarget(); // grabs the latest target speed from the queue, and updates the speed_target variable
    void getSpeedFromEncoder(); // gets the speed from the encoder
    double calculatePWM(int current_speed); //calculates the PWM value based on the current speed and the target speed,KI and KP
    void applyPWM(double pwm); 
    static void taskWrapper(void* param);// applies the PWM value to the motor, and sets the direction, and acounts for deadzone
  public:
    QueueHandle_t speed_queue;

    MotorController(MotorControllerInfo info);
    void begin();
    void setSpeed(double speed); //extern in order to push a speed
    double getSpeed(); //get actual speed of the motor
    void control();
    
    void startControlThread(int priority);
    void stopControlThread(); //control loop, should be called in a separate task, and will update the motor speed based on the target speed and the encoder feedback, and run on specific thread
};