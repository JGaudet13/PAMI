#include "MotorController.h"
#include <Arduino.h>

/**
 * Constructor: Uses Initialization List to satisfy the QuadratureEncoder requirements
 */
MotorController::MotorController(MotorControllerInfo info)
    : _pin1(info._pin1),
      _pin2(info._pin2),
      _pwmPin(info._pwmPin),
      encoder(info.encoderPin1, info.encoderPin2, info.unit),
      resolution(info.resolution),
      wheel_diameter(info.wheel_diameter),
      KP(info.KP),
      KI(info.KI),
      min_pwm(info.min_pwm),
      max_rps(info.max_rps),
      _channel(info._channel)
{
    running = false;
    direction = true;
    speed_target = 0.0;
    error_sum = 0.0;
    speed_ = 0.0;

    // Initialize the queue to hold 1 double (the latest speed command)
    speed_queue = xQueueCreate(1, sizeof(double));
}


  void MotorController::begin() {
    encoder.begin();
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    ledcSetup(_channel, 20000, 8); 
    ledcAttachPin(_pwmPin, _channel); 
    
    running = true;
}


void MotorController::setSpeed(double speed) {
    // Non-blocking send: overwrites the old target with the new one
    xQueueOverwrite(speed_queue, &speed);
}

double MotorController::getSpeed() {
    return speed_;
}

/**
 * Core Control Loop: Runs the pipeline
 */
void MotorController::control() {
    if (!running) return;
    
    updateTarget();
    getSpeedFromEncoder();
    double pwm_out = calculatePWM(speed_);
    // Serial.println("Current Speed: " + String(speed_) + " RPS, Target: " + String(speed_target) + " RPS, PWM: " + String(pwm_out));
    applyPWM(pwm_out);
}

// --- Private Functions ---

void MotorController::updateTarget() {
    double new_target;
    // Check if a new speed was sent to the queue
    if (xQueueReceive(speed_queue, &new_target, 0) == pdTRUE) {
        speed_target = new_target;
    }
}

void MotorController::getSpeedFromEncoder() {
  static int64_t last_time = 0;
  int64_t current_time = esp_timer_get_time();
  
  // 1. Get ticks since the last reset
  int16_t delta_ticks = encoder.getCount(); 
  
  // 2. Clear hardware counter immediately to start fresh
  encoder.reset(); 

  double dt = (current_time - last_time) / 1000000.0;
  if (dt <= 0) return;

  // 3. RPS = (Delta Ticks / Resolution) / Time
  speed_ = (double)delta_ticks / (double)resolution / dt;

  last_time = current_time;
}

double MotorController::calculatePWM(int current_speed) {
  double target_pct = speed_target / max_rps;
  double current_pct = current_speed / max_rps;

  // 2. Calculate Error in Percentage
  double error = target_pct - current_pct;
  
  // 3. Integral term (accumulate the percentage error over time)
  // We multiply by 0.01 because our loop runs every 10ms (0.01s)
  error_sum += error * 0.01; 
  
  // Constrain error_sum to prevent "Integral Windup"
  error_sum = constrain(error_sum, -1.0, 1.0);

  // 4. PID Formula (Result is a percentage of power)
  // KP and KI now act on the 0.0-1.0 scale
  double output_pct = (KP * error) + (KI * error_sum);

  // 5. Add Feedforward (Start with the percentage we *expect* to need)
  output_pct += target_pct;

  // 6. Scale to 0-255 for the PWM hardware
  double final_pwm = output_pct * 255.0;

  // 7. Apply Deadzone (min_pwm) so the motor actually turns at low speeds
  if (abs(final_pwm) < min_pwm && abs(speed_target) > 0.01) {
      final_pwm = (final_pwm > 0) ? min_pwm : -min_pwm;
  }

  return constrain(final_pwm, -255, 255);
}

void MotorController::applyPWM(double pwm) {
    // 1. Determine Direction
    direction = (pwm >= 0);
    int magnitude = abs((int)pwm);
    Serial.println("Applying PWM: " + String(pwm) + " | Direction: " + (direction ? "Forward" : "Backward") + " | Magnitude: " + String(magnitude));
    // 2. Handle Stop / Deadzone
    if (magnitude < 2) { // Tiny threshold to prevent buzzing at 0
        digitalWrite(_pin1, LOW);
        digitalWrite(_pin2, LOW);
        ledcWrite(_pwmPin, 0);
        return;
    }

    // 3. Apply Deadzone Offset (Feed-forward)
    if (magnitude < min_pwm) {
        magnitude = (int)min_pwm;
    }

    // 4. Set Hardware Pins
    if (direction) {
        digitalWrite(_pin1, HIGH);
        digitalWrite(_pin2, LOW);
    } else {
        digitalWrite(_pin1, LOW);
        digitalWrite(_pin2, HIGH);
    }
   ledcWrite(_channel, magnitude);

}

void MotorController::taskWrapper(void* pvParameters) {
  MotorController* motor = (MotorController*)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // Run at 100Hz (10ms)

  while (true) {
      motor->control();
      // vTaskDelayUntil ensures a rock-solid 100Hz frequency
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
* Starts the background thread
*/
void MotorController::startControlThread(int priority) {
  if (_controlTaskHandle != NULL) return; // Already running!

  // Create the task. We pass 'this' so the wrapper knows which motor to control.
  xTaskCreate(
      taskWrapper,
      "MotorCtrlTask",
      8192,
      this,
      priority,
      &_controlTaskHandle
  );
}

/**
* Ends the background thread
*/
void MotorController::stopControlThread() {
  if (_controlTaskHandle != NULL) {
      vTaskDelete(_controlTaskHandle);
      _controlTaskHandle = NULL;
      
      // Safety: Stop the motor hardware immediately
      applyPWM(0);
  }
}