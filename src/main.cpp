#include <Arduino.h>

#include "Tail.h"
#include "Motor.h"

#include "SensingThread.h"
#include "PIDLineFollower.h"
#include "RobotMotion.h"

#ifndef ENABLE_MICROROS_START_TOPIC
#define ENABLE_MICROROS_START_TOPIC 0
#endif

#ifndef REQUIRE_TEAM_BEFORE_START
#define REQUIRE_TEAM_BEFORE_START ENABLE_MICROROS_START_TOPIC
#endif

#ifndef PAMI_ID
#define PAMI_ID 1
#endif

#if ENABLE_MICROROS_START_TOPIC
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#endif

#if PAMI_ID == 1
const char START_TOPIC[] = "/pami1/start";
const char TEAM_TOPIC[] = "/pami1/team";
#elif PAMI_ID == 2
const char START_TOPIC[] = "/pami2/start";
const char TEAM_TOPIC[] = "/pami2/team";
#elif PAMI_ID == 3
const char START_TOPIC[] = "/pami3/start";
const char TEAM_TOPIC[] = "/pami3/team";
#elif PAMI_ID == 4
const char START_TOPIC[] = "/pami4/start";
const char TEAM_TOPIC[] = "/pami4/team";
#elif PAMI_ID == 5
const char START_TOPIC[] = "/pami5/start";
const char TEAM_TOPIC[] = "/pami5/team";
#else
const char START_TOPIC[] = "/pami/start";
const char TEAM_TOPIC[] = "/pami/team";
#endif

const bool AUTO_START_ON_BOOT = true;
const bool TEAM_REQUIRED_BEFORE_START = REQUIRE_TEAM_BEFORE_START;
const ObstacleBehavior ACTIVE_OBSTACLE_BEHAVIOR = OBSTACLE_GAME;
const double OBSTACLE_STOP_DISTANCE_CM = 20.0;
const unsigned long GAME_OBSTACLE_PAUSE_MS = 3000;
const unsigned long GAME_OBSTACLE_COOLDOWN_MS = 2000;
const double STRAIGHT_CORRECTION_GAIN = 0.0015;
const double MAX_STRAIGHT_CORRECTION = 0.12;

#if PAMI_ID == 3
const double LEFT_MOTOR_TRIM = 1.00;
const double RIGHT_MOTOR_TRIM = 0.90;
#elif PAMI_ID == 4
const double LEFT_MOTOR_TRIM = 0.975;
const double RIGHT_MOTOR_TRIM = 1.00;
#else
const double LEFT_MOTOR_TRIM = 1.00;
const double RIGHT_MOTOR_TRIM = 1.00;
#endif

enum TeamColor {
  TEAM_YELLOW,
  TEAM_BLUE
};

Motor m_left(15, 7, 16, 1, Right);
Motor m_right(5, 6, 4, 0, Left);
Tail tail;
SensingThread sensors;
PIDLineFollower lf(&m_right, &m_left, &sensors);
RobotMotion motion(&m_right, &m_left, &sensors.encoder);
PID pid(0.1, 0, 0);
double alpha = 0.6;

volatile bool start_requested = false;
bool start_pending = false;
bool path_running = false;
bool path_done = false;
bool tail_wag_started = false;
TeamColor selected_team = TEAM_BLUE;
bool team_selected = !TEAM_REQUIRED_BEFORE_START;

#if ENABLE_MICROROS_START_TOPIC
rcl_node_t node;
rcl_subscription_t start_subscription;
rcl_subscription_t team_subscription;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Bool start_msg;
std_msgs__msg__UInt8 team_msg;
#endif

const char* teamName(TeamColor team) {
  return team == TEAM_BLUE ? "blue" : "yellow";
}

const char* obstacleBehaviorName(ObstacleBehavior behavior) {
  if (behavior == OBSTACLE_AVOIDANCE) return "avoidance";
  if (behavior == OBSTACLE_GAME) return "game";
  return "ignore";
}

double readObstacleDistanceCm() {
  return sensors.sensor_values().obstacle;
}

void requestStart(bool should_start) {
  if (!should_start || path_running || path_done) {
    return;
  }

  if (TEAM_REQUIRED_BEFORE_START && !team_selected) {
    start_pending = true;
    Serial.println("Start received before team. Waiting for team topic.");
    return;
  }

  start_requested = true;
}

void selectTeam(TeamColor team) {
  selected_team = team;
  team_selected = true;
  Serial.println("Team selected: " + String(teamName(selected_team)));

  if (start_pending) {
    start_pending = false;
    requestStart(true);
  }
}

#if ENABLE_MICROROS_START_TOPIC
void startCallback(const void* msgin) {
  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
  requestStart(msg->data);
}

void teamCallback(const void* msgin) {
  const std_msgs__msg__UInt8* msg = (const std_msgs__msg__UInt8*)msgin;

  if (msg->data == 0) {
    selectTeam(TEAM_YELLOW);
  } else if (msg->data == 1) {
    selectTeam(TEAM_BLUE);
  }
}

void setupStartSubscriber() {
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pami", "", &support);
  rclc_subscription_init_default(
    &start_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    START_TOPIC
  );
  rclc_subscription_init_default(
    &team_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    TEAM_TOPIC
  );
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &start_subscription,
    &start_msg,
    &startCallback,
    ON_NEW_DATA
  );
  rclc_executor_add_subscription(
    &executor,
    &team_subscription,
    &team_msg,
    &teamCallback,
    ON_NEW_DATA
  );
}
#endif

void runSharedTestPath() {
  motion.moveForwardCm(100, 0.5);
  Serial.println("Finished first forward move.");
  motion.turnRightDeg(55, 0.5);
  Serial.println("Finished right turn.");
  motion.moveForwardCm(20, .8);
}

void runPami1BluePath() {
  motion.moveForwardCm(107, 0.5);
  Serial.println("Finished PAMI 1 first forward move.");
  motion.turnRightDeg(90, 0.5);
  Serial.println("Finished PAMI 1 right turn.");
  motion.moveForwardCm(20, 0.5);
  Serial.println("Finished PAMI 1 second forward move.");
}

void runPami1YellowPath() {
  motion.moveForwardCm(107, 0.5);
  Serial.println("Finished PAMI 1 first forward move.");
  motion.turnRightDeg(90, 0.5);
  Serial.println("Finished PAMI 1 right turn.");
  motion.moveForwardCm(20, 0.5);
  Serial.println("Finished PAMI 1 second forward move.");
}

void runPami1Path() {
  if (selected_team == TEAM_BLUE) {
    runPami1BluePath();
  } else {
    runPami1YellowPath();
  }
}

void runPami2YellowPath() {
  motion.moveForwardCm(95, 0.5);
  Serial.println("Finished PAMI 2 yellow first forward move.");
  motion.turnLeftDeg(80, 0.5);
  Serial.println("Finished PAMI 2 yellow left turn.");
  motion.moveForwardCm(65, 0.5);
  Serial.println("Finished PAMI 2 yellow second forward move.");
  motion.turnRightDeg(80, 0.5);
  Serial.println("Finished PAMI 2 yellow right turn.");
  motion.moveForwardCm(10, 0.5);
  Serial.println("Finished PAMI 2 yellow final forward move.");
}

void runPami2BluePath() {
  motion.moveForwardCm(95, 0.5);
  Serial.println("Finished PAMI 2 blue first forward move.");
  motion.turnRightDeg(80, 0.5);
  Serial.println("Finished PAMI 2 blue right turn.");
  motion.moveForwardCm(65, 0.5);
  Serial.println("Finished PAMI 2 blue second forward move.");
  motion.turnLeftDeg(80, 0.5);
  Serial.println("Finished PAMI 2 blue left turn.");
  motion.moveForwardCm(10, 0.5);
  Serial.println("Finished PAMI 2 blue final forward move.");
}

void runPami2Path() {
  if (selected_team == TEAM_BLUE) {
    runPami2BluePath();
  } else {
    runPami2YellowPath();
  }
}

void runPami3YellowPath() {
  motion.moveForwardCm(160, 0.5);
  Serial.println("Finished PAMI 3 yellow first forward move.");
  motion.turnLeftDeg(70, 0.5);
  Serial.println("Finished PAMI 3 yellow left turn.");
  motion.moveForwardCm(50, 0.5);
  Serial.println("Finished PAMI 3 yellow final forward move.");
}

void runPami3BluePath() {
  motion.moveForwardCm(90, 0.5);
  Serial.println("Finished PAMI 3 blue forward move.");
}

void runPami3Path() {
  if (selected_team == TEAM_BLUE) {
    runPami3BluePath();
  } else {
    runPami3YellowPath();
  }
}

void runPami4BluePath() {
  motion.moveForwardCm(150, 0.5);
  Serial.println("Finished PAMI 4 blue first forward move.");
}

void runPami4YellowPath() {
  motion.moveForwardCm(155, 0.5);
  Serial.println("Finished PAMI 4 yellow first forward move.");
  motion.turnLeftDeg(100, 0.5);
  Serial.println("Finished PAMI 4 yellow left turn.");
  motion.moveForwardCm(12, 0.5);
  Serial.println("Finished PAMI 4 yellow final forward move.");
}

void runPami4Path() {
  if (selected_team == TEAM_BLUE) {
    runPami4BluePath();
  } else {
    runPami4YellowPath();
  }
}

void runPami5Path() {
  
}

void wagTailAfterPath() {
  if (!tail_wag_started) {
    Serial.println("Tail wag active.");
    tail.write(60);
    delay(300);
    tail.write(120);
    delay(300);
    tail_wag_started = true;
  }

  tail.Wag();
}

void runPath() {
  path_running = true;

  Serial.println("Starting PAMI " + String(PAMI_ID) + " path.");
  Serial.println("Team: " + String(teamName(selected_team)));

#if PAMI_ID == 1
  runPami1Path();
#elif PAMI_ID == 2
  runPami2Path();
#elif PAMI_ID == 3
  runPami3Path();
#elif PAMI_ID == 4
  runPami4Path();
#elif PAMI_ID == 5
  runPami5Path();
#else
  runSharedTestPath();
#endif

  Serial.println("Path complete. Starting tail wag.");

  path_running = false;
  path_done = true;
  wagTailAfterPath();
}

void setup() {
  Serial.begin(115200);
  m_right.init();
  m_left.init();
  tail.attach(14);
  tail.setAngleLimits(60, 120);
  tail.initialPosition();  
  sensors.setup_sensors();
  sensors.start_sensing_thread();
  

  motion.setSpeedLimits(-1.0, 1.0);
  motion.setMotorTrim(LEFT_MOTOR_TRIM, RIGHT_MOTOR_TRIM);
  motion.setStraightCorrection(STRAIGHT_CORRECTION_GAIN, MAX_STRAIGHT_CORRECTION);
  motion.setMovementTimeoutMs(30000);
  motion.setObstacleReader(readObstacleDistanceCm);
  motion.setObstacleBehavior(
    ACTIVE_OBSTACLE_BEHAVIOR,
    OBSTACLE_STOP_DISTANCE_CM,
    GAME_OBSTACLE_PAUSE_MS,
    GAME_OBSTACLE_COOLDOWN_MS
  );

#if ENABLE_MICROROS_START_TOPIC
  setupStartSubscriber();
#endif

  if (AUTO_START_ON_BOOT) {
    requestStart(true);
  }

  Serial.println("PAMI ID: " + String(PAMI_ID));
  Serial.println("Team topic: " + String(TEAM_TOPIC));
  Serial.println("Selected team: " + String(teamName(selected_team)));
  Serial.println("Motor trim L/R: " + String(LEFT_MOTOR_TRIM) + " / " + String(RIGHT_MOTOR_TRIM));
  Serial.println("Straight correction gain/max: " + String(STRAIGHT_CORRECTION_GAIN) + " / " + String(MAX_STRAIGHT_CORRECTION));
  Serial.println("Obstacle mode: " + String(obstacleBehaviorName(ACTIVE_OBSTACLE_BEHAVIOR)));
  Serial.println("Waiting for start topic: " + String(START_TOPIC));
  
}

void loop() { 
#if ENABLE_MICROROS_START_TOPIC
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
#endif

  if (start_requested && !path_running && !path_done) {
    start_requested = false;
    runPath();
  }

  if (path_done) {
    wagTailAfterPath();
  }
}
