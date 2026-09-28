/*
 * Micro-ROS ESP32 — Scrubber + Vacuum + Pump Actuator & Motor Control
 *
 * Topics:
 *   SUB  /move_scrubbAct   std_msgs/Bool             true=extend, false=retract
 *   SUB  /move_vaccAct     std_msgs/Bool             true=extend, false=retract
 *   SUB  /toggle_scrubber  std_msgs/Bool             true=ON, false=OFF
 *   SUB  /toggle_vacuum    std_msgs/Bool             true=ON, false=OFF
 *   SUB  /toggle_pump      std_msgs/Bool             true=ON, false=OFF
 *   PUB  /actuator_states  std_msgs/Int8MultiArray   [scrubb, vacc] 1=extended 0=retracted
 *   PUB  /motor_states     std_msgs/Int8MultiArray   [scrubb, vacc, pump] 1=ON 0=OFF
 *
 * Pin Assignment (ESP32 safe GPIOs):
 *   Scrubber Actuator : dir1=4,  pwm1=5
 *   Vacuum Actuator   : dir1=6,  pwm1=7
 *   Scrubber Relay    : GPIO 12
 *   Vacuum Relay      : GPIO 13
 *   Pump PWM          : GPIO 2   (PWM only, no dir pin — always LOW dir)
 *   Ultrasonic        : TODO (6 sensors, to be added later)
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8_multi_array.h>

// ============================================================
// Pin Definitions
// ============================================================
#define SCRUBB_DIR_PIN    4
#define SCRUBB_PWM_PIN    5

#define VACC_DIR_PIN      6
#define VACC_PWM_PIN      7

#define SCRUBB_RELAY_PIN  12
#define VACC_RELAY_PIN    13

#define PUMP_PWM_PIN      14
#define PUMP_PWM_DUTY     100   // 0–255

// TODO: Ultrasonic pins

// ============================================================
// State Variables
// ============================================================
bool scrubbAct_state    = false;
bool vaccAct_state      = false;
bool scrubbMotor_state  = false;
bool vaccMotor_state    = false;
bool pumpMotor_state    = false;  

unsigned long scrubbActionStart = 0;
unsigned long vaccActionStart   = 0;
const unsigned long scrubberDuration = 10000;  //ms
const unsigned long vaccumDuration   = 10000;  //ms

bool scrubbActRunning = false;
bool vaccActRunning   = false;

// ============================================================
// Reconnection Helper
// ============================================================
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

enum Motion { STOPPED, EXTENDING, RETRACTING };
Motion scrubbMotion = STOPPED;
Motion vaccMotion   = STOPPED;

// ============================================================
// Forward Declarations
// ============================================================
void extendScrubber();
void retractScrubber();
void stopScrubberAct();
void extendVacuum();
void retractVacuum();
void stopVacuumAct();
void pumpON();
void pumpOFF();

// ============================================================
// RCL Objects
// ============================================================
rclc_support_t     support;
rcl_init_options_t init_options;
rcl_node_t         micros_node;
rcl_timer_t        timer;
rclc_executor_t    executor;
rcl_allocator_t    allocator;

rcl_publisher_t    actuator_states_pub;
rcl_publisher_t    motor_states_pub;

rcl_subscription_t scrubbAct_sub;
rcl_subscription_t vaccAct_sub;
rcl_subscription_t scrubbMotor_sub;
rcl_subscription_t vaccMotor_sub;
rcl_subscription_t pumpMotor_sub;   

// ============================================================
// Messages
// ============================================================
std_msgs__msg__Int8MultiArray actuator_states_msg;
int8_t actuator_states_data[2];

std_msgs__msg__Int8MultiArray motor_states_msg;
int8_t motor_states_data[3];   // <-- extended to 3: [scrubb, vacc, pump]

std_msgs__msg__Bool scrubbAct_msg;
std_msgs__msg__Bool vaccAct_msg;
std_msgs__msg__Bool scrubbMotor_msg;
std_msgs__msg__Bool vaccMotor_msg;
std_msgs__msg__Bool pumpMotor_msg;  // <-- pump message

// ============================================================
// Actuator Control
// ============================================================
void extendScrubber() {
    digitalWrite(SCRUBB_DIR_PIN, HIGH);
    digitalWrite(SCRUBB_PWM_PIN, HIGH);
    scrubbActRunning  = true;
    scrubbMotion      = EXTENDING;
    scrubbActionStart = millis();
}

void retractScrubber() {
    digitalWrite(SCRUBB_DIR_PIN, LOW);
    digitalWrite(SCRUBB_PWM_PIN, HIGH);
    scrubbActRunning  = true;
    scrubbMotion      = RETRACTING;
    scrubbActionStart = millis();
}

void stopScrubberAct() {
    digitalWrite(SCRUBB_PWM_PIN, LOW);
    scrubbActRunning = false;
    scrubbMotion     = STOPPED;
}

void extendVacuum() {
    digitalWrite(VACC_DIR_PIN, HIGH);
    digitalWrite(VACC_PWM_PIN, HIGH);
    vaccActRunning  = true;
    vaccMotion      = EXTENDING;
    vaccActionStart = millis();
}

void retractVacuum() {
    digitalWrite(VACC_DIR_PIN, LOW);
    digitalWrite(VACC_PWM_PIN, HIGH);
    vaccActRunning  = true;
    vaccMotion      = RETRACTING;
    vaccActionStart = millis();
}

void stopVacuumAct() {
    digitalWrite(VACC_PWM_PIN, LOW);
    vaccActRunning = false;
    vaccMotion     = STOPPED;
}

// ============================================================
// Pump Control (PWM only, no dir pin)
// ============================================================
void pumpON() {
    analogWrite(PUMP_PWM_PIN, PUMP_PWM_DUTY);
    pumpMotor_state = true;
}

void pumpOFF() {
    analogWrite(PUMP_PWM_PIN, 0);
    pumpMotor_state = false;
}

// ============================================================
// Callbacks
// ============================================================
void scrubbAct_sub_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data == true && !scrubbAct_state) {
        extendScrubber();
    } else if (msg->data == false && scrubbAct_state) {
        retractScrubber();
    }
}

void vaccAct_sub_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data == true && !vaccAct_state) {
        extendVacuum();
    } else if (msg->data == false && vaccAct_state) {
        retractVacuum();
    }
}

void scrubbMotor_sub_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    scrubbMotor_state = msg->data;
    digitalWrite(SCRUBB_RELAY_PIN, scrubbMotor_state ? HIGH : LOW);
}

void vaccMotor_sub_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    vaccMotor_state = msg->data;
    digitalWrite(VACC_RELAY_PIN, vaccMotor_state ? HIGH : LOW);
}

void pumpMotor_sub_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    msg->data ? pumpON() : pumpOFF();
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    rcl_ret_t ret = RCL_RET_OK;

    actuator_states_data[0] = scrubbAct_state ? 1 : 0;
    actuator_states_data[1] = vaccAct_state   ? 1 : 0;
    ret = rcl_publish(&actuator_states_pub, &actuator_states_msg, NULL);
    (void)ret;

    motor_states_data[0] = scrubbMotor_state ? 1 : 0;
    motor_states_data[1] = vaccMotor_state   ? 1 : 0;
    motor_states_data[2] = pumpMotor_state   ? 1 : 0;  // <-- pump
    ret = rcl_publish(&motor_states_pub, &motor_states_msg, NULL);
    (void)ret;
}

// ============================================================
// Create / Destroy Entities
// ============================================================
bool create_entities() {
    const char *node_name = "esp32_uros_node";
    const char *ns        = "";
    const int domain_id   = 71;

    allocator    = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();

    rcl_ret_t ret = RCL_RET_OK;
    ret = rcl_init_options_init(&init_options, allocator);          (void)ret;
    ret = rcl_init_options_set_domain_id(&init_options, domain_id); (void)ret;
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&micros_node, node_name, ns, &support);

    // Publishers
    rclc_publisher_init_best_effort(
        &actuator_states_pub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
        "/actuator_states");

    rclc_publisher_init_best_effort(
        &motor_states_pub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
        "/motor_states");

    // Subscribers
    rclc_subscription_init_best_effort(
        &scrubbAct_sub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/move_scrubbAct");

    rclc_subscription_init_best_effort(
        &vaccAct_sub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/move_vaccAct");

    rclc_subscription_init_best_effort(
        &scrubbMotor_sub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/toggle_scrubber");

    rclc_subscription_init_best_effort(
        &vaccMotor_sub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/toggle_vacuum");

    rclc_subscription_init_best_effort(
        &pumpMotor_sub, &micros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/toggle_pump");

    // Timer 50ms = 20Hz
    rclc_timer_init_default2(
        &timer, &support,
        RCL_MS_TO_NS(50), timer_callback, true);

    // Executor — 5 subs + 1 timer = 6 handles
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 6, &allocator);
    rclc_executor_add_subscription(&executor, &scrubbAct_sub,   &scrubbAct_msg,   &scrubbAct_sub_callback,   ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &vaccAct_sub,     &vaccAct_msg,     &vaccAct_sub_callback,     ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &scrubbMotor_sub, &scrubbMotor_msg, &scrubbMotor_sub_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &vaccMotor_sub,   &vaccMotor_msg,   &vaccMotor_sub_callback,   ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &pumpMotor_sub,   &pumpMotor_msg,   &pumpMotor_sub_callback,   ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_init_options_fini(&init_options);
    rcl_node_fini(&micros_node);
    rclc_support_fini(&support);

    rcl_publisher_fini(&actuator_states_pub, &micros_node);
    rcl_publisher_fini(&motor_states_pub,    &micros_node);
    rcl_subscription_fini(&scrubbAct_sub,    &micros_node);
    rcl_subscription_fini(&vaccAct_sub,      &micros_node);
    rcl_subscription_fini(&scrubbMotor_sub,  &micros_node);
    rcl_subscription_fini(&vaccMotor_sub,    &micros_node);
    rcl_subscription_fini(&pumpMotor_sub,    &micros_node); 
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(SCRUBB_DIR_PIN,   OUTPUT);
    pinMode(SCRUBB_PWM_PIN,   OUTPUT);
    pinMode(VACC_DIR_PIN,     OUTPUT);
    pinMode(VACC_PWM_PIN,     OUTPUT);
    pinMode(SCRUBB_RELAY_PIN, OUTPUT);
    pinMode(VACC_RELAY_PIN,   OUTPUT);
    pinMode(PUMP_PWM_PIN,     OUTPUT); 

    // Safe defaults
    stopScrubberAct();
    stopVacuumAct();
    pumpOFF();
    digitalWrite(SCRUBB_RELAY_PIN, LOW);
    digitalWrite(VACC_RELAY_PIN,   LOW);

    // Publisher msg buffer init
    actuator_states_msg.data.data     = actuator_states_data;
    actuator_states_msg.data.size     = 2;
    actuator_states_msg.data.capacity = 2;
    actuator_states_msg.layout.dim.data     = NULL;
    actuator_states_msg.layout.dim.size     = 0;
    actuator_states_msg.layout.dim.capacity = 0;
    actuator_states_msg.layout.data_offset  = 0;

    motor_states_msg.data.data     = motor_states_data;
    motor_states_msg.data.size     = 3;    
    motor_states_msg.data.capacity = 3;
    motor_states_msg.layout.dim.data     = NULL;
    motor_states_msg.layout.dim.size     = 0;
    motor_states_msg.layout.dim.capacity = 0;
    motor_states_msg.layout.data_offset  = 0;

    // Subscriber msg init
    scrubbAct_msg.data   = false;
    vaccAct_msg.data     = false;
    scrubbMotor_msg.data = false;
    vaccMotor_msg.data   = false;
    pumpMotor_msg.data   = false;  

    state = WAITING_AGENT;
}

// ============================================================
// Loop
// ============================================================
void loop() {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(200,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                    ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) destroy_entities();
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(1000,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5))
                    ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }

    // Auto-stop scrubber after duration, update state
    if (scrubbActRunning && (millis() - scrubbActionStart >= scrubberDuration)) {
        scrubbAct_state = (scrubbMotion == EXTENDING);
        stopScrubberAct();
    }

    // Auto-stop vacuum after duration, update state
    if (vaccActRunning && (millis() - vaccActionStart >= vaccumDuration)) {
        vaccAct_state = (vaccMotion == EXTENDING);
        stopVacuumAct();
    }

    // TODO: Read ultrasonic sensors here
}