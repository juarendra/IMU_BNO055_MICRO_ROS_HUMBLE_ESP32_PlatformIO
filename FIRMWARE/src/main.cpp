#include <micro_ros_platformio.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/imu.h>
rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu msg_imu;

#include <sensor_msgs/msg/magnetic_field.h>
rcl_publisher_t publisher_mag;
sensor_msgs__msg__MagneticField msg_mag;


#include <std_msgs/msg/string.h>
std_msgs__msg__String string_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

float gyro_cov_ = 0.00001;
float accel_cov_ = 0.00001;

void error_loop(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(1000);
  ESP.restart();
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    //RCCHECK(rmw_uros_sync_session(10));

    //unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = 0;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}



bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_imu, &node);
  rcl_publisher_fini(&publisher_mag, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  digitalWrite(LED_PIN, LOW);
  
  return true;
}


void publishData(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
      sensors_event_t  angVelocityData , linearAccelData, magnetometerData;
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

      struct timespec time_stamp = getTime();
      msg_imu.header.stamp.sec = time_stamp.tv_sec;
      msg_imu.header.stamp.nanosec = time_stamp.tv_nsec;
      msg_imu.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");

      imu::Quaternion quat_raw = bno.getQuat();
      msg_imu.orientation.x = quat_raw.x();
      msg_imu.orientation.y = quat_raw.y();
      msg_imu.orientation.z = quat_raw.z();
      msg_imu.orientation.w = quat_raw.w();

      msg_imu.linear_acceleration.x = linearAccelData.acceleration.x;
      msg_imu.linear_acceleration.y = linearAccelData.acceleration.y;
      msg_imu.linear_acceleration.z = linearAccelData.acceleration.z;

      msg_imu.linear_acceleration_covariance[0] = accel_cov_;
      msg_imu.linear_acceleration_covariance[4] = accel_cov_;
      msg_imu.linear_acceleration_covariance[8] = accel_cov_;

      msg_imu.angular_velocity.x = angVelocityData.gyro.x;
      msg_imu.angular_velocity.y = angVelocityData.gyro.y;
      msg_imu.angular_velocity.z = angVelocityData.gyro.z;
      
      if(msg_imu.angular_velocity.x > -0.01 && msg_imu.angular_velocity.x < 0.01 )
        msg_imu.angular_velocity.x = 0; 

      if(msg_imu.angular_velocity.y > -0.01 && msg_imu.angular_velocity.y < 0.01 )
        msg_imu.angular_velocity.y = 0;

      if(msg_imu.angular_velocity.z > -0.01 && msg_imu.angular_velocity.z < 0.01 )
        msg_imu.angular_velocity.z = 0;

      msg_imu.angular_velocity_covariance[0] = -1;

      msg_mag.header.stamp.sec = time_stamp.tv_sec;
      msg_mag.header.stamp.nanosec = time_stamp.tv_nsec;
      msg_mag.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");

      msg_mag.magnetic_field.x = magnetometerData.magnetic.x;
      msg_mag.magnetic_field.y = magnetometerData.magnetic.y;
      msg_mag.magnetic_field.z = magnetometerData.magnetic.z;


      uint8_t sys, gyro, accel, mag = 0;
      bno.getCalibration(&sys, &gyro, &accel, &mag);

      RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
      RCSOFTCHECK(rcl_publish(&publisher_mag, &msg_mag, NULL));
    }

} 


bool createEntities()
{
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 30);

  //create init_options
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_imu_node_", "", &support));
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_mag,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu/mag_raw"));
  
  const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        publishData
  ));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  
  // synchronize time with the agent
  syncTime();
  digitalWrite(LED_PIN, HIGH);
  

  return true;
}



void setup() {
  set_microros_serial_transports(Serial);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    error_loop();
  }
  else{
   
    flashLED(3);

  }

}

void loop() {
  switch (state) 
    {
      case WAITING_AGENT:
          EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT);
          break;
      case AGENT_AVAILABLE:
          state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
          if (state == WAITING_AGENT) 
          {
              destroyEntities();
          }
          break;
      case AGENT_CONNECTED:
          EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
          if (state == AGENT_CONNECTED) 
          {
              rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
          }
          break;
      case AGENT_DISCONNECTED:
          destroyEntities();
          ESP.restart();
          state = WAITING_AGENT;
          break;
      default:
          break;
    }
}
