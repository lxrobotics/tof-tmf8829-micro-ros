/**
 * Copyright (c) 2026 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

#include "src/tmf8829_driver_arduino/tmf8829/tmf8829_app.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define LED_PIN 13

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/* TMF8829 16x16 zone result map. */
#define TMF8829_PIXEL_DATA_SIZE (3U)  /* uint16 distance + uint8 confidence */

/* TMF8829 16x16 mode field-of-view (typ. 41 deg per axis). */
#define FOV                     ((float)(41.0 * M_PI / 180.0))

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

typedef union
{
  float    f;
  uint8_t  b[4];
} FLOAT2UINT8_T;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void error_loop();
void on_tmf8829_data(uint8_t const * data, uint16_t const len);
void init_point_cloud_msg();

/**************************************************************************************
 * GLOBAL MEMBER VARIABLES
 **************************************************************************************/

static rcl_publisher_t publisher;
static rclc_executor_t executor;
static rclc_support_t  support;
static rcl_allocator_t allocator;
static rcl_node_t      node;

static sensor_msgs__msg__PointCloud2 msg;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "tmf8829_io_driver", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    "scan"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCSOFTCHECK(rmw_uros_sync_session(1000));

  init_point_cloud_msg();

  tmf8829_setupFn( 2 /* log-level nr*/, 400*1000UL);
  tmf8829_enable();
  tmf8829_start_measurement();
}

void loop()
{
  tmf8829_loopFn(on_tmf8829_data);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void init_point_cloud_msg()
{
  static micro_ros_utilities_memory_rule_t rules[] =
  {
    { "header.frame_id", 16                          },
    { "fields",          3                           },
    { "fields.name",     2                           },
    { "data",            16 * 16 * 3 * sizeof(float) }
  };

  micro_ros_utilities_memory_conf_t conf = {0};
  conf.rules   = rules;
  conf.n_rules = sizeof(rules) / sizeof(rules[0]);

  bool const ok = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    &msg,
    conf);
  if (!ok) error_loop();

  msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "tof_frame");

  msg.height       = 16;
  msg.width        = 16;
  msg.is_bigendian = false;
  msg.point_step   = 3 * sizeof(float);
  msg.row_step     = 3 * sizeof(float) * 16;
  msg.is_dense     = false;

  msg.fields.size = 3;
  char const * field_names[3] = { "x", "y", "z" };
  for (size_t i = 0; i < 3; i++)
  {
    msg.fields.data[i].name     = micro_ros_string_utilities_set(msg.fields.data[i].name, field_names[i]);
    msg.fields.data[i].offset   = i * sizeof(float);
    msg.fields.data[i].datatype = sensor_msgs__msg__PointField__FLOAT32;
    msg.fields.data[i].count    = 1;
  }

  msg.data.size = 16 * 16 * 3 * sizeof(float);
}

void on_tmf8829_data(uint8_t const * data, uint16_t const len)
{
  for (uint16_t h = 0; h < 16; h++)
  {
    for (uint16_t w = 0; w < 16; w++)
    {
      uint16_t const idx          = h * 16 + w;
      uint16_t const raw_distance = *((uint16_t const *)(data + idx * TMF8829_PIXEL_DATA_SIZE));
      float          depth_m      = (float)raw_distance * 0.25f / 1000.0f;

      FLOAT2UINT8_T x, y, z;
      x.f = sinf((float)w * (FOV / (float)16) - FOV / 2.0f) * depth_m;
      y.f = sinf((float)h * (FOV / (float)16) - FOV / 2.0f) * depth_m;
      z.f = depth_m;

      uint8_t * dst = msg.data.data + idx * 3 * sizeof(float);
      for (size_t k = 0; k < sizeof(float); k++) dst[0 * sizeof(float) + k] = x.b[k];
      for (size_t k = 0; k < sizeof(float); k++) dst[1 * sizeof(float) + k] = y.b[k];
      for (size_t k = 0; k < sizeof(float); k++) dst[2 * sizeof(float) + k] = z.b[k];
    }
  }

  int64_t const time_ns = rmw_uros_epoch_nanos();

  msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

void error_loop()
{
  for(;;)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
