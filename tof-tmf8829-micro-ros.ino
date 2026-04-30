/**
 * Copyright (c) 2026 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "src/tmf8829_driver_arduino/tmf8829/tmf8829_app.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define I2C_CLK_SPEED               400000

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void on_tmf8829_data(
  uint8_t const * data,
  uint16_t const len);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while(!Serial) { }
  
  tmf8829_setupFn( 2 /* log-level nr*/, I2C_CLK_SPEED );
  tmf8829_enable();
  tmf8829_start_measurements();
}

void loop()
{
  tmf8829_loopFn(on_tmf8829_data);
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void on_tmf8829_data(
  uint8_t const * data,
  uint16_t const len)
{
  char msg_header[64] = {0};
  snprintf(msg_header, sizeof(msg_header), "[%ld] on_tmf8829_data (len = %d): ", millis(), len);
  Serial.print(msg_header);

  uint16_t const PIXEL_DATA_SIZE = 3;

  for ( uint16_t cnt = 0 ; cnt < len ; cnt += PIXEL_DATA_SIZE )
  {
    uint16_t const * raw_distance_ptr = (uint16_t const *)(data + cnt);
    uint16_t const   raw_distance = *raw_distance_ptr;
    float    const   distance = raw_distance * 0.25f;
    uint16_t const   distance_int = static_cast<uint16_t>(distance);

    Serial.print(distance_int);
    Serial.print(", ");
  }

  Serial.println();
}
