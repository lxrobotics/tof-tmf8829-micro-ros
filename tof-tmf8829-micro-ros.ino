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
  Serial.print(millis());
  Serial.print(" on_tmf8829_data: ");

  for ( uint16_t cnt = 0 ; cnt < len ; cnt ++ )
  {
    Serial.print(static_cast<int>(data[cnt]));
    Serial.print(", ");
  }

  Serial.println();
}