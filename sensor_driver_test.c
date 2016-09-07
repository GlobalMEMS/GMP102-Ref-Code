/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
 *
 * Date : 2016/08/26
 *
 * Revision : 1.0.0
 *
 * Usage: GMP102 Sensor Driver Test
 *
 ****************************************************************************
 * @section License
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/
 
 /*! @file sensor_driver_test.c
 *  @brief  GMP102 Sensor Driver Test Main Program 
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include "gmp102.h"

#define DELAY_MS(ms)	//.....     /* Add your time delay function here */


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

	s8 s8Res; 
	bus_support_t gmp102_bus;
	float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa;
	s16 s16T;
	s32 s32P;

	/* Add your HW initialization code here
	...
	...
	...
	...
	*/
	
	/* GMP102 I2C bus setup */
	bus_init_I2C(&gmp102_bus, GMP102_7BIT_I2C_ADDR); //Initialize I2C bus
	gmp102_bus_init(&gmp102_bus); //Initailze GMP102 bus to I2C

	/* GMP102 soft reset */
	s8Res = gmp102_soft_reset();
	
	/* Wait 100ms for reset complete */
	DELAY_MS(100);
	
	/* GMP102 get the pressure calibration parameters */
	s8Res = gmp102_get_calibration_param(fCalibParam);
	
	/* GMP102 initialization setup */
	s8Res = gmp102_initialization();
	
  for(;;)
  {
		/* Measure P */
		s8Res = gmp102_measure_P(&s32P);
		printf("P(code)=%d\r", s32P);
		
		/* Mesaure T */
		s8Res = gmp102_measure_T(&s16T);
		printf("T(code)=%d\r", s16T);
		
		/* Compensation */
		gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
		printf("P(Pa)=%f\r", fP_Pa);
		printf("T(C)=%f\r", fT_Celsius);

		/* Delay 1 sec */
		DELAY_MS(1000);
  }
}
