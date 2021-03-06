/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
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
#include "pSensor_util.h"

#define DELAY_MS(ms)	//.....     /* Add your time delay function here */


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

	s8 s8Res; 
	bus_support_t gmp102_bus;
	float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa, fAlt_m;
	s16 s16Value[GMP102_CALIBRATION_PARAMETER_COUNT];
	u8 u8Power[GMP102_CALIBRATION_PARAMETER_COUNT];
	s16 s16T;
	s32 s32P, s32P64_Pa, s32P32_Pa, s32T_Celsius;

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
	s8Res = gmp102_get_calibration_param_fixed_point(s16Value, u8Power);
	
	/* GMP102 initialization setup */
	s8Res = gmp102_initialization();

	/* set sea level reference pressure */
	//If not set, use default 101325 Pa for pressure altitude calculation
	set_sea_level_pressure_base(102100.f);
	
  for(;;)
  {
		/* Measure P */
		s8Res = gmp102_measure_P(&s32P);
		printf("P(code)=%d\r", s32P);
		
		/* Mesaure T */
		s8Res = gmp102_measure_T(&s16T);
		printf("T(code)=%d\r", s16T);
		
		/* Compensation, choose one of the three below in actual implementation*/
		gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
		gmp102_compensation_fixed_point_s64(s16T, s32P, s16Value, u8Power, &s32T_Celsius, &s32P64_Pa);
		gmp102_compensation_fixed_point_s32(s16T, s32P, s16Value, u8Power, &s32T_Celsius, &s32P32_Pa);
		printf("P(Pa)=%f, %d, %d\r", fP_Pa, s32P64_Pa, s32P32_Pa);
		printf("T(C)=%f, %f\r", fT_Celsius, s32T_Celsius/256.0);

		/* Pressure Altitude */
		fAlt_m = pressure2Alt(fP_Pa);
		printf("Alt(m)=%f\r", fAlt_m);

		/* Delay 1 sec */
		DELAY_MS(1000);
  }
}
