/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmp102.c
 *
 * Date : 2016/10/05
 *
 * Revision : 1.1.0
 *
 * Usage: GMP102 sensor driver header file
 *
 ****************************************************************************
 * 
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
 
/*! @file gmp102.c
 *  @brief  GMP102 Sensor Driver File 
 *  @author Joseph FC Tseng
 */
 
 #include <stddef.h>
 #include <math.h>
 #include "gmp102.h"
 
 #define WAIT_FOR_DRDY_LOOP_DELAY(count) {int i;for(i = 0; i < (count); ++i);}
 
 bus_support_t* pGMP102Bus = 0;
 
 static const float GMP102_CALIB_SCALE_FACTOR[] = {
                    1.0E+00,
                    1.0E-05,
                    1.0E-10,
                    1.0E-05,
                    1.0E-10,
                    1.0E-15,
                    1.0E-12,
                    1.0E-17,
                    1.0E-21 };
 
 /*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmp102_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
	s8 comRslt = -1;
	if(pGMP102Bus == NULL){
		return -127;
	}
	else{
		comRslt = pGMP102Bus->bus_read(pGMP102Bus->u8DevAddr, u8Addr, pu8Data, u8Len);
		if(comRslt < u8Len) comRslt = -1;
	}
	
	return comRslt;
}
 

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gmp102_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
	s8 comRslt = -1;
	if(pGMP102Bus == NULL){
		return -127;
	}
	else{
		comRslt = pGMP102Bus->bus_write(pGMP102Bus->u8DevAddr, u8Addr, pu8Data, u8Len);
		if(comRslt < u8Len) comRslt = -1;
	}
	
	return comRslt;	
}

/*!
 * @brief GMP102 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_bus_init(bus_support_t* pbus){
	
	s8 comRslt = -1;
	u8 u8Data;
	
	//assign the I2C/SPI bus
	if(pbus == NULL)
		return -127;
	else
		pGMP102Bus = pbus;
	
	//Read chip ID
	comRslt = gmp102_burst_read(GMP102_REG_PID, &u8Data, 1);
	
	return comRslt;
}
 
/*!
 * @brief GMP102 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_soft_reset(void){
	
	s8 comRslt = -1;
	u8 u8Data = GMP102_SW_RST_SET_VALUE;
	
	//Set 00h = 0x24
	comRslt = gmp102_burst_write(GMP102_RST__REG, &u8Data, 1);
	
	return comRslt;
}

/*!
 * @brief Get gmp102 calibration parameters
 *        - Read calibration register AAh~BBh total 18 bytes 
 *        - Compose 9 calibration parameters from the 18 bytes
 *
 * @param fCalibParam: the calibration parameter array returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_get_calibration_param(float* fCalibParam){
	
	u8 u8DataBuf[GMP102_CALIBRATION_REGISTER_COUNT];
	s8 comRslt;
	s32 tmp, shift, i;
	
	//read the calibration registers
	comRslt = gmp102_burst_read(GMP102_REG_CALIB00, u8DataBuf, GMP102_CALIBRATION_REGISTER_COUNT);
	
	if(comRslt < GMP102_CALIBRATION_REGISTER_COUNT){
		comRslt = -1;
		goto EXIT;
	}
	
	// Get the parameters
	shift = sizeof(s32)*8 - 16;
	for(i = 0; i < GMP102_CALIBRATION_PARAMETER_COUNT; ++i){
		tmp = (u8DataBuf[2 * i] << 8) + u8DataBuf[2 * i + 1];
		fCalibParam[i] = ((tmp << shift) >> (shift + 2)) * (pow(10, (u8DataBuf[2 * i + 1] & 0x03))) * GMP102_CALIB_SCALE_FACTOR[i];
	}
	
	EXIT:
	return comRslt;
}

/*!
 * @brief gmp102 initialization
 *        Set AAh ~ ADh to 0x00
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_initialization(void){
	
	s8 comRslt = 0, s8Tmp;
	u8 u8Data[] = {0, 0, 0, 0};
	
	//Set AAh ~ AD to 0x00
	s8Tmp = gmp102_burst_write(GMP102_REG_CALIB00, u8Data, 4);
	
	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	
	EXIT:
	return comRslt;
	
}

/*!
 * @brief gmp102 measure temperature
 *
 * @param *ps16T calibrated temperature code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_measure_T(s16* ps16T){
	
	s8 comRslt = 0, s8Tmp;
	u8 u8Data[2];
	
	// Set A5h = 0x00, Calibrated data out
	u8Data[0] = 0x00;
	s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG1, u8Data, 1);
	
	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	
	// Set 30h = 0x08, T-Forced mode
	u8Data[0] = 0x08;
	s8Tmp = gmp102_burst_write(GMP102_REG_CMD, u8Data, 1);

	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	
	// Wait for 02h[0] DRDY bit set
	do{

		//wait a while
		WAIT_FOR_DRDY_LOOP_DELAY(1000)
		
		s8Tmp = gmp102_burst_read(GMP102_REG_STATUS, u8Data, 1);

		if(s8Tmp < 0){ //communication error
			comRslt = s8Tmp;
			goto EXIT;
		}
		comRslt += s8Tmp;		
		
	} while( GMP102_GET_BITSLICE(u8Data[0], GMP102_DRDY) != 1);
	
	// Read 09h~0Ah
	s8Tmp = gmp102_burst_read(GMP102_REG_TEMPH, u8Data, 2);

	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;	
	
	// Get the calibrated temperature in code
	*ps16T = (u8Data[0] << 8) + u8Data[1];
	
	EXIT:
	return comRslt;
}


/*!
 * @brief gmp102 measure temperature
 *
 * @param *ps16T calibrated temperature code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_measure_T(s16* ps16T);

/*!
 * @brief gmp102 measure pressure
 *
 * @param *ps32P raw pressure in code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_measure_P(s32* ps32P){

	s8 comRslt = 0, s8Tmp;
	u8 u8Data[3];
	
	// Set A5h = 0x02, raw data out
	u8Data[0] = 0x02;
	s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG1, u8Data, 1);
	
	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	
	// Set 30h = 0x09, P-Forced mode
	u8Data[0] = 0x09;
	s8Tmp = gmp102_burst_write(GMP102_REG_CMD, u8Data, 1);

	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	
	// Wait for 02h[0] DRDY bit set
	do{

		//wait a while
		WAIT_FOR_DRDY_LOOP_DELAY(1000)
		
		s8Tmp = gmp102_burst_read(GMP102_REG_STATUS, u8Data, 1);

		if(s8Tmp < 0){ //communication error
			comRslt = s8Tmp;
			goto EXIT;
		}
		comRslt += s8Tmp;		
		
	} while( GMP102_GET_BITSLICE(u8Data[0], GMP102_DRDY) != 1);
	
	// Read 06h~08h
	s8Tmp = gmp102_burst_read(GMP102_REG_PRESSH, u8Data, 3);

	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;	
	
	s8Tmp = sizeof(*ps32P)*8 - 24;
	// Get the raw pressure in code
	*ps32P = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
	*ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension
	
	EXIT:
	return comRslt;
}

/*!
 * @brief gmp102 temperature and pressure compensation
 *
 * @param s16T calibrated temperature in code
 * @param s32P raw pressure in code
 * @param fParam[] pressure calibration parameters
 * @param *pfT_Celsius calibrated temperature in Celsius returned to caller
 * @param *pfP_Pa calibraated pressure in Pa returned to caller
 * 
 * @return None
 *
 */
void gmp102_compensation(s16 s16T, s32 s32P, float fParam[], float* pfT_Celsius, float* pfP_Pa){
	
	*pfT_Celsius = GMP102_T_CODE_TO_CELSIUS(s16T);
	
	*pfP_Pa = \
				fParam[0] + \
				fParam[1]*s16T + \
				fParam[2]*s16T*s16T + \
				fParam[3]*s32P + \
				fParam[4]*s16T*s32P + \
				fParam[5]*s16T*s16T*s32P + \
				fParam[6]*s32P*s32P + \
				fParam[7]*s16T*s32P*s32P + \
				fParam[8]*s16T*s16T*s32P*s32P;
	
}

/*!
 * @brief gmp102 set pressure OSR
 *
 * @param osrP OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gmp102_set_P_OSR(GMP102_P_OSR_Type osrP){
	
	s8 comRslt = 0, s8Tmp;
	u8 u8Data;
	
	//Read A6h
	s8Tmp = gmp102_burst_read(GMP102_REG_CONFIG2, &u8Data, 1);
	
	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;	

	//Set the A6h[2:0] OSR bits
	u8Data = GMP102_SET_BITSLICE(u8Data, GMP102_P_OSR, osrP);
	s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG2, &u8Data, 1);
	
	if(s8Tmp < 0){ //communication error
		comRslt = s8Tmp;
		goto EXIT;
	}
	comRslt += s8Tmp;
	
	EXIT:
	return comRslt;
}
