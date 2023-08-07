
/*****************************************************************************
 * Copyright (c) 2020-2021, WiSig Networks Pvt Ltd. All rights reserved.     *
 * www.wisig.com                                                             *
 *                                                                           *
 * All information contained herein is property of WiSig Networks Pvt Ltd.   *
 * unless otherwise explicitly mentioned.                                    *
 *                                                                           *
 * The intellectual and technical concepts in this file are proprietary      *
 * to WiSig Networks and may be covered by granted or in process national    *
 * and international patents and are protect by trade secrets and            *
 * copyright law.                                                            *
 *                                                                           *
 * Redistribution and use in source and binary forms of the content in       *
 * this file, with or without modification are not permitted unless          *
 * permission is explicitly granted by WiSig Networks.                       *
 * If WiSig Networks permits this source code to be used as a part of        *
 * open source project, the terms and conditions of CC-By-ND (No Derivative) *
 * license (https://creativecommons.org/licenses/by-nd/4.0/) shall apply.    *
 *****************************************************************************/

#include "wnI2c.h"
#include "wn5gNrPsDataTypes.h"
#include <stdio.h>


/*------------------------- I2C DRIVER ----------------------------*/

/*---------------------------------------------------------------
wnI2Csetaddress - This function is to Load/Set I2C target address
Arguments:
	device - Pointer to structure
	i2c_addr - Target address
---------------------------------------------------------------*/
wnVoid wnI2CSetTargetaddress (wnI2CDeviceT *device, wnUInt32 i2c_addr)
{	
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	i2c_instance->I2C_TAR &= ~(0x3FFF);
	
	reg_val = i2c_instance->I2C_TAR;
	reg_val |= (i2c_addr & 0x3FFF);;
	i2c_instance->I2C_TAR = reg_val;
}

/*---------------------------------------------------------------
wnI2Csetaddress - This function is to Load/Set I2C slave address
Arguments:
	device - Pointer to structure
	i2c_addr - Target address
---------------------------------------------------------------*/
wnVoid wnI2CSetSlaveAddress (wnI2CDeviceT *device, wnUInt32 i2c_addr)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	i2c_instance->I2C_SAR &= ~(0x3FFF);
	
	reg_val = i2c_instance->I2C_SAR;
	reg_val |= (i2c_addr & 0x3FFF);
	i2c_instance->I2C_SAR = reg_val;
}

/*---------------------------------------------------------------
wnIsI2CBusy : This function is to check whether I2C is busy or not
Arguments:
	device - Pointer to structure
return - WN_TRUE/WN_FALSE
---------------------------------------------------------------*/
wnBool wnIsI2CBusy (wnI2CDeviceT *device)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_STATUS;

	if((reg_val & 0x1) == 0x1){
		return WN_TRUE;
	}
	else{
		return WN_FALSE;
	}

}

/*--------------------------------------------------------------
wnI2CMasterEnableDisable - This function is to enable or disable I2C controller as master
Arguments:
	device - Pointer to structure
	enable_disable - enum to disable or enable master 
--------------------------------------------------------------*/
wnVoid wnI2CMasterEnableDisable (wnI2CDeviceT *device, wnUInt32 enable_disable)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	reg_val = i2c_instance->I2C_CTRL;
	if(enable_disable == I2C_MASTER_ENABLE){
		reg_val |= (0x1 << I2C_MASTER_ENABLE_DISABLE_POS);
		reg_val |= (0x1 << I2C_SLAVE_ENABLE_DISABLE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
	else{
		reg_val &= ~(0x1 << I2C_MASTER_ENABLE_DISABLE_POS);
		reg_val &= ~(0x1 << I2C_SLAVE_ENABLE_DISABLE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
}

/*--------------------------------------------------------------
wnI2CSlaveEnableDisable - This function is to enable or disable I2C controller as slave 
Arguments:
	device - Pointer to structure
	enable_disable - enum to disable or enable slave
--------------------------------------------------------------*/
wnVoid wnI2CSlaveEnableDisable (wnI2CDeviceT *device, wnUInt32 enable_disable)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	reg_val = i2c_instance->I2C_CTRL;
	if(enable_disable == I2C_SLAVE_ENABLE){	
		reg_val &= ~(0x1 << I2C_MASTER_ENABLE_DISABLE_POS);
		reg_val &= ~(0x1 << I2C_SLAVE_ENABLE_DISABLE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
	else{
		reg_val |= (0x1 << I2C_MASTER_ENABLE_DISABLE_POS);
		reg_val |= (0x1 << I2C_SLAVE_ENABLE_DISABLE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
}

/*-------------------------------------------------------------
wnI2CSetSpeed - This function is to set the operating speed for I2C
Arguments:
	device - Pointer to structure
	i2c_speed - speed value
return - I2C_SUCCESS/I2C_FAILURE
-------------------------------------------------------------*/

wnBool  wnI2CSetSpeed (wnI2CDeviceT *device, wnUInt32 i2c_speed)
{	
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	wnUInt32 clock_mhz = device->I2C_CLOCK/1000000;
	
	if(wnIsI2CEnabledDisabled(device) == WN_FALSE){
		switch(i2c_speed){
			
		case I2C_SPEED_STANDARD:	
			reg_val = (wnUInt32)(i2c_instance->I2C_CTRL);
			reg_val =  ( reg_val & ( ~(3<<I2C_SPEED_POS)) ) | (I2C_SPEED_STANDARD<<I2C_SPEED_POS) ;
			i2c_instance->I2C_CTRL = reg_val;
			
			i2c_instance->I2C_SS_SCL_HCNT &= ~(0xFFFF);
			i2c_instance->I2C_SS_SCL_HCNT = ((clock_mhz*4000 / 100)+1);

			i2c_instance->I2C_SS_SCL_LCNT &= ~(0xFFFF);
			i2c_instance->I2C_SS_SCL_LCNT |= ((clock_mhz*4700 / 100)+1);

			break;
		case I2C_SPEED_FAST:
			reg_val = (wnUInt32)(i2c_instance->I2C_CTRL);
			reg_val =  ( reg_val & ( ~(3<<I2C_SPEED_POS)) ) | (I2C_SPEED_FAST<<I2C_SPEED_POS) ;
			i2c_instance->I2C_CTRL = reg_val;
			
			i2c_instance->I2C_FS_SCL_HCNT &= ~(0xFFFF);
			i2c_instance->I2C_FS_SCL_HCNT |= ((clock_mhz*600 / 100)+1);
			
			i2c_instance->I2C_FS_SCL_LCNT &= ~(0xFFFF);
			i2c_instance->I2C_FS_SCL_LCNT |= ((clock_mhz*1300 / 100)+1);
			break;
		}

		return I2C_SUCCESS;
	}	
	else{
		return I2C_FAILURE;
	}
}


/*--------------------------------------------------------------
wnI2CSetMasterAddressMode - This function is to set 7-bit or 10-bit addressing mode for master
Arguments:
	device - Pointer to structure
	mode - enum to select the master addressing mode 7-bit or 10-bit
--------------------------------------------------------------*/
wnVoid wnI2CSetMasterAddressMode (wnI2CDeviceT *device, wnUInt32 mode )
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	reg_val = i2c_instance->I2C_CTRL;
	
	if(mode == I2C_7BITADDR){
		reg_val &= ~(0x1 << I2C_MASTER_ADDRESS_MODE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
	else{
		reg_val |= (0x1 << I2C_MASTER_ADDRESS_MODE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
}

/*--------------------------------------------------------------
wnI2CSetMasterAddressMode - This function is to set 7-bit or 10-bit addressing mode for master
Arguments:
	device - Pointer to structure
	mode - enum to select the slave addressing mode 7-bit or 10-bit
--------------------------------------------------------------*/
wnVoid wnI2CSetSlaveAddressMode (wnI2CDeviceT *device, wnUInt32 mode)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	reg_val = i2c_instance->I2C_CTRL;
	
	if(mode == I2C_7BITADDR){
		reg_val &= ~(0x1 << I2C_SLAVE_ADDRESS_MODE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
	else{
		reg_val |= (0x1 << I2C_SLAVE_ADDRESS_MODE_POS);
		i2c_instance->I2C_CTRL = reg_val;
	}
}

/*--------------------------------------------------------------
wnI2CWrite - This function is to write data transmitted on the bus
Arguments:
	device - Pointer to structure
	data - data to be written on bus to transmit
--------------------------------------------------------------*/
wnVoid wnI2CWrite (wnI2CDeviceT *device, wnUInt8 data)
{ 
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	i2c_instance->I2C_DATA_CMD =  data;
}

/*--------------------------------------------------------------
wnI2Cread - This function is to read data received from the bus
Arguments:
	device - Pointer to structure
return - data received on the bus 
--------------------------------------------------------------*/
wnUInt8 wnI2CRead (wnI2CDeviceT *device)
{
	wnUInt8 read_data = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	read_data = (i2c_instance->I2C_DATA_CMD & 0xFF);
	return read_data;
}

/*--------------------------------------------------------------
wnI2cIsRxFifoFull : This function is to check whether RX FIFO is full or not
Arguments:
	dev - Pointer to structure
return : WN_TRUE/WN_FALSE
-------------------------------------------------------------*/

wnBool wnI2cIsRxFifoFull (wnI2CDeviceT *device)
{
	wnUInt32 reg_val;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_STATUS;
	
	if(((reg_val >> I2C_RX_FIFO_RFF_POS) & 1) == 1){
		return WN_TRUE;
	}
	else{
		return WN_FALSE;
	}
}

/*--------------------------------------------------------------
wnI2cIsRxFifoEmpty : This function is to check whether RX FIFO is Empty or not
Arguments:
	dev - Pointer to structure
return : WN_TRUE/WN_FALSE
-------------------------------------------------------------*/

wnBool wnI2cIsRxFifoEmpty (wnI2CDeviceT *device)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_STATUS;
	if(((reg_val >> I2C_RX_FIFO_RFNE_POS) & 1) == 0){
		return WN_TRUE;
	}
	else{
		return WN_FALSE;
	}
}

/*-------------------------------------------------------------
wnI2cIsTxFifoFull : This function is to check whether TX FIFO is full or not
Arguments:
	device - Pointer to structure
return : WN_TRUE/WN_FALSE
-------------------------------------------------------------*/

wnBool wnI2cIsTxFifoFull (wnI2CDeviceT *device)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_STATUS;
	
	if(((reg_val >> I2C_TX_FIFO_TFNF_POS) & 1) == 0){
		return WN_TRUE;
	}
	else{
		return WN_FALSE;
	}
}	

/*-------------------------------------------------------------
wnI2cIsTxFifoEmpty : This function is to check whether TX FIFO is empty or not
Arguments:
	device - Pointer to structure
return : WN_TRUE/WN_FALSE
-------------------------------------------------------------*/

wnBool wnI2cIsTxFifoEmpty (wnI2CDeviceT *device)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_STATUS;
	
	if(((reg_val >> I2C_TX_FIFO_TFE_POS) & 1) == 1){
		return WN_TRUE;
	}
	else{
		return WN_FALSE;
	}
}

/*-------------------------------------------------------------------
wnI2cRestartEnableDisable : This function is to Enable or Disable the Restart of I2C
Arguments:
	device - Pointer to structure
	enableDisable - ENABLE/DISABLE enum
-------------------------------------------------------------------*/

wnVoid wnI2cRestartEnableDisable (wnI2CDeviceT *device, wnUInt32 enableDisable)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_CTRL;
	
	if(enableDisable == ENABLE){
		reg_val |= (0x1 << I2C_RESTART_ENABLE_DISABLE_POS);
		i2c_instance->I2C_CTRL =reg_val;
	}
	else{
		reg_val &= ~(0x1 << I2C_RESTART_ENABLE_DISABLE_POS);
		i2c_instance->I2C_CTRL =reg_val;
	}		
}

/*-------------------------------------------------------------------
wnIsI2CEnabledDisabled : This function is to check whether I2C is enabled or disabled
Arguments:
	device - Pointer to structure
return : WN_TRUE/WN_FALSE
------------------------------------------------------------------*/

wnBool wnIsI2CEnabledDisabled (wnI2CDeviceT *device)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_ENABLE;
	
	if(((reg_val >> I2C_ENABLE_DISABLE_POS) & 1) == ENABLE){
		return WN_TRUE;
	}
	else{
		return WN_FALSE;
	}
}

/*------------------------------------------------------------------
wnI2cEnableDisable : This function is to enable or disable the I2C
Arguments:
	device - Pointer to structure
	enablediasble - ENABLE/DISABLE
return - I2C_SUCCESS/I2C_FAILURE
------------------------------------------------------------------*/

wnBool wnI2cEnableDisable (wnI2CDeviceT *device, wnUInt32 enabledisable)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_ENABLE;

	if(wnIsI2CBusy(device) == WN_FALSE){
		if(enabledisable == ENABLE){
			reg_val |= (0x1 << I2C_ENABLE_DISABLE_POS);
			i2c_instance->I2C_ENABLE = reg_val;
		}
		else{
			reg_val &= ~(0x1 << I2C_ENABLE_DISABLE_POS);
			i2c_instance->I2C_ENABLE = reg_val;}
		return I2C_SUCCESS;
	}
	else
	return I2C_FAILURE;
}

/*------------------------------------------------------------------
wnI2cSetTransmitMode : This function is to set the transmission mode of I2C
Arguments:
	device - Pointer to structure
	mode - Mode of transmission
return : I2C_SUCCESS/I2C_FAILURE
------------------------------------------------------------------*/

wnBool wnI2cSetTransmitMode (wnI2CDeviceT *device, wnUInt32 mode)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_TAR;
	
	if(wnIsI2CEnabledDisabled(device) == WN_FALSE){
		i2c_instance->I2C_TAR &= ~(0x3 <<I2C_TRANSMIT_MODE_POS);
		reg_val |= (mode << I2C_TRANSMIT_MODE_POS);
		i2c_instance->I2C_TAR = reg_val;
		return I2C_SUCCESS;
	}
	else{
		return I2C_FAILURE;
	}
}

/*------------------------------------------------------------------
wnI2cMaskInterrupts : This function is to mask the interrupts of I2C
Arguments:
	device - Pointer to structure
	interrupt - interrupt to be masked enum
return : I2C_SUCCESS/I2C_FAILURE
------------------------------------------------------------------*/

wnBool wnI2cMaskInterrupts (wnI2CDeviceT *device, wnUInt32 interrupt )
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_INTR_MASK_REG;

	if(( reg_val&interrupt) != 0){
		reg_val &= ~interrupt;
		i2c_instance->I2C_INTR_MASK_REG = reg_val;
		return I2C_SUCCESS;
	}
	else
	I2C_FAILURE;
}

/*-----------------------------------------------------------------
wnI2cMaskInterrupts : This function is to Unmask the interrupts of I2C
Arguments:
	device - Pointer to structure
	interrupt - interrupt to be Unmasked enum
return : I2C_SUCCESS/I2C_FAILURE
-----------------------------------------------------------------*/

wnBool wnI2cUnMaskInterrupts (wnI2CDeviceT *device, wnUInt32 interrupt )
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;

	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_val = i2c_instance->I2C_INTR_MASK_REG;

	if((reg_val&interrupt) != interrupt ){
		reg_val |= interrupt;
		i2c_instance->I2C_INTR_MASK_REG = reg_val;
		return I2C_SUCCESS;
	}
	else
		return I2C_FAILURE;
}

/*----------------------------------------------------------------
wnI2cClearInterrupts : This function is to clear the interrupts of I2C
Arguments:
	device - Pointer to structure
	interrupt - interrupt to be cleared
----------------------------------------------------------------*/

wnVoid wnI2cClearInterrupts (wnI2CDeviceT *device, wnUInt32 interrupt)
{
	wnUInt32 reg_val = 0;
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	if(interrupt == WN_I2C_IRQ_ALL_E)
	reg_val = (i2c_instance->I2C_CLR_INTR & 0x1);
	else{
		if (interrupt == WN_I2C_IRQ_RX_UNDER_E)
			reg_val = ((i2c_instance->I2C_CLR_RX_UNDER) & 0x1);
		else if (interrupt == WN_I2C_IRQ_RXOVER_E)
			reg_val = ((i2c_instance->I2C_CLR_RX_OVER) & 0x1);
		else if (interrupt == WN_I2C_IRQ_TXOVER_E)
			reg_val = ((i2c_instance->I2C_CLR_TX_OVER) & 0x1);
		else if (interrupt == WN_I2C_IRQ_RDREQ_E)
			reg_val = ((i2c_instance->I2C_CLR_READ_REQ_INTR) & 0x1);
		else if (interrupt == WN_I2C_IRQ_TXABRT_E)
			reg_val = ((i2c_instance->I2C_CLR_TX_ABRT_INTR) & 0x1);
		else if (interrupt == WN_I2C_IRQ_RXDONE_E)
			reg_val = ((i2c_instance->I2C_CLR_RX_DONE) & 0x1);
		else if (interrupt == WN_I2C_IRQ_ACTIVITY_E)
			reg_val = ((i2c_instance->I2C_CLR_ACTIVITY) & 0x1);
		else if (interrupt == WN_I2C_IRQ_STOP_DET_E)
			reg_val = ((i2c_instance->I2C_CLR_STOP_DET) & 0x1);
		else if (interrupt == WN_I2C_IRQ_START_DET_E)
			reg_val = ((i2c_instance->I2C_CLR_START_DET) & 0x1);
		else if  (interrupt == WN_I2C_IRQ_GEN_CALL_E)
			reg_val = ((i2c_instance->I2C_CLR_GEN_CALL) & 0x1);
	}
}

/*----------------------------------------------------------------
wnI2cReadReq : This function is to raise a read request from master
Arguments:
	device - pointer to structure
----------------------------------------------------------------*/
wnVoid wnI2cReadReq (wnI2CDeviceT *device)
{
	wnI2CT *i2c_instance;

	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	i2c_instance->I2C_DATA_CMD = (0x1 << READ_REQ_POS);
}

/*----------------------------------------------------------------
wnI2cIsRawIrqActive : This function is to check the Raw IRQ status of an interrupt
Arguments
	device - pointer to structure
	interrupt - Interrupt whose status is to be checked
return - WN_TRUE/WN_FALSE
----------------------------------------------------------------*/
wnBool wnI2cIsRawIrqActive (wnI2CDeviceT *device, wnUInt32 interrupt)
{
	wnI2CT *i2c_instance;
	wnUInt32 reg_value = 0;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS ;
	reg_value = i2c_instance->I2C_RAW_INTR_STATUS;
	
	if((reg_value) & interrupt)
		return WN_TRUE;
	else
		return WN_FALSE;
}

/*----------------------------------------------------------------
wnI2cSetRxThreshold : This function is to set the threshold level of RX FIFO
Arguments:
	device - pointer to structure
	rx_buff_level - value of the RX buffer level to be set
----------------------------------------------------------------*/
wnVoid wnI2cSetRxThreshold (wnI2CDeviceT *device, wnUInt32 rx_buff_level)
{	
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	//	rx_buff_level -= 1;
	i2c_instance->I2C_RX_TL = rx_buff_level;
}

/*--------------------------------------------------------------------
wnI2cSetTxThreshold : This function is to set the threshold level of TX FIFO
Arguments:
	device - pointer to structure
	tx_buff_level - value of the TX buffer level to be set
--------------------------------------------------------------------*/
wnVoid wnI2cSetTxThreshold (wnI2CDeviceT *device, wnUInt32 tx_buff_level)
{
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	i2c_instance->I2C_TX_TL = tx_buff_level;	
}

/* -------------------------------------------------------------------                                                                
wnGetAbrtsource : This function is to get the source of transfer abort.
Arguments:
	device - pointer to the structure
return - Source of abort
--------------------------------------------------------------------*/	
wnUInt32 wnI2cGetAbrtsource (wnI2CDeviceT *device)
{
	wnI2CT *i2c_instance;
	wnUInt32 reg_val;
	wnUInt32 flushed_cmd_count;

	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	reg_val = (i2c_instance->I2C_TX_ABORT_SRC);

	return (reg_val & 0x1FFFF);;
}

/*----------------------------------------------------------------
wnIsTxAborted : This function is to check whether transfer is aborted or not
Arguments:
	device - pointer to structure
return - WN_TRUE/WN_FALSE
----------------------------------------------------------------*/
wnUInt32 wnI2cIsTxAborted (wnI2CDeviceT *device)
{
	wnI2CT *i2c_instance;
	
	i2c_instance = (wnI2CT *)device->BASE_ADDRESS;
	
	if(i2c_instance->I2C_TX_ABORT_SRC)
		return WN_TRUE;
	else
		return WN_FALSE;
}

/*---------------------------------------------------------------- 
wnI2CConfig - This function is to do the required configuraions for I2C 
Arguments:
	config_params - pointer to structure
return - I2c_SUCCESS/I2C_FAILURE
----------------------------------------------------------------*/
wnBool wnI2CConfig (wnI2CDeviceT *device, wnI2CConfigParametersT *config_params)
{
	if(wnI2cEnableDisable(device, DISABLE) == I2C_SUCCESS){
		
		wnI2cClearInterrupts (device, WN_I2C_IRQ_ALL_E);	
		device->I2C_CLOCK = I2C_CLK;
		wnI2CSetSpeed (device,config_params->SPEED);
		
		if(config_params->MASTER_MODE == I2C_MASTER_ENABLE)
			wnI2CSetMasterAddressMode (device,config_params->ADDRESSING_MODE);
		else 
			wnI2CSetSlaveAddressMode (device,config_params->ADDRESSING_MODE);
		
		wnI2cRestartEnableDisable (device,config_params->RESTART_EN);
		if(config_params->MASTER_MODE == I2C_MASTER_ENABLE)
			wnI2CMasterEnableDisable (device,config_params->MASTER_MODE);
		else
			wnI2CSlaveEnableDisable (device,config_params->SLAVE_MODE);
			
		wnI2cSetTransmitMode (device,config_params->TRANSMIT_MODE);
		
		if(config_params->TARGET_ADDRESS != 0x0){
			wnI2CSetTargetaddress (device,config_params->TARGET_ADDRESS);
			wnI2CSetSlaveAddress (device,I2C_SLAVE_ADDR);}
		if(config_params->SLAVE_ADDRESS != 0x0){
			wnI2CSetSlaveAddress (device,config_params->SLAVE_ADDRESS);
			wnI2CSetTargetaddress (device,I2C_TAR_ADDR);}
		
		wnI2cEnableDisable (device,ENABLE);
		
		return I2C_SUCCESS;
	}
	else
	return I2C_FAILURE;
}

/*-------------------------------------------------------------------------*/

