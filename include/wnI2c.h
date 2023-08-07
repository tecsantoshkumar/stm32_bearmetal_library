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
 
/*-----------------------------------------I2C---------------------------------------------*/
#ifndef __WN_I2C_DRIVER_H__
#define __WN_I2C_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "wn5gNrPsDataTypes.h"
#include "wisig.h"


#define I2C_CLK									SystemCoreClock		/*system clock*/

#define I2C_SLAVE_ENABLE_DISABLE_POS			  6					/* I2C slave enable/disable bit position */
#define I2C_RESTART_ENABLE_DISABLE_POS			  5					/* I2C restart enable/disable */
#define I2C_MASTER_ADDRESS_MODE_POS				  4					/* I2C master address mode bit position */
#define I2C_SLAVE_ADDRESS_MODE_POS				  3					/* I2C slave address mode bit position */
#define I2C_SPEED_POS							  1					/* i2C speed bit position */
#define I2C_MASTER_ENABLE_DISABLE_POS			  0					/* I2C master enable/disable position */

#define I2C_TRANSMIT_MODE_POS					 10					/* I2C transmit mode bit position*/

#define I2C_RX_FIFO_RFF_POS						  4					/* I2C RXFIFO RFF bit position*/
#define I2C_RX_FIFO_RFNE_POS					  3					/* I2C RXFIFO RFNE bit position*/
#define I2C_TX_FIFO_TFNF_POS					  1					/* I2C TXFIFO TFNF bit position*/
#define I2C_TX_FIFO_TFE_POS						  2					/* I2C TXFIFO TFE bit position*/

#define I2C_ENABLE_DISABLE_POS					  0					/* I2C enable/disable bit position*/
 
#define I2C_TX_BUFFER_DEPTH_POS					 16					/* I2C TX buffer depth position*/
#define I2C_RX_BUFFER_DEPTH_POS					  8					/* I2C RX buffer depth position */
#define I2C_HAS_DMA_POS							  6					/* I2C DMA on/off bit position */
#define I2C_MAX_SPEED_MODE_POS					  2					/* I2C speed mode parameter bit position*/
#define I2C_APB_DATA_WIDTH_POS					  0					/* I2C data width bit position*/

#define READ_REQ_POS							  8					/* I2C master data read request bit position */

#define I2C_TAR_ADDR							0x55				/* I2C default target address */
#define I2C_SLAVE_ADDR							0x55    			/* I2C default slave address */

/* macros for selecting address mode*/

#define I2C_7BITADDR 							 0x0				/* To enable 7-bit address mode */
#define I2C_10BITADDR 							 0x1				/* To enable 10-bit address mode */

/* macros for setting master enabled/disabled*/

#define I2C_MASTER_DISABLE 						 0x0				/* To enable I2C as master */
#define I2C_MASTER_ENABLE 						 0x1				/* To disable I2C as master */

/* macros for setting slave enabled/disabled*/
#define I2C_SLAVE_ENABLE 						 0x0				/* To enable I2C as slave */
#define I2C_SLAVE_DISABLE 						 0x1				/* To disable I2C as slave */

#define I2C_SUCCESS								 0x1				/* I2C success */
#define	I2C_FAILURE								 0x0				/* I2C failure */

/* macros for setting I2C enable/disable*/
#define DISABLE									 0x0				/* Disable I2C */
#define ENABLE									 0x1				/* Enable I2C */

/* macros for setting the transmission mode*/
#define I2c_TX_TARGET							 0x0				/*normal transfer using target address*/

/* macros for setting transmission speed mode*/
#define I2C_SPEED_STANDARD						 0x1				/*Standard Speed Mode*/
#define I2C_SPEED_FAST							 0x2				/*Fast Speed Mode*/
							

#ifdef I2C_CONFIG

#define CONFIG_CLK_CTRL_REG (*((volatile uint32_t* const) (ARM_S11VD_BASE + 0x0C)))
#define TST_ATB_SEL_REG     (*((volatile uint32_t* const) (ARM_D11VD_BASE + 0x10)))
#define CONFIG_IOMUX_REG    (*((volatile uint32_t* const) (ARM_S11VD_BASE + 0x14)))

#define MUX_I2C		{CONFIG_CLK_CTRL_REG = 0x1A3; \
						TST_ATB_SEL_REG	= 0x01;		\
						CONFIG_IOMUX_REG = 0x18;}

#define MUX_UART	{CONFIG_CLK_CTRL_REG = 0x1A1; \
						TST_ATB_SEL_REG	= 0x05;		\
						CONFIG_IOMUX_REG = 0x20;}
#endif	

typedef struct wnI2CConfigParameters{
	wnUInt32 ADDRESSING_MODE;
	wnUInt32 SPEED;
	wnUInt32 RESTART_EN;
	wnUInt32 MASTER_MODE;
	wnUInt32 SLAVE_MODE;
	wnUInt32 TRANSMIT_MODE;
	wnUInt32 TARGET_ADDRESS;
	wnUInt32 SLAVE_ADDRESS; 
}wnI2CConfigParametersT;

/* wnI2CirqE - enum for selecting the interrupt*/
typedef enum wnI2Cirq{
	WN_I2C_IRQ_NONE_E		= 0x000,
	WN_I2C_IRQ_RX_UNDER_E	= 0x001,
	WN_I2C_IRQ_RXOVER_E		= 0x002,
	WN_I2C_IRQ_RXFULL_E		= 0x004,
	WN_I2C_IRQ_TXOVER_E		= 0x008,
	WN_I2C_IRQ_TXEMPTY_E	= 0x010,
	WN_I2C_IRQ_RDREQ_E		= 0x020,
	WN_I2C_IRQ_TXABRT_E		= 0x040,
	WN_I2C_IRQ_RXDONE_E		= 0x080,
	WN_I2C_IRQ_ACTIVITY_E	= 0x100,
	WN_I2C_IRQ_STOP_DET_E	= 0x200,
	WN_I2C_IRQ_START_DET_E	= 0x400,
	WN_I2C_IRQ_GEN_CALL_E	= 0x800,
	WN_I2C_IRQ_ALL_E		= 0x8ff
}wnI2CirqE;

/*wnI2CT - structure to access the I2C configuration registers*/
typedef struct wnI2C
{
  __IO	wnUInt32 I2C_CTRL;											/* I2C control register */
  __IO	wnUInt32 I2C_TAR;											/* I2C target address register */
  __IO	wnUInt32 I2C_SAR;											/* I2C slave address register*/	
		wnUInt32 IDLE_REG_1;                                        /* Not used or Reserved */
  __IO	wnUInt32 I2C_DATA_CMD;		  								/* I2C data command register */
  __IO	wnUInt32 I2C_SS_SCL_HCNT;									/* I2C high count register */
  __IO	wnUInt32 I2C_SS_SCL_LCNT;									/* I2C low count register */
  __IO	wnUInt32 I2C_FS_SCL_HCNT;									/* I2C fast speed high count register */
  __IO	wnUInt32 I2C_FS_SCL_LCNT;									/* I2C fast speed low count register */
		wnUInt32 IDLE_REG_2;										/* Not used or Reserved */
		wnUInt32 IDLE_REG_3;
  __I	wnUInt32 I2C_INTR_STATUS_REG;								/* I2C interrupt status register */
  __IO	wnUInt32 I2C_INTR_MASK_REG;  								/* I2C interrupt mask register */
  __IO	wnUInt32 I2C_RAW_INTR_STATUS;								/* I2C raw interrupt status register */
  __IO	wnUInt32 I2C_RX_TL;											/* I2C RX FIFO level register */
  __IO	wnUInt32 I2C_TX_TL;											/* I2C TX FIFo level register */
  __I	wnUInt32 I2C_CLR_INTR;       								/* I2C clear all interrupt register */
  __I	wnUInt32 I2C_CLR_RX_UNDER;									/* I2C clear rx under interrupt register */
  __I	wnUInt32 I2C_CLR_RX_OVER;									/* I2C clear rx over interrupt register */
  __I	wnUInt32 I2C_CLR_TX_OVER;									/* I2C clear tx over interrupt register */
  __I	wnUInt32 I2C_CLR_READ_REQ_INTR; 							/* I2C clear read request interrupt register */
  __I	wnUInt32 I2C_CLR_TX_ABRT_INTR;								/* I2C clear TX abort interrupt register */
  __I	wnUInt32 I2C_CLR_RX_DONE;									/* I2C clear RX done interrupt register */
  __I	wnUInt32 I2C_CLR_ACTIVITY;									/* I2C clear activity interrupt register */
  __I	wnUInt32 I2C_CLR_STOP_DET;        							/* I2C clear stop detection interrupt register */
  __I	wnUInt32 I2C_CLR_START_DET;									/* I2C clear start detection interrupt register */
  __I	wnUInt32 I2C_CLR_GEN_CALL;									/* I2C clear general call interrupt register */
  __IO	wnUInt32 I2C_ENABLE;										/* I2C enable register */
  __I	wnUInt32 I2C_STATUS;										/* I2C status register */
  __I	wnUInt32 I2C_TX_FIFO_LEVEL;									/* I2C TX FIFO level register */
  __I	wnUInt32 I2C_RX_FIFO_LEVEL;									/* I2C RX FIFO level register */
  __IO	wnUInt32 I2C_SDA_HOLD;										/* I2C SDA hold time length register */
  __I	wnUInt32 I2C_TX_ABORT_SRC;									/* I2C transmission abort source register */
		wnUInt32 IDLE_REG_4;										/* Not used or Reserved */
		wnUInt32 IDLE_REG_5;										/* Not used or Reserved */
		wnUInt32 IDLE_REG_6;										/* Not used or Reserved */
		wnUInt32 IDLE_REG_7;										/* Not used or Reserved */
  __IO	wnUInt32 I2C_SDA_SETUP;										/* I2C SDA setup register */
  __IO	wnUInt32 I2C_ACK_GEN_CALL;									/* I2C acknowledgment general call register */
  __I	wnUInt32 I2C_ENABLE_STATUS;									/* I2C enable status register */
  __IO	wnUInt32 I2C_FS_SPKLEN;										/* I2C speed spike suppression limit register*/
		wnUInt32 IDLE_REG_8[19];									/* Not used or Reserved */
  __IO	wnUInt32 I2C_REG_TIMEOUT;									/* I2C register timeout counter reset value register */
  __I	wnUInt32 I2C_COMP_PARAM1;									/* I2C hardware component parameters register */
  __I	wnUInt32 I2C_COMP_VERSION;									/* I2C component version register */
  __I	wnUInt32 I2C_COMP_TYPE;									    /* I2C component type register */
}wnI2CT;

/*wnI2CDeviceT - structure to store the individual device information */
typedef struct wnI2CDevice{
	const wnChar *DEVICE_NAME;									/* device name */
	wnUInt32 DATA_WIDTH; 										/* data width */
	wnUInt32 BASE_ADDRESS;										/* I2C BaseAddresses */
	wnVoid *COMP_PARAMETERS;									/* component parameters */
	wnUInt32 I2C_CLOCK;											/* I2C clock value*/
}wnI2CDeviceT;

/* Base Addresses of I2C_0 and I2C_1 */
#define I2C_0_BASE_ADDR 						0X50002000		/* I2C0 base address*/
#define I2C_1_BASE_ADDR  						0x50003000		/* I2C1 base address*/

/*  WISIG_I2C Driver API's */

/* wnI2Csetaddress - Load I2C target address */
wnVoid wnI2CSetTargetaddress(wnI2CDeviceT *device, wnUInt32 i2c_addr);

/* wnI2Csetaddress - Load I2C slave address */
wnVoid wnI2CSetSlaveAddress(wnI2CDeviceT *device, wnUInt32 i2c_addr);

/* wnIsI2CBusy : To check whether I2C is busy or not*/
wnBool wnIsI2CBusy(wnI2CDeviceT *device);

/* wnI2CMasterEnableDisable - I2C master enable and disable */
wnVoid wnI2CMasterEnableDisable(wnI2CDeviceT *device, wnUInt32 i2c_enable_disable);

/* wnI2CSlaveEnableDisable - I2C slave enable and disable */
wnVoid wnI2CSlaveEnableDisable(wnI2CDeviceT *device, wnUInt32 enable_disable);

/* wnI2CSetSpeed - Setting the speed for I2C transactions */
wnBool wnI2CSetSpeed(wnI2CDeviceT *device, wnUInt32 i2c_speed);

/* wnI2CSetMasterAddressMode - Setting 7-bit or 10-bit addressing mode for master */
wnVoid wnI2CSetMasterAddressMode(wnI2CDeviceT *device, wnUInt32 mode);

/* wnI2CSetMasterAddressMode - Setting 7-bit or 10-bit addressing mode for slave */
wnVoid wnI2CSetSlaveAddressMode(wnI2CDeviceT *device, wnUInt32 mode);

/* wnI2CConfig - To do the required configuraions for I2C */
wnBool wnI2CConfig(wnI2CDeviceT *device,wnI2CConfigParametersT *config_params);

/* wnI2CWrite - I2C write data to be transmitted on the bus */
wnVoid wnI2CWrite(wnI2CDeviceT *device, wnUInt8 data);

/* wnI2Cread - I2C read data to be received from the bus */
wnUInt8 wnI2CRead(wnI2CDeviceT *device);

/* wnI2cIsRxFifoFull : To check whether RX FIFO is full or not */
wnBool wnI2cIsRxFifoFull(wnI2CDeviceT *device);

/* wnI2cIsRxFifoEmpty : To check whether RX FIFO is Empty or not*/
wnBool wnI2cIsRxFifoEmpty(wnI2CDeviceT *device);

/* wnI2cIsTxFifoFull : To check whether TX FIFO is full or not*/
wnBool wnI2cIsTxFifoFull(wnI2CDeviceT *device);

/* wnI2cIsTxFifoEmpty : To check whether TX FIFO is empty or not*/
wnBool wnI2cIsTxFifoEmpty(wnI2CDeviceT *device);

/* wnI2cRestartEnableDisable : To Enable or Disable the Restart of I2C*/
wnVoid wnI2cRestartEnableDisable(wnI2CDeviceT *device, wnUInt32 enableDisable);

/* wnIsI2CEnabledDisabled : To check whether I2C is enabled or disabled*/
wnBool wnIsI2CEnabledDisabled(wnI2CDeviceT *device);

/*  wnI2cEnableDisable : To enable or disable the I2C*/
wnBool wnI2cEnableDisable(wnI2CDeviceT *device, wnUInt32 enablediasble);

/* wnI2cSetTransmitMode : To set the transmission mode of I2C*/
wnBool wnI2cSetTransmitMode(wnI2CDeviceT *device, wnUInt32 mode);

/* wnI2cMaskInterrupts : To mask the interrupts*/
wnBool wnI2cMaskInterrupts(wnI2CDeviceT *device, wnUInt32 interrupt );

/* wnI2cUnMaskInterrupts : To unmask the interrupts*/
wnBool wnI2cUnMaskInterrupts(wnI2CDeviceT *device, wnUInt32 interrupt );

/* wnI2cClearInterrupts : To clear the interrupts*/
wnVoid wnI2cClearInterrupts(wnI2CDeviceT *device, wnUInt32 interrupt);

/* wnI2cReadReq : To make master request data from the slave as a receiver */
wnVoid wnI2cReadReq(wnI2CDeviceT *device);

/* wnI2cIsRawIrqActive : To check the status of the status of the interrupt */
wnBool wnI2cIsRawIrqActive(wnI2CDeviceT *device,wnUInt32 interrupt);

/* wnI2cSetRxThreshold : To set the threshold level of RX FIFO */
wnVoid wnI2cSetRxThreshold(wnI2CDeviceT *device,wnUInt32 rx_buff_level);

/* wnI2cSetTxThreshold : To set the threshold level of TX FIFO */
wnVoid wnI2cSetTxThreshold(wnI2CDeviceT *device,wnUInt32 tx_buff_level);

/* wnGetAbrtsource : To get the source of transfer abort */
wnUInt32 wnI2cGetAbrtsource(wnI2CDeviceT *device);

/* wnIsTxAborted : To check whether transfer is aborted or not */
wnUInt32 wnI2cIsTxAborted(wnI2CDeviceT *device);

#endif


