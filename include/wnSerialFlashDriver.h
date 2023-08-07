/**************************************************************************/
/*                                                                        */
/* The content of this file or document is CONFIDENTIAL and PROPRIETRY    */
/* to  Adeptchips Services Pvt Ltd. It  is  subject  to  the terms of a   */
/* License Agreement  between Licensee and  Apeptchips Services Pvt Ltd.  */
/* restricting  among other things, the use, reproduction, distribution   */
/* and transfer. Each of the embodiments, including this information and  */
/* any derivative work shall retain this copyright notice                 */
/*                                                                        */
/* Copyright Adeptchips Services Pvt Ltd.                                 */
/* All rights reserved                                                    */
/*                                                                        */
/**************************************************************************/

#ifndef WNSERIALFLASHDRIVER_H_
#define WNSERIALFLASHDRIVER_H_

#include "wn5gNrPsDataTypes.h"

#define PAGE_SIZE					256

#define OP_READ_MANUF_DEVICE_ID 	0x90
#define OP_WRITE_ENABLE  			0x06
#define OP_PAGE_PROGRAM  			0x02
#define OP_READ_DATA  				0x03
#define OP_SECTOR_ERASE 			0x20
#define OP_FAST_READ_DATA  			0x0B
#define OP_FAST_READ_QUAD  			0x6B

#define SECTOR_ERASE_4K				0x20
#define SECTOR_ERASE_32K 			0x52
#define SECTOR_ERASE_64K			0xD8

#define Size_32K					0x8000
#define Size_64K 					0x10000
#define Size_4K 					0x1000

#define SPI_LANE_SINGLE				0x0
#define SPI_LANE_DUAL				0x1
#define SPI_LANE_QUAD				0x2

#define SPI_MODE_BOOT 				0x0 
#define SPI_MODE_PIO 				0x1
#define SPI_MODE_DMA				0x2
#define SPI_MODE_XIP				0x3


#define QSPI_BASE  ((wnQspiT *)WN_QSPI_BASEADDR)

typedef struct {		
	wnUInt32 XIP_CTRL1;
	wnUInt32 XIP_CTRL2;
	wnUInt32 SPI_CONFIG;
	wnUInt32 SPI_CTRL;
	wnUInt32 SPI_CLK_CTRL;
	wnUInt32 SPI_CMND_THRSHLD;
	wnUInt32 SPI_INT_THRSHLD;
	wnUInt32 SPI_INT_ENABLE;
	wnUInt32 SPI_INT_STATUS;
	wnUInt32 SPI_TX_DATA;
	wnUInt32 SPI_RX_DATA;
	wnUInt32 SPI_STATUS1;
	wnUInt32 SPI_STATUS2;
	wnUInt32 BD_CTRL;
	wnUInt32 BD_CURR_ADDR;
	wnUInt32 Reserved0;
	wnUInt32 BD_BASE_ADDR;
	wnUInt32 BD_STATUS;
	wnUInt32 BD_POLL_CTRL;
	wnUInt32 BD_TX_DMA_STATUS;
	wnUInt32 BD_RX_DMA_STATUS;
	wnUInt32 SPI_CTRL_THRSHLD;
	wnUInt32 SPI_INT_SIG_ENABLE;
	wnUInt32 SPI_TAP_CTRL;
	wnUInt32 SPI_STATUS_CTRL;
	wnUInt32 SPI_XIP_CTRL3;
	wnUInt32 SPI_XIP_CTRL4;
	wnUInt32 SPI_BCH_CTRL0;
	wnUInt32 SPI_BCH_STATUS;
	wnUInt32 SPI_XIP_CTRL5;
	wnUInt32 Reserved1;
	wnUInt32 Reserved2;
	wnUInt32 SPI_BSTRAP_REG1;
	wnUInt32 SPI_BSTRAP_REG2;
	wnUInt32 SPI_BSTRAP_REG3;
	wnUInt32 SPI_BSTRAP_REG4;
	wnUInt32 SPI_BSTRAP_REG5;
	wnUInt32 SPI_BSTRAP_REG6;
	wnUInt32 SPI_BSTRAP_REG7;
	wnUInt32 SPI_REV_ID_REG;
} wnQspiT;


typedef enum wnQspi{

	WN_QSPI_INVALID 	= -1,
	WN_QSPI_SUCCESS 	= 0,
	WN_QSPI_IS_READY 	= 1,
	WN_QSPI_IS_BUSY 	= 2
} wnQspiE;

wnInt32 Qspi_Config(wnQspiT *QSPI, wnUInt8 mode, wnUInt8 spiLane);
wnInt32 Qspi_ClkCntrl(wnQspiT *QSPI,wnUInt32 qspiClkDivisorValue );
wnInt32	Qspi_Cmnd_thrshld(wnQspiT *QSPI, wnUInt32 txbytecount, wnUInt32 rxbytecount);
wnInt32 Write_Enable(wnQspiT *QSPI);
wnInt32 Sector_Erase(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 eraseSize);

wnInt32 Qspi_init(wnQspiT *QSPI, wnUInt8 mode, wnUInt32 qspiClkDivisorValue, wnUInt8 spiLane);
wnInt32 Page_Program(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 byteCount, wnUInt8 *srcBuff, wnUInt8 spiLane);
wnInt32 Flash_Read(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 byteCount, wnUInt8 *destBuff, wnUInt8 spiLane );
wnInt32 Flash_Write(wnQspiT *QSPI, wnUInt32 address, wnUInt32 byteCount,wnUInt8 *srcBuff, wnUInt8 spiLane);

wnInt32 Flash_FastRead(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 byteCount, wnUInt8 *destBuff, wnUInt8 spiLane);

#endif
