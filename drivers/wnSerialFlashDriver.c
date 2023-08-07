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

#include "wnSerialFlashDriver.h"
#include "wnSerialFlashDriver_Hw.h"
#include <stdio.h>

wnInt32 Qspi_init(wnQspiT *QSPI, wnUInt8 mode, wnUInt32 qspiClkDivisorValue, wnUInt8 spiLane )
{	
	if (QSPI == NULL)
		return WN_QSPI_INVALID;
	
	if(WN_QSPI_INVALID != Qspi_ClkCntrl(QSPI, qspiClkDivisorValue))
	{	
		if(WN_QSPI_INVALID != Qspi_Config(QSPI, mode, spiLane)) 
			return WN_QSPI_SUCCESS;
	}
	return WN_QSPI_INVALID;
}

wnInt32 Flash_Write(wnQspiT *QSPI, wnUInt32 address, wnUInt32 byteCount,wnUInt8 *srcBuff, wnUInt8 spiLane)
{		
	wnUInt32 size;
	wnUInt32 page_offset;
	
	if( (address + byteCount) > 0x3FFFFF )
		return WN_QSPI_INVALID;
		
	do
	{	
		page_offset = address & 0xFF;
			
		if ( byteCount > PAGE_SIZE )				
			size = PAGE_SIZE;
		else
			size =  byteCount;
				
		if((page_offset + size) > PAGE_SIZE)
				size = PAGE_SIZE - page_offset;
	
		Page_Program(QSPI, address, size, srcBuff, 0b00);
			
		byteCount = ( byteCount - size );	
		address   = ( address + size ) ;
		srcBuff   = ( srcBuff + size );
			
	} while (byteCount > 0);	
		
	return WN_QSPI_SUCCESS;	
		
}

wnInt32 Page_Program(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 byteCount, wnUInt8 *srcBuff, wnUInt8 spiLane)
{
	
	wnUInt32 cntrlVal =0;
	
	if (QSPI == NULL)
		return WN_QSPI_INVALID;
	
	Write_Enable(QSPI);
	
	cntrlVal = ( ( 0x0004 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
				  ( 0x1 << WN_QSPI_CR_CMD_INIT_POS ) |
				  ( spiLane << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
			      ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
                  ( 0x0 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
                  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
                 );
   		
	QSPI->SPI_CTRL = cntrlVal;
		
	wnUInt8 mem3 = ((addr & 0x00FF0000) >> 16); //MSB Address (a16-a23)  
    wnUInt8 mem2 = ((addr & 0x0000FF00) >> 8); // middle byte
    wnUInt8 mem1 = ( addr & 0x000000FF); //LSB byte 

	QSPI->SPI_TX_DATA = ((mem1<<24) | (mem2 << 16) | (mem3 << 8 ) | OP_PAGE_PROGRAM );
	
  for ( int i=0; i < byteCount; i++)
   {
       if (i == (byteCount-1)){
		   		
		 	cntrlVal = 0;
	
			cntrlVal = ( ( 0x0001 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
				  ( 0x1 << WN_QSPI_CR_CMD_INIT_POS ) |
				  ( spiLane << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
			      ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
                  ( 0x1 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
                  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
                   );
                      		
			QSPI->SPI_CTRL = cntrlVal; 	
		}
		else
		{ 
		   	cntrlVal = 0;
	
			cntrlVal = ( ( 0x0001 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
				  ( 0x1 << WN_QSPI_CR_CMD_INIT_POS ) |
				  ( spiLane << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
			      ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
                  ( 0x0 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
                  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
                   );
                      		
			QSPI->SPI_CTRL = cntrlVal;
		
		}
	   
	   for ( int i=0; i < 0xFFFF; i++);
	   QSPI->SPI_TX_DATA = srcBuff[i]; // Data of 1-Bytes   	   
	 }

 }

wnInt32 Flash_Read(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 byteCount, wnUInt8 *destBuff, wnUInt8 spiLane)
{
	wnUInt32 cntrlVal = 0;

	if (QSPI == NULL)
		return WN_QSPI_INVALID;
			
        cntrlVal = ( ((0x0004 + (spiLane == 2? 1 : 0)) << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
					  ( 0x1 << WN_QSPI_CR_CMD_INIT_POS ) | 
					  ( (spiLane & 0) << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
					  ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
					  ( 0x0 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
					  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
					);

			
	QSPI->SPI_CTRL = cntrlVal;
	
	wnUInt8 mem3 = ((addr & 0x00FF0000) >> 16); //MSB Address (a16-a23)  
	wnUInt8 mem2 = ((addr & 0x0000FF00) >> 8); // middle byte
	wnUInt8 mem1 = ( addr & 0x000000FF); //LSB byte		
		
    QSPI->SPI_TX_DATA = ((mem1<<24) | (mem2 << 16) | (mem3 << 8 ) | (spiLane == 2? OP_FAST_READ_QUAD : OP_READ_DATA) );
    if (spiLane == 2)
        QSPI->SPI_TX_DATA = 0; // ensure dummy data byte

	for ( int i=0; i < byteCount; i++)
	{
       if (i == (byteCount-1)){
		   		
		 	cntrlVal = 0;
	
			cntrlVal = ( ( 0x0001 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
						  ( 0x2 << WN_QSPI_CR_CMD_INIT_POS ) |
						  ( spiLane << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
						  ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
						  ( 0x1 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
						  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
						);
                      		
			QSPI->SPI_CTRL = cntrlVal; 	
		}
		else
		{ 
		   	cntrlVal = 0;
	
			cntrlVal = ( ( 0x0001 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
				  ( 0x2 << WN_QSPI_CR_CMD_INIT_POS ) |
				  ( spiLane << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
			      ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
                  ( 0x0 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
                  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
                   );
                      		
			QSPI->SPI_CTRL = cntrlVal;	
		}
				
		while( !(QSPI->SPI_INT_STATUS & ( 1 <<5 )));
		
		destBuff[i]= QSPI->SPI_RX_DATA ; 
	 }		 
}

wnInt32 Qspi_Config(wnQspiT *QSPI, wnUInt8 mode, wnUInt8 spiLane )
{
	wnUInt32 configval = 0;
	wnUInt32 cntrlval1 = 0;
	wnUInt32 cntrlval2 = 0;
	wnUInt32 bchval = 0;
    
	if (QSPI == NULL)
		return WN_QSPI_INVALID;

	if (mode == SPI_MODE_PIO) {
		
		configval = ((mode << WN_QSPI_CFG_MODE_SELECT_POS) | 
				     (0x1 << WN_QSPI_CFG_CPHA_POS) | 
				     (0x1 << WN_QSPI_CFG_CPOL_POS) | 
					 (0x0 << WN_QSPI_CFG_LSBL_POS) | 
				     (0x0 << WN_QSPI_CFG_SOFT_RESET_POS) |  
					 (spiLane << WN_QSPI_CFG_SPI_LANES_POS) | 
				     (0x1 << WN_QSPI_CFG_SPI_ENABLE_POS) | 
					 (0x1 << WN_QSPI_CFG_SPI_CS_POS));
				 
		QSPI->SPI_CONFIG = configval;
	}
		
	else if (mode == SPI_MODE_XIP)	{
                    
        cntrlval1 = (((spiLane == 2? OP_FAST_READ_QUAD : OP_READ_DATA) << WN_QSPI_XIP_CR1_READ_OPCODE_POS) | 
             (0x3 << WN_QSPI_XIP_CR1_READ_ADDRBYTE_POS) | 
             (((spiLane == 2)? 0x1 : 0) << WN_QSPI_XIP_CR1_DUMMY_BYTES_POS) | 
             (spiLane << WN_QSPI_XIP_CR1_TYPE_DATA_POS));                      
	
		QSPI->XIP_CTRL1 = cntrlval1;

		cntrlval2 = (0x2 << WN_QSPI_XIP_CR2_PAGESIZE_POS);  
		QSPI->XIP_CTRL2 = cntrlval2;

		bchval = (0x7 << WN_QSPI_BCH_CR0_CODESIZE_POS); 	
		QSPI->SPI_BCH_CTRL0 = bchval;
	
		configval = (( mode << WN_QSPI_CFG_MODE_SELECT_POS) | ( 0x0 << WN_QSPI_CFG_CPHA_POS) | 
					 ( 0x1 << WN_QSPI_CFG_CPOL_POS) | ( 0x0 << WN_QSPI_CFG_LSBL_POS) | 
					 ( 0x1 << WN_QSPI_CFG_AHB_BURST_INCR_EN_POS) | ( 0x1<< WN_QSPI_CFG_AHB_BURST_INCR4_EN_POS) | 
					 ( 0x1 << WN_QSPI_CFG_AHB_BURST_INCR8_EN_POS) | ( 0x1 << WN_QSPI_CFG_AHB_BURST_INCR16_EN_POS) |
					 ( 0x0 << WN_QSPI_CFG_SOFT_RESET_POS) |  ( spiLane << WN_QSPI_CFG_SPI_LANES_POS) | 
					 ( 0x1 << WN_QSPI_CFG_SPI_ENABLE_POS) | (0x1 << WN_QSPI_CFG_SPI_CS_POS));
					 
		QSPI->SPI_CONFIG = configval; 
	}
	else
		return WN_QSPI_INVALID;
	
	return WN_QSPI_SUCCESS;
}

wnInt32 Qspi_ClkCntrl(wnQspiT *QSPI,wnUInt32 qspiClkDivisorValue )
{
	if (QSPI == NULL)
		return WN_QSPI_INVALID;
		
	QSPI->SPI_CLK_CTRL = 0;
	
	QSPI->SPI_CLK_CTRL = ((WN_QSPI_INTERNAL_CLK_ENABLE << WN_QSPI_INTERNAL_CLK_ENABLE_POS )|
							(qspiClkDivisorValue << WN_QSPI_INTERNAL_CLK_DIVISOR_VALUE_POS));
	
	return WN_QSPI_SUCCESS;
}

wnInt32	Qspi_Cmnd_thrshld(wnQspiT *QSPI, wnUInt32 txbytecount, wnUInt32 rxbytecount)
{
	if (QSPI == NULL)
		return WN_QSPI_INVALID;
		
	QSPI->SPI_CMND_THRSHLD = (txbytecount << 16);
	QSPI->SPI_CMND_THRSHLD |= (rxbytecount << 0);
	
	return WN_QSPI_SUCCESS;
}

wnInt32 Write_Enable(wnQspiT *QSPI)
{
	wnUInt32 cntrlVal;
	
	if (QSPI == NULL)
		return WN_QSPI_INVALID;
	
	cntrlVal = ( ( 0x1 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
				  ( 0x1 << WN_QSPI_CR_CMD_INIT_POS ) |
				  ( 0x0 << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
			      ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
                  ( 0x1 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
                  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
                   );

	QSPI->SPI_CTRL =  cntrlVal;	
	
	QSPI->SPI_TX_DATA = OP_WRITE_ENABLE; // for WRITE ENABLE
	
	return WN_QSPI_SUCCESS;
}

wnInt32 Sector_Erase(wnQspiT *QSPI, wnUInt32 addr, wnUInt32 eraseSize)
{
	wnUInt8 Command;
	wnUInt32 cntrlVal = 0;	

	if (QSPI == NULL)
		return WN_QSPI_INVALID;
		

	if( ! (addr + (eraseSize - 1) <= 0x3FFFFF) )		
		return WN_QSPI_INVALID;

	switch(eraseSize)
	{
		case Size_4K:
		Command = SECTOR_ERASE_4K;
		break;
		
		case Size_32K :
		Command = SECTOR_ERASE_32K;
		break;
		
		case Size_64K :
		Command = SECTOR_ERASE_64K;
		break;
		
		default:
		//printf("\r\n Invalid Erase Size\n");
		break;
	}

	cntrlVal = ( ( 0x0004 << WN_QSPI_CR_SPI_TxRx_COUNT_POS ) |
				  ( 0x1 << WN_QSPI_CR_CMD_INIT_POS ) |
				  ( 0x0 << WN_QSPI_CR_SPI_LANE_MODE_POS ) |
			      ( 0x0 << WN_QSPI_CR_DEV_SEL1_POS ) |
                  ( 0x1 << WN_QSPI_CR_CHIP_DEASSERT_POS ) |
                  ( 0x0 << WN_QSPI_CR_SDR_DDR_POS ) 
                );

   	QSPI->SPI_CTRL = cntrlVal;	
 
	wnUInt8 mem3 = ((addr & 0x00FF0000) >> 16); //MSB Address (a16-a23) 
   	
    wnUInt8 mem2 = ((addr & 0x0000FF00) >> 8); // middle byte
    wnUInt8 mem1 = ( addr & 0x000000FF); //LSB byte   
    
	QSPI->SPI_TX_DATA = ((mem1<<24) | (mem2 << 16) | (mem3 << 8 ) | Command );
	
	return WN_QSPI_SUCCESS;
}
