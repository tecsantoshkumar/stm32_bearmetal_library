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

#ifndef WNSERIALFLASHDRIVER_HW_H_
#define WNSERIALFLASHDRIVER_HW_H_

#include "wn5gNrPsDataTypes.h"


/** QSPI BASE Address **/
#define WN_QSPI_BASEADDR      					0x32000000U


	#define  WN_QSPI_CFG_MODE_SELECT_POS  		   	0     /** MODE SELECT **/	
	#define  WN_QSPI_CFG_CPHA_POS				   	3	  /** Clock Phase **/
	#define  WN_QSPI_CFG_CPOL_POS				   	4	  /** Clock Polarity **/
	#define  WN_QSPI_CFG_LSBL_POS				   	5	  /** DATA Format **/
	#define  WN_QSPI_CFG_WP_POS					   	9	  /** Write Protect**/
	#define  WN_QSPI_CFG_HOLD_POS				   	10	  /** Hold **/
	#define  WN_QSPI_CFG_AHB_BURST_INCR_EN_POS	   	11	  /** MasterBurst capability**/
	#define  WN_QSPI_CFG_AHB_BURST_INCR4_EN_POS	   	12	  /** MasterBurst capability by 4**/
	#define  WN_QSPI_CFG_AHB_BURST_INCR8_EN_POS	   	13	  /** MasterBurst capability  by 8**/
	#define  WN_QSPI_CFG_AHB_BURST_INCR16_EN_POS   	14	  /** MasterBurst capability  by 16**/
	#define  WN_QSPI_CFG_SOFT_RESET_POS			   	16	  /** Software Reset **/
	#define  WN_QSPI_CFG_SPI_LANES_POS			   	20	  /** Max DATA Lanes**/
	#define  WN_QSPI_CFG_SPI_ENABLE_POS			   	23	  /** SPI Enable**/
	#define  WN_QSPI_CFG_SPI_CS_POS				   	24	  /** SPI Chip Select**/
		
	#define  WN_QSPI_CR_SPI_TxRx_COUNT_POS  	   	0     /** Total no.of bytes to Transmit/Receive  **/	
	#define  WN_QSPI_CR_CMD_INIT_POS  			   	16     /** CMD INIT  **/
	#define  WN_QSPI_CR_SPI_LANE_MODE_POS  		   	18     /** SPI Lane Mode Single/Dual/Quad  **/
	#define  WN_QSPI_CR_DEV_SEL1_POS  			   	20     /** SPI DEV Select[1]  **/
	#define  WN_QSPI_CR_CHIP_DEASSERT_POS  		   	22     /** Chip Select Assert **/
	#define  WN_QSPI_CR_SDR_DDR_POS  			   	23     /** SDR/DDR **/
	#define  WN_QSPI_CR_STATUS_POS  			   	24     /** Status Check **/
	#define  WN_QSPI_CR_DEV_SEL2_POS  			   	25     /** SPI DEV Select[2]  **/
	#define  WN_QSPI_CR_ECC_CAL_EN_POS  		   	26     /** ECC_CAL_EN   **/
	#define  WN_QSPI_CR_ECC_ACCESS_EN_POS  		   	27     /** ECC_ACCESS_EN   **/
	#define  WN_QSPI_CR_RX_ECC_CAL_START_POS  	   	28     /** RX_ECC_CAL_START  **/


	/** QSPI Clock Divisor Values, from 0 to Base Clock divided by 2048 **/

	#define WN_QSPI_CLK_DIVISOR_VALUE_0		        0x000 	/** Base Clock- Highest Freq of SPI CLK**/
	#define WN_QSPI_CLK_DIVISOR_VALUE_2		        0x001 	/** Base Clock divided by 2    **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_4			    0x002 	/** Base Clock divided by 4	   **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_8			    0x004	/** Base Clock divided by 8	   **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_16		    0x008 	/** Base Clock divided by 16   **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_32		    0x010 	/** Base Clock divided by 32   **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_64		    0x020 	/** Base Clock divided by 64   **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_128		    0x040 	/** Base Clock divided by 128  **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_256		    0x080 	/** Base Clock divided by 256  **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_512		    0x100 	/** Base Clock divided by 512  **/
	#define WN_QSPI_CLK_DIVISOR_VALUE_1024		    0x200 	/** Base Clock divided by 1024 */

	#define WN_QSPI_CLK_DIVISOR_VALUE_2048		    0x400 	/** Base Clock divided by 2048 **/

	#define WN_QSPI_INTERNAL_CLK_ENABLE				0x1  
	#define  WN_QSPI_INTERNAL_CLK_STABLE			0x1

	#define WN_QSPI_INTERNAL_CLK_ENABLE_POS 		0		
	#define WN_QSPI_INTERNAL_CLK_STABLE_POS			1 
	#define WN_QSPI_INTERNAL_CLK_DIVISOR_VALUE_POS	8	
	
		
	#define WN_QSPI_XIP_CR1_TYPE_CMND_POS   			0 
	#define WN_QSPI_XIP_CR1_TYPE_ADDR_POS				2
	#define WN_QSPI_XIP_CR1_TYPE_MODE_POS				4
	#define WN_QSPI_XIP_CR1_TYPE_DUMMY_POS				6
	#define WN_QSPI_XIP_CR1_TYPE_DATA_POS				8
	#define WN_QSPI_XIP_CR1_READ_OPCODE_POS				10
	#define WN_QSPI_XIP_CR1_READ_ADDRBYTE_POS			18
	#define WN_QSPI_XIP_CR1_DUMMY_BYTES_POS				21
	#define WN_QSPI_XIP_CR1_TYPE_CMND_DDR_POS			24 
	#define WN_QSPI_XIP_CR1_TYPE_ADDR_DDR_POS			25
	#define WN_QSPI_XIP_CR1_TYPE_MODE_DDR_POS			26
	#define WN_QSPI_XIP_CR1_TYPE_DUMMY_DDR_POS			27
	#define WN_QSPI_XIP_CR1_TYPE_DATA_DDR_POS			28
	#define WN_QSPI_XIP_CR1_TYPE_CMND_SDR2DDR_POS		29
	
	  
	#define WN_QSPI_XIP_CR2_MODE_CODE_POS   		0
	#define WN_QSPI_XIP_CR2_MODE_BYTES_POS			8
	#define WN_QSPI_XIP_CR2_DEV_SEL					10
	#define WN_QSPI_XIP_CR2_ECC_ON_OFF_POS			13
	#define WN_QSPI_XIP_CR2_ECC_OFFSET_POS			14
	#define WN_QSPI_XIP_CR2_NAND_POS				22
	#define WN_QSPI_XIP_CR2_PLANE					23
	#define WN_QSPI_XIP_CR2_PAGESIZE_POS			25

	#define WN_QSPI_XIP_CR3_INT1_CODE1_POS   		0
	#define WN_QSPI_XIP_CR3_INT2_CODE2_POS			8
	#define WN_QSPI_XIP_CR3_INT3_CODE3_POS			16
	#define WN_QSPI_XIP_CR3_TYPE_INT1_POS			24
	#define WN_QSPI_XIP_CR3_INT1_CNT_POS			26
	#define WN_QSPI_XIP_CR3_INT1_STS_CHK_POS		28

	#define WN_QSPI_BCH_CR0_BCHN_POS				0
	#define WN_QSPI_BCH_CR0_BCHT_POS				16
	#define WN_QSPI_BCH_CR0_CODESIZE_POS			20
	 
	#define WN_QSPI_BCH_STS_ERR_CNT_POS				0
	#define WN_QSPI_BCH_STS_PAGE_ERR_CNT_POS		16
	

#endif
