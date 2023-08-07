
#include <stdio.h>
#include "qspi.h"


//Command with address format (32 bit)
//-------------------------------------
//  4         3        1        0
//---------------------------------
//|ADDR0-7| ADDR8-15|ADDR16-24|CMD|
//---------------------------------
#define BYTE1(addr) (((addr) & 0x00FF0000) >> 8)  //addr16 to 1st Byte
#define BYTE2(addr) (((addr) & 0x0000FF00) << 8)  //addr8  to 2rd Byte
#define BYTE3(addr) (((addr) & 0x000000FF) << 24) //addr0  to 3th Byte

uint8_t qspi_set_write_status(wnQspiT *QSPI, uint8_t cmd, uint8_t data)
{
    static SPI_CTRL_T ctrl;
    uint8_t status;
                        
    qspi_write_enable(QSPI);    
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);
    } while (!(status & 0x02));
            
    ctrl.reg           = 0;
            
    ctrl.SPI_TxRxCount = 2;
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 1;
    ctrl.SDR_DDR       = 0;
		    
    QSPI->SPI_TX_DATA.reg = (data << 8 | cmd);
    QSPI->SPI_CTRL.reg    = ctrl.reg;
        
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);
    } while (status & 0x01);
  
    return 0;
}

uint8_t qspi_get_read_status(wnQspiT *QSPI, uint8_t status_reg)
{
    static SPI_CTRL_T ctrl;
    uint8_t status;
            
    ctrl.reg           = 0;
            
    ctrl.SPI_TxRxCount = 1;
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 0;
    ctrl.SDR_DDR       = 0;
	
    QSPI->SPI_TX_DATA.reg = status_reg; 
    QSPI->SPI_CTRL.reg    = ctrl.reg;
          		
    ctrl.SPI_TxRxCount = 1;
    ctrl.CMD_INIT      = CMD_RX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 1;
    ctrl.SDR_DDR       = 0;
	
    QSPI->SPI_CTRL.reg = ctrl.reg;
                        
    while (!QSPI->SPI_INT_STATUS.RX_BUF_THRES_INT) 
        ;
    
    status = QSPI->SPI_RX_DATA.reg;
                    	
    return status;	
}

uint32_t qspi_write_enable(wnQspiT *QSPI)
{
    SPI_CTRL_T ctrl;
    int status;
    
    if (QSPI == NULL)
		return QSPI_INVALID;
        
	ctrl.reg           = 0;
    
    ctrl.SPI_TxRxCount = 1;
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 1;
    ctrl.SDR_DDR       = 0;

    QSPI->SPI_TX_DATA.reg = OP_WRITE_ENABLE; 
    QSPI->SPI_CTRL.reg    = ctrl.reg;
        
    return QSPI_SUCCESS;
}

uint32_t qspi_erase(wnQspiT *QSPI, uint32_t addr, uint32_t size)
{
    SPI_CTRL_T ctrl;    
    uint8_t cmd;
    uint8_t status;    
    
    if (QSPI == NULL)
        return QSPI_INVALID;
        
	if ((addr + size - 1) >= 0x3FFFFF)
		return QSPI_INVALID;
    
	switch (size)
	{
		case Size_4K:
            cmd = SECTOR_ERASE_4K;
            break;		
		case Size_32K :
            cmd = SECTOR_ERASE_32K;
            break;		
		case Size_64K :
            cmd = SECTOR_ERASE_64K;
            break;		
		default:
            return QSPI_INVALID;
            break;
	}
        
	//enable WEL (Write Enable Latch) before erase
	qspi_write_enable(QSPI);    
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);
    } while (!(status & 0x02));

	ctrl.reg           = 0;
	
    ctrl.SPI_TxRxCount = 4;
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 1;    
    ctrl.SDR_DDR       = 0;
    
    QSPI->SPI_TX_DATA.reg = (BYTE3(addr) | BYTE2(addr) | BYTE1(addr) | cmd);    
    QSPI->SPI_CTRL.reg    = ctrl.reg;
           
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);    
    } while (status & 0x03);

    return QSPI_SUCCESS;
}

uint32_t qspi_cmd_thres(wnQspiT *QSPI, uint32_t dwTxCount, uint32_t dwRxCount)
{
    SPI_CMD_THRES_T SPI_CMD_THRES;
    
	if (QSPI == NULL)
		return QSPI_INVALID;
	
	SPI_CMD_THRES.reg = 0;
    SPI_CMD_THRES.RX_CMD_THRES = dwRxCount; // no of dwords ( dwRxCount * 4  bytes)
	SPI_CMD_THRES.TX_CMD_THRES = dwTxCount; // no of dwords ( dwTxCount * 4  bytes)
    
    QSPI->SPI_CMD_THRES.reg = SPI_CMD_THRES.reg;
	
	return QSPI_SUCCESS;
}

uint32_t qspi_int_thres(wnQspiT *QSPI, uint32_t dwTxCount, uint32_t dwRxCount)
{
    SPI_INT_THRES_T SPI_INT_THRES;
    
	if (QSPI == NULL)
		return QSPI_INVALID;
	
	SPI_INT_THRES.reg = 0;
    SPI_INT_THRES.RX_INT_THRES = dwRxCount;  // no of dwords ( dwRxCount * 4  bytes)
	SPI_INT_THRES.TX_INT_THRES = dwTxCount;  // no of dwords ( dwRxCount * 4  bytes)
    
    QSPI->SPI_INT_THRES.reg = SPI_INT_THRES.reg;
	
	return QSPI_SUCCESS;
}

uint32_t qspi_putchar(wnQspiT *QSPI, uint32_t addr, uint8_t ch)
{
    SPI_CTRL_T ctrl;
    uint8_t status;
    
	if (QSPI == NULL)
		return QSPI_INVALID;
        
    qspi_write_enable(QSPI);
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);
    } while (!(status & 0x02));

    ctrl.reg           = 0;
    
    ctrl.SPI_TxRxCount = 4;
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 0;
    ctrl.SDR_DDR       = 0;
    
    QSPI->SPI_TX_DATA.reg = (BYTE3(addr) | BYTE2(addr) | BYTE1(addr) | OP_PAGE_PROGRAM);
    QSPI->SPI_CTRL.reg    = ctrl.reg; 

    ctrl.SPI_TxRxCount = 1;
    ctrl.Chip_deassert = 1;
            
    QSPI->SPI_TX_DATA.reg = ch;
    QSPI->SPI_CTRL.reg    = ctrl.reg;
    
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);
    } while (status & 0x01);

    return QSPI_SUCCESS;
}


uint32_t qspi_page_program(wnQspiT *QSPI, uint32_t addr, uint32_t count, uint8_t *src, uint8_t spiLane)
{
    SPI_CTRL_T ctrl;
    uint8_t status;
    
	if (QSPI == NULL)
		return QSPI_INVALID;
        		       
    for (int i = 0; i < count; i++) {
        qspi_putchar(QSPI, addr + i, src[i]);
    }
        
    return QSPI_SUCCESS;
}

#if 0
uint32_t qspi_page_program(wnQspiT *QSPI, uint32_t addr, uint32_t count, uint8_t *src, uint8_t spiLane)
{
    SPI_CTRL_T ctrl;
    uint8_t status;
    
	if (QSPI == NULL)
		return QSPI_INVALID;
        
    qspi_write_enable(QSPI);

    ctrl.reg = 0;
    
    ctrl.SPI_TxRxCount = 4;
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = spiLane;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 0;
    ctrl.SDR_DDR       = 0;
    
    QSPI->SPI_TX_DATA.reg = (BYTE3(addr) | BYTE2(addr) | BYTE1(addr) | OP_PAGE_PROGRAM);
    QSPI->SPI_CTRL.reg = ctrl.reg; 
    
    int tcount = count / 4;
    
    ctrl.SPI_TxRxCount = 256;
    ctrl.Chip_deassert = 1;

    uint32_t *d = (uint32_t *)src;

    for (int i = 0; i < tcount; i++) {               
        QSPI->SPI_TX_DATA.reg = d[i];
    }
    
    QSPI->SPI_CTRL.reg  = ctrl.reg;
    
    do {
        status = qspi_get_read_status(QSPI, OP_READ_STATUS_REGISTER_1);
    } while (status & 0x01);

    return QSPI_SUCCESS;
}
#endif


#if 0
uint32_t qspi_page_program(wnQspiT *QSPI, uint32_t addr, uint32_t count, uint8_t *src, uint8_t spiLane)
{
    SPI_CTRL_T ctrl;
    uint8_t status;
    
	if (QSPI == NULL)
		return QSPI_INVALID;
        		       
    for (int i = 0; i < count; i++) {
        qspi_putchar(QSPI, addr + i, src[i]);
    }
        
    return QSPI_SUCCESS;
}
#endif

uint32_t qspi_write(wnQspiT *QSPI, uint32_t addr, uint32_t count, uint8_t *src, uint8_t spiLane)
{
	uint32_t size;
	uint32_t page_offset;
    
    if (QSPI == NULL)
		return QSPI_INVALID;

    if ((addr + count) >= 0x400000 )   // 4MB
		return QSPI_INVALID;
       
	do
	{	            
        page_offset = addr & 0xFF;
        size = PAGE_SIZE - page_offset;
        
        if (count <= size) size = count;
        
        qspi_page_program(QSPI, addr, size, src, LANE_SINGLE);
        
		count = (count - size);
		addr  = (addr + size);
		src   = (src + size);			
	} while (count > 0);	
    		
	return QSPI_SUCCESS;
}

uint32_t qspi_read(wnQspiT *QSPI, uint32_t addr, uint32_t count, uint8_t *dest, uint8_t spiLane)
{
    SPI_CTRL_T ctrl;
    
    if (QSPI == NULL)
        return QSPI_INVALID;
                       
    ctrl.reg           = 0;	
    
    ctrl.SPI_TxRxCount = 4 + (spiLane == 2 ? 1 : 0); 
    ctrl.CMD_INIT      = CMD_TX;
    ctrl.SPI_lane_mode = LANE_SINGLE;
    ctrl.SPI_DEV_SEL1  = 0;
    ctrl.SPI_DEV_SEL2  = 0;
    ctrl.Chip_deassert = 0;
    ctrl.SDR_DDR       = 0;
          
    QSPI->SPI_CTRL.reg = ctrl.reg; 
    
    uint32_t data = (BYTE3(addr) | BYTE2(addr) | BYTE1(addr) | (spiLane == 2 ? OP_FAST_READ_QUAD : OP_READ_DATA));    
    QSPI->SPI_TX_DATA.reg = data;
    
    //Send extra 1 dummy bytes for OP_FAST_READ_QUAD(0x6B) opcode
    if (spiLane == 2) {
        QSPI->SPI_TX_DATA.reg = 0;        
    }

    ctrl.SPI_TxRxCount = 1;
    ctrl.CMD_INIT      = CMD_RX;
    ctrl.SPI_lane_mode = spiLane;    	  
		       
    for (int i = 0; i < count; i++)
    {	        
        ctrl.Chip_deassert = 0;

        if (i >= (count - 1)) {
            ctrl.Chip_deassert = 1;            
        }

        QSPI->SPI_CTRL.reg = ctrl.reg;
                        
        while (!QSPI->SPI_INT_STATUS.RX_BUF_THRES_INT) 
        //while (!QSPI->SPI_INT_STATUS.RX_BUF_EMPTY) 
			;
                
        dest[i] = QSPI->SPI_RX_DATA.reg;
    }
    		
    return QSPI_SUCCESS;
}

uint32_t qspi_clk_init(wnQspiT *QSPI, uint16_t qspiClkDivisorValue)
{    
    SPI_CLK_CTRL_T clk;

	if (QSPI == NULL)
		return QSPI_INVALID;
			            
    clk.reg = 0;
            
    clk.InternalClockEnable = 1;
    clk.ClockDivisorValue   = qspiClkDivisorValue;
        
    QSPI->SPI_CLK_CTRL.reg = clk.reg;
    
    while (!QSPI->SPI_CLK_CTRL.InternalClockStable)
        ;
		
	return QSPI_SUCCESS;
}

static uint32_t qspi_config_mode_pio(wnQspiT *QSPI, uint8_t mode, uint8_t spiLane, uint16_t qspiClkDivisorValue)
{    
	SPI_CONFIG_T config;

    if (QSPI == NULL)
        return QSPI_INVALID;

	config.reg        = 0;
	
    config.ModeSelect = MODE_PIO;
    //Mode3
    config.CPHA       = 1;   
    config.CPOL       = 1; 
    //mode0
    //config.CPHA       = 0;   
    //config.CPOL       = 0;   
    
    config.LSBF       = 0;   
    config.WP         = 0;
    config.HOLD       = 0;
    config.SOFT_RST   = 0;
    config.SPI_LANES  = LANE_QUAD;  // enable SPI_OUT_PIN_EN[3:0] output pins
    config.SPI_EN     = 1;
    config.SPI_CS     = 1;

    QSPI->SPI_CONFIG.reg = config.reg;
	
    // Set CMD Register thrashold values
	qspi_cmd_thres(QSPI, 256, 1);
    
    // Set Tx/Rx Registers thrashold values
    qspi_int_thres(QSPI, 256, 1);
        
	if (qspi_clk_init(QSPI, qspiClkDivisorValue) == QSPI_INVALID)
		return QSPI_INVALID;
    
    QSPI->SPI_INT_ENABLE.reg = 0;
    
    SPI_INT_ENABLE_T ints;
    
    ints.TX_BUF_EMPTY        = 1;
	ints.TX_BUF_FULL         = 0;
	ints.TX_BUF_THRES_INT    = 1;
	ints.RX_BUF_EMPTY        = 1;
	ints.RX_BUF_FULL         = 0;
    ints.RX_BUF_THRES_INT    = 1;    
	ints.CTRL_BUF_FULL_INT   = 0;
	ints.CTRL_BUF_EMPTY_INT  = 1;
	ints.CTRL_BUF_THRESH_INT = 0;
    QSPI->SPI_INT_ENABLE.reg = ints.reg;
        
    return QSPI_SUCCESS;
}

uint32_t qspi_mode_reset(wnQspiT *QSPI, uint8_t mode)
{
    uint32_t status = QSPI_SUCCESS;
    SPI_CONFIG_T config;
    
    if (QSPI == NULL)
        return QSPI_INVALID;
		
    config.reg        = QSPI->SPI_CONFIG.reg;
            
    config.ModeSelect = mode;
    config.SPI_EN     = 1;
    config.SPI_CS     = 1;
    config.SOFT_RST   = 1;

    QSPI->SPI_CONFIG.reg = config.reg;
		
	while (QSPI->SPI_CONFIG.SOFT_RST)
		;
	
	return QSPI_SUCCESS;
}

uint32_t qspi_init(wnQspiT *QSPI, uint8_t mode, uint8_t spiLane, uint16_t qspiClkDivisorValue)
{	
	if (QSPI == NULL)
		return QSPI_INVALID;
	
	qspi_mode_reset(QSPI, mode);
			
    if (qspi_config_mode_pio(QSPI, mode, spiLane, qspiClkDivisorValue) == QSPI_INVALID) 
        return QSPI_INVALID;
                	
	return QSPI_SUCCESS;
}
