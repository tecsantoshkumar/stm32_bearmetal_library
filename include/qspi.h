#ifndef __QSIP_REG_H__
#define __QSIP_REG_H__

#include <stdint.h>

enum {
    MODE_BOOT = 0,
    MODE_PIO  = 1,
    MODE_DMA  = 2 ,
    MODE_XIP  = 3
};

enum {
	CMD_IDLE = 0,
	CMD_TX   = 1,
	CMD_RX   = 2,
};

enum {
	LANE_SINGLE = 0,
	LANE_DOUBLE = 1,
	LANE_QUAD   = 2
};

//#define CONFIG_SFC_CLK_38_4_MHZ		0x2
//#define CONFIG_SFC_LANES_4            0x2

#define PAGE_SIZE					 256

#define OP_READ_MANUF_DEVICE_ID 	 0x90
#define OP_WRITE_ENABLE  			 0x06
#define OP_PAGE_PROGRAM  			 0x02
#define OP_READ_DATA  				 0x03
#define OP_SECTOR_ERASE 			 0x20
#define OP_FAST_READ_DATA  			 0x0B
#define OP_FAST_READ_QUAD  			 0x6B

#define OP_READ_STATUS_REGISTER_1    0x05
#define OP_READ_STATUS_REGISTER_2    0x35
#define OP_READ_STATUS_REGISTER_3    0x15

#define OP_WRITE_STATUS_REGISTER_1   0x01
#define OP_WRITE_STATUS_REGISTER_2   0x31
#define OP_WRITE_STATUS_REGISTER_3   0x11


#define SECTOR_ERASE_4K				0x20
#define SECTOR_ERASE_32K 			0x52
#define SECTOR_ERASE_64K			0xD8

#define Size_4K 					0x1000
#define Size_32K					0x8000
#define Size_64K 					0x10000

#define SPI_LANE_SINGLE				0x0
#define SPI_LANE_DUAL				0x1
#define SPI_LANE_QUAD				0x2

#define SPI_MODE_BOOT 				0x0 
#define SPI_MODE_PIO 				0x1
#define SPI_MODE_DMA				0x2
#define SPI_MODE_XIP				0x3

//-------------- Register BEGIN ------------------------------------------

typedef union {
	struct {
		uint32_t XIP_SPI_TYPE_CMD:2;
		uint32_t XIP_SPI_TYPE_ADDR:2;
		uint32_t XIP_SPI_TYPE_MODE:2;
		uint32_t XIP_SPI_TYPE_DUMMY:2;
		uint32_t XIP_SPI_TYPE_DATA:2;
		uint32_t XIP_SPI_READ_OPCODE:8;
		uint32_t XIP_READ_ADDR_BYTES:3;
		uint32_t XIP_DUMMY_BYTES:3;
		uint32_t XIP_SPI_TYPE_CMD_DDR:1;
		uint32_t XIP_SPI_TYPE_ADDR_DDR:1;
		uint32_t XIP_SPI_TYPE_MODE_DDR:1;
		uint32_t XIP_SPI_TYPE_DUMMY_DDR:1;
		uint32_t XIP_SPI_TYPE_DATA_DDR:1;
		uint32_t XIP_SPI_TYPE_CMD_SDR2DDR:1;
		uint32_t Reserved:2;
	};
	uint32_t reg;
} XIP_CTRL1_T;

typedef union {
	struct {
		uint32_t XIP_SPI_MODE_CODE:8;
		uint32_t XIP_MODE_BYTES:2;
		uint32_t XIP_SPI_DEV_SEL:3;
		uint32_t XIP_ecc_on_off:1;
		uint32_t XIP_ecc_offset:8;
		uint32_t XIP_SPI_NAND:1;
		uint32_t XIP_SPI_PLANE:2;
		uint32_t XIP_PAGE_SIZE:3;
		uint32_t Reserved:4;	
	};
	uint32_t reg;
} XIP_CTRL2_T;

typedef union {
    struct {
        uint32_t ModeSelect:3;
		uint32_t CPHA:1;
		uint32_t CPOL:1;
		uint32_t LSBF:1;
		uint32_t Reserved0:3;
		uint32_t WP:1;
		uint32_t HOLD:1;
		uint32_t AHB_BURST_INCR_EN:1;
		uint32_t AHB_BURST_INCR4_EN:1;
		uint32_t AHB_BURST_INCR8_EN:1;
		uint32_t AHB_BURST_INCR16_EN:1;
		uint32_t Reserved1:1;
		uint32_t SOFT_RST:1;
		uint32_t Reserved3:3;
		uint32_t SPI_LANES:2;
		uint32_t Reserved4:1;
		uint32_t SPI_EN:1;
		uint32_t SPI_CS:8;
    };
    uint32_t reg;
} SPI_CONFIG_T;

typedef union {
    struct {
        uint32_t SPI_TxRxCount:16;
		uint32_t CMD_INIT:2;
		uint32_t SPI_lane_mode:2;
		uint32_t SPI_DEV_SEL1:2;
		uint32_t Chip_deassert:1;
		uint32_t SDR_DDR:1;
		uint32_t StatusCheck:1;
		uint32_t SPI_DEV_SEL2:1;
		uint32_t ECC_CAL_EN:1;
		uint32_t ECC_ACCESS_EN:1;
		uint32_t RX_ECC_CAL_START:1;
		uint32_t Reserved:3;
    };
    uint32_t reg;
} SPI_CTRL_T;

typedef union {
    struct {
        uint32_t InternalClockEnable:1;
		uint32_t InternalClockStable:1;
		uint32_t Reserved1:6;
		uint32_t ClockDivisorValue:11;
		uint32_t Reserved2:13;
    };
    uint32_t reg;
} SPI_CLK_CTRL_T;


typedef union {
    struct {
        uint32_t RX_CMD_THRES:16;
		uint32_t TX_CMD_THRES:16;
    };
    uint32_t reg;
} SPI_CMD_THRES_T;

typedef union {
    struct {
        uint32_t RX_INT_THRES:16;
		uint32_t TX_INT_THRES:16;
    };
    uint32_t reg;
} SPI_INT_THRES_T;

typedef union {
    struct {
        uint32_t TX_BUF_EMPTY:1;
		uint32_t TX_BUF_FULL:1;
		uint32_t TX_BUF_THRES_INT:1;
		uint32_t RX_BUF_EMPTY:1;
		uint32_t RX_BUF_FULL:1;
		uint32_t RX_BUF_THRES_INT:1;
		uint32_t CTRL_BUF_FULL_INT:1;
		uint32_t CTRL_BUF_EMPTY_INT:1;
		uint32_t CTRL_BUF_THRESH_INT:1;
		uint32_t bdp_csr_cbd_int:1;
		uint32_t bdp_csr_pkt_int:1;
		uint32_t mas_err_int_en:1;
		uint32_t Corr_err_int_en:1;
		uint32_t Uncorr_err_int_en:1;
		uint32_t Reserved:18;
    };
    uint32_t reg;
} SPI_INT_ENABLE_T;

typedef union {
    struct {
        uint32_t TX_BUF_EMPTY:1;
		uint32_t TX_BUF_FULL:1;
		uint32_t TX_BUF_THRES_INT:1;
		uint32_t RX_BUF_EMPTY:1;
		uint32_t RX_BUF_FULL:1;
		uint32_t RX_BUF_THRES_INT:1;
		uint32_t CTRL_BUF_FULL_INT:1;
		uint32_t CTRL_BUF_EMPTY_INT:1;
		uint32_t CTRL_BUF_THRESH_INT:1;
		uint32_t bdp_csr_cbd_int:1;
		uint32_t bdp_csr_pkt_int:1;
		uint32_t mas_err_int_en:1;
		uint32_t Corr_err_int_en:1;
		uint32_t Uncorr_err_int_en:1;
		uint32_t Reserved:18;
    };
    uint32_t reg;
} SPI_INT_STATUS_T;

typedef union {
    struct {
        uint32_t TX_DATA:32;
    };
    uint32_t reg;
} SPI_TX_DATA_T;

typedef union {
    struct {
        uint32_t RX_DATA:32;
    };
    uint32_t reg;
} SPI_RX_DATA_T;


typedef union {
    struct {
        uint32_t RX_FIFO_CNT:16;
		uint32_t TX_FIFO_FREE:16;
    };
    uint32_t reg;
} SPI_STATUS1_T;

typedef union {
    struct {
        uint32_t TX_FIFO_OVF:1;
		uint32_t RX_FIFO_UNF:1;
		uint32_t Reserved1:1;
		uint32_t SPI_IO0_IO3:4;
		uint32_t CBUF_DATA_AVAIL:9;
		uint32_t CMD_INIT:2;
		uint32_t Reserved2:14;
    };
    uint32_t reg;
} SPI_STATUS2_T;

typedef union {
    struct {
        uint32_t csr_dma_en:1;
		uint32_t csr_bdp_poll_cntr_en:1;
		uint32_t csr_bdp_ch_start_wstb:1;
		uint32_t Reserved:29;
    };
    uint32_t reg;
} BD_CTRL_T;

typedef union {
    struct {
        uint32_t bdp_curr_bd_addr:32;
    };
    uint32_t reg;
} BD_CURR_ADDR_T;

typedef union {
    struct {
        uint32_t csr_bdp_bd_addr:32;
    };
    uint32_t curr_addr;
} BD_BASE_ADDR_T;

typedef union {
    struct {
        uint32_t bdp_csr_bdctrl:16;
		uint32_t bdp_csr_dma_actv:1;
		uint32_t bdp_csr_dma_start:1;
		uint32_t bdp_csr_state:4;
		uint32_t reserved:10;
    };
    uint32_t bd_status;
} BD_STATUS_T;

typedef union {
    struct {
        uint32_t csr_bdp_poll_cntr:16;
		uint32_t reserved:16;
    };
    uint32_t bd_poll_ctrl;
} BD_POLL_CTRL_T;

typedef union {
    struct {
        uint32_t dma_csr_tx_curr_buflen:16;
		uint32_t dma_csr_txbuf_spc_avail:16;
    };
    uint32_t bd_tx_dma_status;
} BD_TX_DMA_STATUS_T;

typedef union {
    struct {
        uint32_t dma_csr_rx_curr_buflen:16;
		uint32_t dma_csr_rxbuf_spc_avail:16;
    };
    uint32_t bd_rx_dma_status;
} BD_RX_DMA_STATUS_T;

typedef union {
    struct {
        uint32_t CNTRL_THRES:1;
		uint32_t Reserved:31;
    };
    uint32_t reg;
} SPI_CTRL_THRES_T;


typedef union {
    struct {
        uint32_t TX_BUF_EMPTY:1;
		uint32_t TX_BUF_FULL:1;
		uint32_t TX_BUF_THRES_INT:1;
		uint32_t RX_BUF_EMPTY:1;
		uint32_t RX_BUF_FULL:1;
		uint32_t RX_BUF_THRES_INT:1;
		uint32_t CTRL_BUF_FULL_INT:1;
		uint32_t CTRL_BUF_EMPTY_INT:1;
		uint32_t CTRL_BUF_THRESH_INT:1;
		uint32_t bdp_csr_cbd_int:1;
		uint32_t bdp_csr_pkt_int:1;
		uint32_t mas_err_int:1;
		uint32_t Corr_err_int_en:1;
		uint32_t Uncorr_err_int_en:1;
		uint32_t Reserved:18;
    };
    uint32_t reg;
} SPI_INT_SIG_ENABLE_T;

typedef union {
    struct {
        uint32_t CLK_OUT_DLY_CNT:6;
		uint32_t DATA_OUT_DLY_CNT:6;
		uint32_t CLK_IN_DLY_CNT:6;
		uint32_t Reserved:14;
    };
    uint32_t spi_tap_ctrl;
} SPI_TAP_CTRL_T;

typedef union {
    struct {
        uint32_t StatusData:16;
		uint32_t NoOfBytesToSend:2;
		uint32_t StatusLane:2;
		uint32_t STS_BUSY_BIT0_BIT7_VALID:1;
		uint32_t Reserved:11;
    };
    uint32_t spi_status_ctrl;
} SPI_STATUS_CTRL_T;

typedef union {
    struct {
        uint32_t XIP_SPI_INIT1_CODE1:8;
		uint32_t XIP_SPI_INIT1_CODE2:8;
		uint32_t XIP_SPI_INIT1_CODE3:8;
		uint32_t XIP_SPI_TYPE_INIT1:2;
		uint32_t XIP_SPI_INIT1_CNT:2;
		uint32_t XIP_SPI_INIT1_STS_CHK:1;
		uint32_t Reserved:3;
    };
    uint32_t spi_xip_ctrl3;
} SPI_XIP_CTRL3_T;

typedef union {
    struct {
        uint32_t XIP_SPI_INIT2_CODE1:8;
		uint32_t XIP_SPI_INIT2_CODE2:8;
		uint32_t XIP_SPI_INIT2_CODE3:8;
		uint32_t XIP_SPI_TYPE_INIT2:2;
		uint32_t XIP_SPI_INIT2_CNT:2;
		uint32_t XIP_SPI_INIT2_STS_CHK:1;
		uint32_t Reserved:3;
    };
    uint32_t spi_xip_ctrl4;
} SPI_XIP_CTRL4_T;

typedef union {
    struct {
        uint32_t bch_n:16;
		uint32_t Bch_t:4;
		uint32_t code_size:4;
		uint32_t Reserved:8;
    };
    uint32_t reg;
} SPI_BCH_CTRL0_T;

typedef union {
    struct {
        uint32_t bch_err_cnt:6;
		uint32_t Bch_page_err_cnt:12;
		uint32_t Reserved:14;
    };
    uint32_t spi_bch_status;
} SPI_BCH_STATUS_T;


typedef union {
    struct {
        uint32_t XIP_SPI_INIT1_WAIT_CNT:16;
		uint32_t XIP_SPI_READ_CACHE_OPCODE:8;
		uint32_t XIP_ECC_SPARE_CNT:8;
    };
    uint32_t spi_xip_ctrl5;
} SPI_XIP_CTRL5_T;


typedef union {
    struct {
        uint32_t BOOT_SPI_EN_CPHA:1;
		uint32_t BOOT_SPI_EN_CPOL:1;
		uint32_t BOOT_SPI_EN_LSBF:1;
		uint32_t Reserved1:1;
		uint32_t BOOT_WP:1;
		uint32_t BOOT_HOLD:1;
		uint32_t BOOT_SPI_CLK_FREQ_SEL:11;
		uint32_t BOOT_DEVICE_SELECT:3;
		uint32_t BOOT_SPI_EN:1;
		uint32_t BOOT_SPI_LANES:2;
		uint32_t BOOT_SPI_MODE_CODE:8;
		uint32_t Reserved2:1;
		
    };
    uint32_t spi_bstrap_reg1;
} SPI_BSTRAP_REG1_T;

typedef union {
    struct {
        uint32_t BOOT_SPI_TYPE_CMD:2;
		uint32_t BOOT_SPI_TYPE_ADDR:2;
		uint32_t BOOT_SPI_TYPE_MODE:2;
		uint32_t BOOT_SPI_TYPE_DUMMY:2;
		uint32_t BOOT_SPI_TYPE_DATA:2;
		uint32_t BOOT_SPI_TYPE_INIT1:2;
		uint32_t BOOT_SPI_TYPE_INIT2:2;
		uint32_t BOOT_SPI_READ_OPCODE:8;
		uint32_t BOOT_READ_ADDR_CYCLE:3;
		uint32_t BOOT_DUMMY_BYTES:3;
		uint32_t BOOT_MODE_BYTES:2;
		uint32_t Reserved:2;		
    };
    uint32_t spi_bstrap_reg2;
} SPI_BSTRAP_REG2_T;

typedef union {
    struct {
        uint32_t BOOT_SPI_INIT1_CODE1:8;
		uint32_t BOOT_SPI_INIT1_CODE2:8;
		uint32_t BOOT_SPI_INIT1_CODE3:8;
		uint32_t BOOT_SPI_INIT1_CNT:2;
		uint32_t BOOT_SPI_INIT1_STS_CHK:1;
		uint32_t BOOT_CODE_WORD_SIZE:4;
		uint32_t BOOT_SPI_TYPE_CMD_SDR2DDR:1;
    };
    uint32_t spi_bstrap_reg3;
} SPI_BSTRAP_REG3_T;

typedef union {
    struct {
        uint32_t BOOT_SPI_INIT2_CODE1:8;
		uint32_t BOOT_SPI_INIT2_CODE2:8;
		uint32_t BOOT_SPI_INIT2_CODE3:8;
		uint32_t BOOT_SPI_INIT2_CNT:2;
		uint32_t BOOT_SPI_INIT2_STS_CHK:1;
		uint32_t BOOT_PAGE_SIZE:3;
		uint32_t BOOT_STS_MODE:2;
    };
    uint32_t spi_bstrap_reg4;
} SPI_BSTRAP_REG4_T;

typedef union {
    struct {
        uint32_t BOOT_SPI_TYPE_CMD_DDR:1;
		uint32_t BOOT_SPI_TYPE_ADDR_DDR:1;
		uint32_t BOOT_SPI_TYPE_MODE_DDR:1;
		uint32_t BOOT_SPI_TYPE_DUMMY_DDR:1;
		uint32_t BOOT_SPI_TYPE_DATA_DDR:1;
		uint32_t BOOT_SPI_CS:8;
		uint32_t BOOT_STS_DATA:16;
		uint32_t BOOT_STS_CNT:2;
		uint32_t BOOT_STS_BUSY_BIT0_BIT7_VALID:1;
    };
    uint32_t spi_bstrap_reg5;
} SPI_BSTRAP_REG5_T;

typedef union {
    struct {
        uint32_t BOOT_ECC_ON_OFF:1;
		uint32_t BOOT_ECC_OFFSET:8;
		uint32_t BOOT_BCH_T:4;
		uint32_t BOOT_BCH_N:16;
		uint32_t BOOT_SPI_NAND:1;
		uint32_t BOOT_SPI_PLANE:2;
		uint32_t BOOT_STS_DATA:16;
    };
    uint32_t spi_bstrap_reg6;
} SPI_BSTRAP_REG6_T;

typedef union {
    struct {
        uint32_t BOOT_SPI_INIT1_WAIT_CNT:16;
		uint32_t BOOT_SPI_READ_CACHE_OPCODE:8;
		uint32_t BOOT_ECC_SPARE_CNT:8;
    };
    uint32_t spi_bstrap_reg7;
} SPI_BSTRAP_REG7_T;

typedef union {
    struct {
        uint32_t MajorRevisionNumber:8;
		uint32_t MinorRevisionNumber:8;
		uint32_t MaintanenceRevisionNumber:8;
		uint32_t Reserved:8;
    };
    uint32_t spi_rev_id_reg;
} SPI_REV_ID_REG_T;

//-------------- Register END ------------------------------------------

typedef enum wnQspi {
	QSPI_INVALID   = -1,
	QSPI_SUCCESS   = 0,
	QSPI_IS_READY  = 1,
	QSPI_IS_BUSY   = 2
} wnQspiE;


typedef struct {		
	XIP_CTRL1_T          XIP_CTRL1;
	XIP_CTRL2_T          XIP_CTRL2;
	SPI_CONFIG_T         SPI_CONFIG;
	SPI_CTRL_T           SPI_CTRL;
	SPI_CLK_CTRL_T       SPI_CLK_CTRL;
	SPI_CMD_THRES_T      SPI_CMD_THRES;
	SPI_INT_THRES_T      SPI_INT_THRES;
	SPI_INT_ENABLE_T     SPI_INT_ENABLE;
	SPI_INT_STATUS_T     SPI_INT_STATUS;
	SPI_TX_DATA_T        SPI_TX_DATA;
	SPI_RX_DATA_T        SPI_RX_DATA;
	SPI_STATUS1_T        SPI_STATUS1;
	SPI_STATUS2_T        SPI_STATUS2;
	BD_CTRL_T            BD_CTRL;
	BD_CURR_ADDR_T       BD_CURR_ADDR;
	uint32_t             Reserved0;
	BD_BASE_ADDR_T       BD_BASE_ADDR;
	BD_STATUS_T          BD_STATUS;
	BD_STATUS_T          BD_POLL_CTRL;
	BD_TX_DMA_STATUS_T   BD_TX_DMA_STATUS;
	BD_RX_DMA_STATUS_T   BD_RX_DMA_STATUS;
	SPI_CTRL_THRES_T   SPI_CTRL_THRES;
	SPI_INT_SIG_ENABLE_T SPI_INT_SIG_ENABLE;
	SPI_TAP_CTRL_T       SPI_TAP_CTRL;
	SPI_STATUS_CTRL_T    SPI_STATUS_CTRL;
	SPI_XIP_CTRL3_T      SPI_XIP_CTRL3;
	SPI_XIP_CTRL4_T      SPI_XIP_CTRL4;
	SPI_BCH_CTRL0_T      SPI_BCH_CTRL0;
	SPI_BCH_STATUS_T     SPI_BCH_STATUS;
	SPI_XIP_CTRL5_T      SPI_XIP_CTRL5;
	uint32_t             Reserved1;
	uint32_t             Reserved2;
	SPI_BSTRAP_REG1_T    SPI_BSTRAP_REG1;
	SPI_BSTRAP_REG2_T    SPI_BSTRAP_REG2;
	SPI_BSTRAP_REG3_T    SPI_BSTRAP_REG3;
	SPI_BSTRAP_REG4_T    SPI_BSTRAP_REG4;
	SPI_BSTRAP_REG5_T    SPI_BSTRAP_REG5;
	SPI_BSTRAP_REG6_T    SPI_BSTRAP_REG6;
	SPI_BSTRAP_REG7_T    SPI_BSTRAP_REG7;
	SPI_REV_ID_REG_T     SPI_REV_ID_REG;
} QSPI_T;



uint32_t qspi_write_enable(QSPI_T *QSPI);
uint8_t qspi_get_read_status(QSPI_T *QSPI, uint8_t status_reg);
uint8_t qspi_set_write_status(QSPI_T *QSPI, uint8_t write_reg, uint8_t data);

uint32_t qspi_init(QSPI_T *QSPI, uint8_t mode, uint8_t spiLane, uint16_t qspiClkDivisorValue);
uint32_t qspi_clk_init(QSPI_T *QSPI, uint16_t qspiClkDivisorValue);
uint32_t qspi_mode_reset(QSPI_T *QSPI, uint8_t mode);
uint32_t qspi_read(QSPI_T *QSPI, uint32_t addr, uint32_t count, uint8_t *dest, uint8_t spiLane);
uint32_t qspi_write(QSPI_T *QSPI, uint32_t addr, uint32_t count, uint8_t *src, uint8_t spiLane);
uint32_t qspi_page_program(QSPI_T *QSPI, uint32_t addr, uint32_t byteCount, uint8_t *src, uint8_t spiLane);
uint32_t qspi_erase(QSPI_T *QSPI, uint32_t addr, uint32_t size);
uint32_t qspi_cmd_thres(QSPI_T *QSPI, uint32_t dwTxCount, uint32_t dwRxCount);
uint32_t qspi_int_thres(QSPI_T *QSPI, uint32_t dwTxCount, uint32_t dwRxCount);
uint32_t qspi_init1(QSPI_T *QSPI, uint8_t mode, uint8_t spiLane, uint16_t qspiClkDivisorValue);

#endif /* __QSIP_REG_H__ */

